#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <vector>
#include "DetachedPoleFilter.h"
#include "ParametricFIR.h"
#include "UserPresetOverrides.h"
#include "LiveSearchCache.h"

// BBK Parametric FIR: a single parametric constrained-least-squares FIR
// lowpass (see ParametricFIR.h for the design method). Three user-facing
// controls - cutoff, attenuation at cutoff, and minimum stopband rejection -
// are handed straight to bbk::parametric::designParametricFIR(), along with
// a Sidelobe Decay slider (FilterSpec::sidelobeDecayRatio). The plugin
// auto-detects the host sample rate (44.1/48/96/192 kHz and anything else
// the host reports) rather than hard-locking to 192 kHz.
//
// This plugin previously offered two extra design engines - a Prolate/DPSS
// basis-restricted variant of this same minimax LP, and a completely
// separate Peak-Energy Optimized engine (PeakEnergyFIR.h) - selectable live
// alongside this one, with a 3-way per-mode cache/queue so switching between
// them never re-paid a slow design. Direct A/B listening at 44.1 kHz found
// the plain minimax design here sounded best of the three, so both
// alternatives were removed entirely rather than continuing to carry their
// own bugs (a QP convergence failure in Peak-Energy at 192 kHz, and a
// background-thread contention bug the Peak-Energy time-budget increase
// introduced) - see ParametricFIR.h's own comment for where that effort was
// redirected instead: designParametricFIR() itself now searches far more
// thoroughly (more tap counts, more stopband-edge candidates per tap count,
// a much larger time budget) for the single best result, since this design
// now runs alone with nothing else competing for the background thread.
//
// A redesign never runs on the audio thread, and never blocks the host
// either - not even on a sample-rate change or the very first
// prepareToPlay(). Every redesign, including those, is handed to a
// dedicated background juce::Thread; prepareToPlay() installs a safe
// identity pass-through (pure delay, no filtering - see
// DetachedPoleFilter.h::identityTaps()) immediately and returns, and the
// audio thread crossfades smoothly from whatever was previously active into
// the newly completed design once the background thread finishes (same
// 15 ms linear crossfade mechanism this plugin has always used) so a
// redesign never clicks - and, just as importantly, prepareToPlay() itself
// always returns in milliseconds regardless of how long the actual design
// takes (a demanding spec can legitimately take minutes now that the search
// is deliberately much more thorough - see ParametricFIR.h - which is far
// too long for a host to wait on).
class BBKDetachedPoleAudioProcessor final : public juce::AudioProcessor,
                                             private juce::Thread
{
public:
    BBKDetachedPoleAudioProcessor();
    ~BBKDetachedPoleAudioProcessor() override;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    void processBlock (juce::AudioBuffer<double>&, juce::MidiBuffer&) override;
    bool supportsDoublePrecisionProcessing() const override { return false; }

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return "BBK Parametric FIR"; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    juce::AudioProcessorValueTreeState& getAPVTS() noexcept { return parameters; }

    // The host's actual current sample rate, updated unconditionally on
    // every prepareToPlay() call - deliberately independent of whether a
    // redesign happened, so the UI's sample-rate readout always reflects
    // what the host just reported rather than lagging behind until the
    // next completed design (see DesignSnapshot::sampleRateHz below,
    // which is a different thing: the rate a specific *design* was run
    // for, not necessarily the host's rate right now).
    double getCurrentSampleRateForUI() const noexcept { return currentSampleRate.load(); }

    // Millisecond timestamp (juce::Time::getMillisecondCounter() scale) of
    // the last audio block in which the WET signal actually exceeded the
    // soft-clip knee - i.e. the backstop actually engaged, regardless of
    // whether Auto Headroom is on. The editor compares this against "now"
    // to light a clip indicator for a short hold time. This exists because
    // the backstop is deliberately inaudible/gentle when it engages
    // rarely, which is the point for playback - but that same gentleness
    // means it can hide from the ear exactly when you're trying to tune
    // Headroom down manually by listening for clipping. The indicator
    // gives an objective signal instead of relying on hearing it.
    juce::uint32 getLastClipTimeMsForUI() const noexcept { return lastClipTimeMs.load(); }

    // True whenever a live background design - a Custom-mode search, or
    // Default mode's own fallback to one when the host's sample rate isn't
    // one of the compiled-in bank's 7 swept rates - is currently queued or
    // running for the boundary spec presently in effect, i.e. whatever the
    // UI is showing right now (see DesignSnapshot) is not yet the freshest
    // result being computed. Deliberately distinct from DesignSnapshot::
    // tapCount == 0, which means "no completed design AT ALL yet" (the
    // cold-start/sample-rate-change case, already shown via its own
    // "Designing filter for..." message in the editor) - this instead
    // covers the steady-state case where a PREVIOUS result is already
    // playing but a newer one is being computed because a boundary
    // parameter (cutoff, attenuation, stopband, sidelobe decay, Default
    // on/off, or the Manual/Auto tap-count selector) just changed. See
    // requestBoundaryRedesign() and run() for where this is set/cleared.
    bool isSearchInProgressForUI() const noexcept { return searchInProgress.load(); }

    // Where a published design actually came from - shown in the editor's
    // metrics readout and used to decide whether "Save as Default" is
    // meaningful (saving an already-instant result just re-saves the same
    // taps under the same spec, which is harmless but pointless).
    enum class ResultSource
    {
        LiveSearch,   // Custom mode: a fresh designParametricFIR() search
        PresetBank,   // Default mode: instant lookup in the compiled-in factory bank
        UserOverride, // instant lookup in a filter the user saved themselves (see saveCurrentAsOverride)
        SearchCache   // instant lookup in a Custom-mode result already searched earlier (see LiveSearchCache.h)
    };

    // A snapshot of the most recently completed design, safe to read from
    // the message thread at any time (used by the editor's metrics
    // readout and the "show coefficients" popup).
    struct DesignSnapshot
    {
        double sampleRateHz = 0.0;
        double cutoffHz = 0.0;
        double attenuationAtCutoffDb = 0.0;
        double stopbandRejectionDb = 0.0;
        bbk::parametric::StopbandMode stopbandMode = bbk::parametric::StopbandMode::FlatMask;
        double sidelobeDecayRatio = 1.0;
        bool amplitudeRelaxationOn = true;
        int tapCount = 0;
        double achievedStopbandDb = 0.0;
        bool constraintsMet = false;
        int designAttempts = 0;
        ResultSource source = ResultSource::LiveSearch;
        std::vector<double> taps; // the actual (unpadded) symmetric taps

        // Temporal-concentration metrics from the article ("Impulse-
        // Response Ringing in Digital Reconstruction Filtering"), computed
        // directly from the taps above by
        // bbk::parametric::computeTemporalMetrics() - see ParametricFIR.h
        // for exact definitions and the Case C reference validation.
        bbk::parametric::TemporalMetrics temporal;
    };
    DesignSnapshot getDesignSnapshotForUI() const;

    // Persists the currently-published design (whatever it is - a live
    // Custom-mode result, a Default-mode bank entry, or even an existing
    // override) as a user override for its own exact spec (sample rate,
    // cutoff, attenuation, stopband, decay - see UserPresetOverrides.h).
    // From then on, ANY time that exact spec recurs - in either Default or
    // Custom mode, this session or a future one - requestBoundaryRedesign()
    // finds and uses it instantly, ahead of both the compiled-in factory
    // bank and a fresh live search. A no-op if nothing has been designed
    // yet (tapCount == 0).
    void saveCurrentAsOverride();

private:
    void run() override; // juce::Thread - background redesign worker

    // How many concurrent attemptDesign() worker threads the background
    // search (see run()) is currently allowed to use - re-evaluated once
    // per search "round" via ParametricFIR.h's SearchConcurrencyHooks::
    // pollConcurrency, never decided once for the whole search, so it can
    // back off immediately if load rises mid-search and scale back up once
    // things are calm again. Combines a fixed ceiling
    // (maxSearchWorkerThreads in the .cpp), std::thread::
    // hardware_concurrency() minus cores always reserved for the OS/host/
    // audio thread, system-wide CPU load (see pollSystemCpuBusyFraction()
    // below), and the audio callback's own measured headroom
    // (audioCallbackLoadFraction below) - the harder, more directly-
    // relevant constraint of the two, since it reflects actual real-time
    // scheduling risk on the audio thread itself rather than a generic
    // system-wide number that might not translate into audio starvation
    // at all (or might, even when overall load looks moderate). Always
    // returns at least 1: a fully-loaded system still makes forward
    // progress on a redesign, just serially, exactly like this search has
    // always been capable of.
    int currentAllowedSearchConcurrency();

    // System-wide CPU utilization since the previous call, as a 0.0-1.0
    // fraction, via Windows' GetSystemTimes() (see the .cpp - the actual
    // Win32 call is kept out of this header so it stays a plain, always-
    // compilable declaration). Returns 0.0 ("assume idle") on the very
    // first call, when there is no previous sample yet to diff against,
    // and on any non-Windows build - this plugin only ships for Windows
    // (see README), but keeping a defined fallback rather than a
    // platform-specific build error costs nothing and avoids surprising
    // anyone who compiles this file elsewhere.
    double pollSystemCpuBusyFraction();

    template <typename SampleType>
    void process (juce::AudioBuffer<SampleType>& buffer);

    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    bbk::parametric::FilterSpec specFromParameters() const;

    // Boundary parameter changed (cutoff/attenuation/amplitude relaxation/
    // stopband/sidelobe decay, or a sample-rate change via prepareToPlay):
    // bumps boundaryEpoch, queues a fresh design on the background thread,
    // and discards any design already queued or mid-flight for an older
    // epoch (the same "only the newest request wins" principle this plugin
    // has always used).
    void requestBoundaryRedesign();

    // Bumps versionCounter and writes latestResult/latestSpec/
    // latestVersion/uiSnapshot exactly once, so the audio thread's existing
    // version-based crossfade pickup in process() always sees the newest
    // published design. source just tags the UI snapshot (see
    // DesignSnapshot) - it doesn't change how the result is applied.
    void publishResult (const bbk::parametric::FilterSpec& spec,
                         const bbk::parametric::DesignResult& result,
                         ResultSource source = ResultSource::LiveSearch);

    // Called when the "presetMode" parameter turns on (see ParamListener
    // below): forces cutoff/attenuation/stopband/sidelobeDecay to an
    // operating point, via setValueNotifyingHost - the same "processor
    // drives another parameter's value directly" pattern already used for
    // the Auto Headroom ratchet (see process()). Normally that's the fixed
    // point the preset bank was swept at (attenuation left alone, so the
    // slider still picks one of the 10 precomputed steps) - but if the
    // user has saved an override (see saveCurrentAsOverride()) for the
    // current sample rate, ALL FOUR are instead snapped to that override's
    // own values, attenuation included: "save as my default" means Default
    // should always recall that exact saved point, not just the same
    // cutoff/stopband/decay with whichever attenuation the slider happens
    // to be sitting on. If more than one override exists for this rate,
    // the most recently saved one wins (see saveCurrentAsOverride()'s
    // ordering). This makes the greyed-out sliders in Default mode show
    // the values that are actually in effect, rather than whatever
    // Custom-mode position they were last left at, and means
    // specFromParameters() never needs its own separate preset/custom
    // branch - it just always reads whatever the parameters currently
    // hold.
    void forcePresetOperatingPoint();

    // Called from forcePresetOperatingPoint()'s own start, the instant
    // Default mode turns on: snapshots cutoff/attenuation/stopband/decay
    // AS THEY WERE just before forcePresetOperatingPoint() overwrites them,
    // so restoreCustomPointBeforeDefault() below can put the user's actual
    // Custom-mode point back once Default is unchecked again. Without this,
    // those four parameters simply stayed at whatever Default forced them
    // to forever - so "unchecking Default" looked like it did nothing
    // (still the Default spec, just now unlabelled as one), and any
    // redesign that DID fire was for that same Default spec, not the
    // user's own last Custom entry - which also meant it was never a
    // search-cache hit (see LiveSearchCache.h), so it silently re-ran a
    // full live search from scratch instead of recalling anything,
    // breaking A/B comparison between a Custom find and Default entirely.
    // Guarded against re-capturing on a resent "on" event (e.g. some hosts
    // resend automation at the playhead on transport start - see
    // requestBoundaryRedesign()'s own comment on the same pattern): only
    // captures when currentBoundaryPresetMode is still false, i.e. this is
    // a genuine off->on transition, not a repeat of the same state.
    void captureCustomPointBeforeDefault();

    // Restores whatever captureCustomPointBeforeDefault() saved. Called
    // when "presetMode" turns back off (see ParamListener below), BEFORE
    // requestBoundaryRedesign() - so the very next redesign already targets
    // the user's real last Custom-mode spec (which, if it was searched
    // before turning Default on, is now instantly recalled from
    // LiveSearchCache.h rather than re-searched). A no-op if Default was
    // never actually engaged this session (nothing was ever overwritten),
    // or if this is a resent "off" event while already off.
    void restoreCustomPointBeforeDefault();

    juce::AudioProcessorValueTreeState parameters;

    struct ParamListener final : juce::AudioProcessorValueTreeState::Listener
    {
        BBKDetachedPoleAudioProcessor& owner;
        explicit ParamListener (BBKDetachedPoleAudioProcessor& o) : owner (o) {}
        void parameterChanged (const juce::String& parameterID, float newValue) override
        {
            // Order matters: force cutoff/stopband/decay to the preset
            // operating point (or restore the pre-Default Custom point,
            // going the other way) BEFORE requesting a redesign below, so
            // that redesign already sees the corrected spec instead of one
            // that's about to be superseded a moment later by the
            // parameter changes forcePresetOperatingPoint() itself
            // triggers (each of which re-enters this same listener and
            // requests its own redesign in turn - harmless, see that
            // method's own comment, just a couple of extra superseded
            // requests exactly like a fast slider drag already causes).
            if (parameterID == "presetMode")
            {
                if (newValue > 0.5f)
                    owner.forcePresetOperatingPoint(); // captures the pre-Default point itself first - see its own comment
                else
                    owner.restoreCustomPointBeforeDefault();
            }
            owner.requestBoundaryRedesign();
        }
    } paramListener { *this };

    struct ChannelState
    {
        std::array<double, bbk::detachedpole::historyLength> history {};
        int writeIndex = 0;
        void clear() noexcept { history.fill (0.0); writeIndex = 0; }
    };
    std::vector<ChannelState> channels;

    // Audio-thread-owned "currently playing" tap set and the crossfade
    // target - both fixed-length (maxTapCount), zero-padded, centre
    // aligned (see DetachedPoleFilter.h::padTapsToFixedLength), so every
    // design has exactly the same reported latency and can be convolved
    // with one fixed-size loop regardless of how many taps it actually
    // used.
    std::array<double, bbk::detachedpole::maxTapCount> activeTaps {};
    std::array<double, bbk::detachedpole::maxTapCount> incomingTaps {};
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Linear> designCrossfade;
    bool crossfading = false;

    // Bypass: a second, independent crossfade layered on top of the
    // design crossfade above - 0 means fully processed, 1 means fully dry
    // (delayed by exactly latencySamples, the same fixed delay the
    // filtered path already has by construction, so toggling bypass lines
    // up sample-for-sample with whatever is currently playing and can
    // never click). Deliberately kept independent of the design crossfade
    // rather than folded into a single state machine: if a redesign
    // happens to complete while bypassed, it quietly finishes in the
    // background and the user hears it only once they un-bypass.
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Linear> bypassCrossfade;
    bool lastBypassParam = false;

    // Background design thread hand-off. The audio thread never runs
    // designParametricFIR() itself - a demanding spec can legitimately take
    // minutes now that the search is deliberately thorough (see
    // ParametricFIR.h), fine off the audio thread, fatal on it. Every lock
    // below is only ever held for a very short copy (a FilterSpec/
    // DesignResult/DesignTask, at most maxTapCount doubles), so a
    // best-effort tryEnter() from the audio thread is safe in practice.
    juce::SpinLock specLock;

    // One background-design task: the boundary spec it applies to, and the
    // boundaryEpoch it was queued for (checked again before, and after,
    // the actual design runs - see run() - so a task superseded by a newer
    // boundary change mid-flight is discarded rather than published).
    //
    // manualTapCount/requestedTapCount: a snapshot of the "tapCountAuto"/
    // "manualTapCount" parameters at the moment this task was queued (see
    // requestBoundaryRedesign()) - run() uses these to decide whether to
    // call designParametricFIR()'s own auto M-search or the fixed-M
    // designParametricFIRFixedM() instead (see ParametricFIR.h). Snapshotting
    // here, rather than re-reading the live parameter inside run(), keeps
    // this task self-consistent even if the user changes Auto/Manual again
    // while an older task is still queued or mid-flight - exactly the same
    // "the task carries what it needs, not a live reference" principle
    // already used for spec above.
    struct DesignTask
    {
        bbk::parametric::FilterSpec spec;
        int epoch = 0;
        bool manualTapCount = false;
        int requestedTapCount = 0; // only meaningful when manualTapCount is true
    };
    std::deque<DesignTask> taskQueue;          // guarded by specLock
    int boundaryEpoch = 0;                     // guarded by specLock
    bbk::parametric::FilterSpec currentBoundarySpec; // guarded by specLock

    // Also guarded by specLock, alongside currentBoundarySpec: the
    // specsEqual() dedup below only compares FilterSpec fields, which say
    // nothing about presetMode - toggling Default on/off changes where the
    // result comes from (instant bank lookup vs. live search) even when
    // every FilterSpec field is unchanged (e.g. the user never touched
    // cutoff/attenuation/stopband/decay), so the dedup must also notice
    // that transition or it silently keeps showing whichever result was
    // already published.
    bool currentBoundaryPresetMode = false;

    // Also guarded by specLock, same reasoning and same purpose as
    // currentBoundaryPresetMode immediately above: the specsEqual() dedup
    // says nothing about Manual/Auto tap-count mode either, so a user who
    // only flips "tapCountAuto" or drags "manualTapCount" - touching no
    // other parameter - must still force a fresh redesign, not be silently
    // absorbed by the "nothing really changed" early return. requestedTapCount
    // only matters while currentBoundaryManualTapCountOn is true (comparing
    // it while Auto is on would force a spurious redesign every time the
    // manual slider is nudged with the mouse even though Auto mode ignores
    // it entirely - see requestBoundaryRedesign()).
    bool currentBoundaryManualTapCountOn = false;
    int currentBoundaryRequestedTapCount = 0;

    // Guarded by specLock. See captureCustomPointBeforeDefault()/
    // restoreCustomPointBeforeDefault() above for the full story: this is
    // the user's own cutoff/attenuation/stopband/decay from the instant
    // before Default was last turned on, put back the instant it's turned
    // back off. havePreDefaultSnapshot distinguishes "never captured yet"
    // (Default has never been engaged this session) from a genuinely
    // all-zero snapshot.
    struct PreDefaultSnapshot
    {
        double cutoffHz = 0.0;
        double attenuationAtCutoffDb = 0.0;
        double stopbandRejectionDb = 0.0;
        double sidelobeDecayRatio = 0.0;
    };
    PreDefaultSnapshot preDefaultSnapshot;
    bool havePreDefaultSnapshot = false;

    // Guarded by specLock (not message-thread-only: requestBoundaryRedesign()
    // - which searches this - can run on the audio thread too, same as
    // currentBoundarySpec above). Loaded once in the constructor and
    // updated by saveCurrentAsOverride(); kept in memory so a lookup
    // (potentially once per parameter tick, e.g. mid slider-drag) never
    // has to hit disk - see UserPresetOverrides.h's own comment on why
    // the file itself isn't cached at that layer.
    std::vector<bbk::detachedpole::useroverrides::OverrideEntry> userOverrides;

    // Also guarded by specLock, same reasoning as userOverrides above.
    // Oldest-first: run() appends the newest completed live search to the
    // back and evicts from the front once bbk::detachedpole::searchcache::
    // maxEntries is exceeded - see LiveSearchCache.h and run()'s own
    // comment. Loaded once in the constructor.
    std::vector<bbk::detachedpole::searchcache::CacheEntry> searchCache;

    juce::SpinLock resultLock;
    bbk::parametric::DesignResult latestResult;
    bbk::parametric::FilterSpec latestSpec;
    int latestVersion = 0;
    int consumedVersion = 0; // audio-thread-only: last version picked up

    int versionCounter = 0; // guarded by specLock; shared source of ever-increasing version numbers

    std::atomic<double> currentSampleRate { 0.0 };
    double lastPreparedSampleRate = 0.0;
    std::atomic<bool> hasPrepared { false };

    // Auto-headroom calibration state - audio-thread only. Same design as
    // BBK Phase Corrector's and BBK Temporal FIR's Auto Headroom: a leaky
    // peak-hold of how far the WET (filtered) signal alone has recently
    // pushed past the soft-clip knee; when it stays above the trigger for
    // a sustained period AND the cooldown has elapsed, "headroom" is
    // ratcheted one step more negative via setValueNotifyingHost().
    // One-way only - see PluginProcessor.cpp.
    float clipEnvelope = 0.0f;
    int samplesUntilNextAutoAdjust = 0;
    int autoAdjustCooldownSamples = 0;

    // Written from the audio thread, read from the message thread for the
    // UI's clip indicator - see getLastClipTimeMsForUI() above. Driven by
    // the exact same per-block "did the wet signal exceed the knee"
    // detection that feeds the Auto Headroom ratchet, so the light and
    // Auto react to the same events, not two independent measurements.
    std::atomic<juce::uint32> lastClipTimeMs { 0 };

    // Backs isSearchInProgressForUI() above. Set true exactly when a
    // DesignTask is actually queued (requestBoundaryRedesign()'s final
    // else-branch), set false exactly when a result is published - either
    // an instant one (bank/override/cache hit, in requestBoundaryRedesign()
    // itself) or a completed background search (in run(), only on the
    // branch that actually calls publishResult() - a task discarded as
    // stale, either before or after running the design, never touches this
    // flag, since by construction its being stale means a newer boundary
    // change already updated the flag itself, either back to false via its
    // own instant result or left it true for its own newly-queued task -
    // see both call sites' own comments).
    std::atomic<bool> searchInProgress { false };

    // A cheap, decaying measure of how much of the audio callback's own
    // available time budget (numSamples / sampleRate) each block's actual
    // wall-clock processing took - see process()'s own comment for where
    // this is updated and currentAllowedSearchConcurrency() above for how
    // it throttles the parallel search. Written from the audio thread on
    // every block (a couple of chrono calls plus one atomic store -
    // negligible overhead), read only from the background design thread.
    std::atomic<float> audioCallbackLoadFraction { 0.0f };

    // System-wide CPU utilization sample state for
    // pollSystemCpuBusyFraction() above - only ever touched from the
    // background design thread (never the audio thread), polled at most a
    // few times per second (once per search "round", not continuously),
    // so no locking is needed. Plain integer tick counts rather than a
    // Windows FILETIME/ULARGE_INTEGER type so this header stays free of
    // <windows.h>; the .cpp does the actual interpretation.
    std::uint64_t lastSystemIdleTicks = 0;
    std::uint64_t lastSystemKernelTicks = 0;
    std::uint64_t lastSystemUserTicks = 0;
    bool haveSystemCpuSample = false;

    // Message-thread-only snapshot of the latest completed design, kept
    // separately from the audio-thread hand-off above so the UI never has
    // to contend with the audio thread for it.
    mutable juce::SpinLock uiSnapshotLock;
    DesignSnapshot uiSnapshot;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessor)
};
