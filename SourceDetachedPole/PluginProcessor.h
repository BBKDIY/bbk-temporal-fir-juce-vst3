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

    // True whenever a live background design - a fresh search, or a
    // fallback to one when a spec doesn't land an instant hit in the
    // compiled-in bank/a saved override/preset/the search cache - is
    // currently queued or running for the boundary spec presently in
    // effect, i.e. whatever the UI is showing right now (see DesignSnapshot)
    // is not yet the freshest result being computed. Deliberately distinct
    // from DesignSnapshot::tapCount == 0, which means "no completed design
    // AT ALL yet" (the cold-start/sample-rate-change case, already shown via
    // its own "Designing filter for..." message in the editor) - this
    // instead covers the steady-state case where a PREVIOUS result is
    // already playing but a newer one is being computed because a boundary
    // parameter (cutoff, attenuation, stopband, sidelobe decay, or the
    // Manual/Auto tap-count selector) just changed. See
    // requestBoundaryRedesign() and run() for where this is set/cleared.
    bool isSearchInProgressForUI() const noexcept { return searchInProgress.load(); }

    // A snapshot of the in-progress background search's own live progress -
    // how many candidate tap counts it has tried so far, and the best
    // (lowest-ringing feasible, or least-far-from-compliant if none is
    // feasible yet) result found among them - written from
    // ParametricFIR.h's SearchConcurrencyHooks::onProgress callback (see
    // run()) roughly once per candidate M, so it updates continuously while
    // a search runs. Deliberately separate from DesignSnapshot/
    // getDesignSnapshotForUI() above, which only ever reflects the last
    // PUBLISHED (actually playing) result: per this plugin's design, the
    // audio and the "official" metrics readout stay on the previous result
    // for the whole duration of a search and only switch once, either when
    // the search finishes on its own, the user clicks Stop
    // (requestStopSearch() below), or the safety-net deadline is hit - this
    // snapshot is what lets the editor show live trial-count/best-so-far
    // progress in the meantime without disturbing what's actually playing.
    struct SearchProgressSnapshot
    {
        int attemptsSoFar = 0;
        bool haveBest = false;
        int bestTapCount = 0;
        double bestRPeakPercent = 0.0;
        double bestAchievedStopbandDb = 0.0;

        // True only once "best" above is a fully spec-compliant design, not
        // merely the closest non-degenerate attempt seen so far - see
        // ParametricFIR.h's SearchConcurrencyHooks::onProgress comment. Lets
        // the editor tell "best so far" (compliant) apart from "closest so
        // far, still searching for a compliant tap count" (not compliant
        // yet) instead of showing nothing at all until compliance is found.
        bool bestIsFeasible = false;
    };
    SearchProgressSnapshot getSearchProgressForUI() const;

    // Requests that whichever background search is currently in progress
    // (if any) stop as soon as practical and publish the best result found
    // so far, exactly as if it had hit maxTapCount or its own deadline -
    // see ParametricFIR.h's SearchConcurrencyHooks::shouldStopEarly, which
    // run() wires directly to this flag, and that header's own comment on
    // how a requested stop collapses the current search round's deadline to
    // "now" for a fast (sub-second) response rather than waiting out the
    // rest of a slow candidate's LP solve. A no-op if no search is running.
    // The flag is reset at the start of every newly-queued task in run(),
    // so a stop request can never "leak" forward and silently cut short a
    // later, unrelated search the user didn't ask to stop.
    void requestStopSearch() noexcept { stopSearchRequested.store (true); }

    // Where a published design actually came from - shown in the editor's
    // metrics readout.
    enum class ResultSource
    {
        LiveSearch,   // a fresh designParametricFIR() search
        PresetBank,   // instant lookup in the compiled-in factory bank (see requestBoundaryRedesign()) - also
                      // what an unassigned Preset 1 falls back to, see loadPresetSlot()
        UserOverride, // instant lookup in a filter the user saved themselves (see saveTopCandidateAsOverride/savePresetSlot)
        SearchCache   // instant lookup in a result already searched earlier (see LiveSearchCache.h)
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

        // Up to bbk::parametric::topCandidateCount ranked alternatives to
        // the design above (see ParametricFIR.h::RankedCandidate/
        // DesignResult::topCandidates), best R_peak first - what the
        // editor's own top-N table shows. selectedIndex says which one of
        // these is the design actually described by every field above
        // (taps/tapCount/achievedStopbandDb/temporal): topCandidates[
        // selectedIndex] always matches them exactly when topCandidates is
        // non-empty. Both are left at their defaults (empty/0) for a Manual
        // tap-count result or a factory-bank entry, neither of which
        // has a ranked list to offer - see selectTopCandidate() and
        // saveTopCandidateAsOverride() below for how a non-default index
        // gets here.
        std::vector<bbk::parametric::RankedCandidate> topCandidates;
        int selectedIndex = 0;
    };
    DesignSnapshot getDesignSnapshotForUI() const;

    // Switches which of the currently-published design's own topCandidates
    // is the active (playing) one, without re-running a search - all
    // topCandidates entries are already fully-designed, spec-compliant
    // filters from the same completed search (see ParametricFIR.h), so
    // picking a different one is just a re-publish of already-known taps,
    // same cost as an instant cache/override/bank hit. A no-op if index is
    // out of range or the current design has no ranked list at all (Manual
    // mode, or a factory-bank entry). Does not touch the search cache
    // or any saved override - see saveTopCandidateAsOverride() for making a
    // choice persistent.
    void selectTopCandidate (int index);

    // Persists one specific candidate from the currently-published design's
    // own topCandidates (see DesignSnapshot above) - or, if it has no ranked
    // list at all (Manual mode, or a Preset-slot/bank entry), the single
    // design currently playing - as a user override for its own exact spec
    // (sample rate, cutoff, attenuation, stopband, decay - see
    // UserPresetOverrides.h). The WHOLE ranked list is saved, not just the
    // chosen candidate, tagged with which index was picked (see
    // OverrideEntry::activeIndex) - so revisiting this exact spec later
    // instantly recalls the user's chosen filter AND still offers every
    // other ranked alternative in the editor's own table, exactly as if the
    // search had just finished again. From then on, ANY time that exact
    // spec recurs, this session or a future one, requestBoundaryRedesign()
    // finds and uses it instantly, ahead of both the compiled-in factory
    // bank and a fresh live search. A no-op if nothing has been designed
    // yet (tapCount == 0) or index is out of range for whatever list is
    // being saved. Called from a top-N table row's own per-candidate "Save"
    // button (see PluginEditor.cpp) - if this exact spec already occupies a
    // preset slot (see savePresetSlot() below), that slot assignment is
    // preserved rather than cleared, so re-saving the same find through
    // this button can never silently un-assign a preset.
    void saveTopCandidateAsOverride (int index);

    // Numbered preset slots (0..bbk::detachedpole::useroverrides::
    // numPresetSlots-1) - what replaced the plugin's old single "Default"
    // toggle. Each slot independently remembers one whole operating point
    // (cutoff, attenuation, stopband, decay, and its own ranked candidate
    // list - stored as an OverrideEntry tagged with this slot number, see
    // UserPresetOverrides.h), addressed directly by slot rather than by
    // matching the spec currently dialed in. This is the direct answer to
    // "I don't remember which of the near-infinite parameter combinations
    // gave me a result I liked" - bookmark a good find into a slot the
    // moment you hear it, then come back to that slot later without having
    // to reconstruct or even remember the search that produced it.
    struct PresetSlotInfo
    {
        bool occupied = false;
        bbk::parametric::FilterSpec spec; // meaningful only when occupied is true

        // The slot's own active candidate's actual filter, not just the spec
        // it was searched for - meaningful only when occupied is true. Mirrors
        // RankedCandidate's own "don't persist derived metrics, recompute them
        // from taps" convention (see its comment in ParametricFIR.h): the
        // editor calls bbk::parametric::computeTemporalMetrics (taps,
        // spec.sampleRateHz) to get R_peak/T_0.1%/etc, exactly like the top-N
        // table already does per row, so this struct only needs to carry
        // what that function itself needs plus achievedStopbandDb (which
        // isn't derivable from taps alone).
        int tapCount = 0;
        double achievedStopbandDb = 0.0;
        std::vector<double> taps;
    };
    // For the editor's own slot labels (see PluginEditor.cpp::timerCallback()),
    // which show this alongside spec so a preset reads the same "N taps |
    // R_peak X% | stopband Y dB" summary the top-N table already shows for
    // live search candidates - not just the input spec that was searched for.
    // slot must be in range [0, numPresetSlots); out of range returns an
    // unoccupied result rather than asserting, same defensive posture as
    // selectTopCandidate()'s own range check.
    PresetSlotInfo getPresetSlotInfoForUI (int slot) const;

    // Recalls slot's own exact operating point: forces cutoff/attenuation/
    // stopband/decay (and Tap Count Auto, back on) to match it via
    // setValueNotifyingHost, the same "processor drives another parameter's
    // value directly" pattern forcePresetOperatingPoint used to use for the
    // old Default toggle - each of those parameter changes re-enters
    // requestBoundaryRedesign() via the ParamListener, which (since this
    // exact spec is already sitting in userOverrides, tagged with this slot
    // - see savePresetSlot()) finds and republishes it instantly once every
    // parameter has caught up, no background search needed. If slot is
    // unoccupied: a no-op for slots 1-4 (nothing has ever been saved there
    // yet); slot 0 instead falls back to the compiled-in factory bank's
    // fixed operating point (cutoff/stopband/decay only - Attenuation is
    // left exactly where it already is, so it still picks among the bank's
    // 10 precomputed steps) - the same starting point the old Default
    // toggle always forced, so there's always something useful in Preset 1
    // even before you've saved anything of your own.
    void loadPresetSlot (int slot);

    // Saves whatever is CURRENTLY PLAYING (same source as
    // saveTopCandidateAsOverride(), re-read fresh at call time via
    // getDesignSnapshotForUI() so it always reflects the very latest Use/
    // Load) directly into the given slot, unconditionally overwriting
    // whatever was there before. A no-op if nothing has been designed yet.
    // If some other override already exists for this exact spec (e.g. it
    // was already saved via a top-N row's plain "Save" button), that entry
    // is replaced rather than duplicated, same dedup rule
    // saveTopCandidateAsOverride() already uses.
    void savePresetSlot (int slot);

    // Writes every currently-occupied preset slot (and nothing else - a
    // plain, non-preset override saved via a top-N row's "Save" button is
    // NOT included) to destFile, in the same XML shape UserPresetOverrides.h
    // already uses for its own persistence - see UserPresetOverrides.h::
    // saveAll()'s own comment on why loadAll()/saveAll() both take a file
    // argument. This is the whole "share with a friend" feature: destFile
    // is just an ordinary file the user can email, message, or drop
    // anywhere, and importPresets() below reads it back on the other end.
    void exportPresets (const juce::File& destFile);

    // Reads srcFile (expected to be a file exportPresets() produced, though
    // any valid UserPresetOverrides-shaped XML works) and merges every
    // preset it contains into this install's own bank, slot by slot: for
    // each occupied slot the file defines, whatever currently occupies that
    // same slot locally is displaced (demoted to a plain, non-preset
    // override, same as savePresetSlot()'s own overwrite rule) and replaced
    // with the imported one. Slots the file doesn't define are left
    // completely untouched - importing a friend's 2-preset export never
    // disturbs the other 3 slots you've already built up yourself. Returns
    // false if srcFile couldn't be read or contained no presets at all (a
    // plain override-only export, or an unrelated/corrupt file); true
    // otherwise, including if fewer presets were merged than the file
    // nominally listed (only entries actually tagged with a valid slot are
    // merged - see UserPresetOverrides.h::loadAll()'s own dedup guard).
    bool importPresets (const juce::File& srcFile);

    // Forces a genuinely fresh live search for the CURRENT spec, bypassing
    // every instant source (compiled-in bank, a saved override, and - the
    // whole point of this button - an existing search-cache entry for this
    // exact spec) that would otherwise short-circuit requestBoundaryRedesign()
    // and just replay an old result. Answers the direct question of whether
    // a spec that already has saved/cached results can be re-searched: it
    // can, on demand, without needing to nudge a parameter and back just to
    // force a cache miss. The fresh result still gets written back into the
    // search cache on completion (see run()), so it replaces whatever was
    // cached before - a deliberate re-search is exactly as much "the new
    // answer for this spec" as the original one was. A no-op before the
    // very first prepareToPlay().
    void requestFreshSearch();

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
    // selectedIndex says which entry of result.topCandidates (if any) this
    // publish represents - see DesignSnapshot::selectedIndex's own comment -
    // and is copied straight into uiSnapshot alongside topCandidates itself;
    // it means nothing when topCandidates is empty and defaults to 0 (the
    // ordinary "just-published the best/only result" case).
    void publishResult (const bbk::parametric::FilterSpec& spec,
                         const bbk::parametric::DesignResult& result,
                         ResultSource source = ResultSource::LiveSearch,
                         int selectedIndex = 0);

    juce::AudioProcessorValueTreeState parameters;

    struct ParamListener final : juce::AudioProcessorValueTreeState::Listener
    {
        BBKDetachedPoleAudioProcessor& owner;
        explicit ParamListener (BBKDetachedPoleAudioProcessor& o) : owner (o) {}
        void parameterChanged (const juce::String&, float) override
        {
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
    // mutable: getPresetSlotInfoForUI() needs to lock this from a const
    // method, same reasoning as uiSnapshotLock/searchProgressLock below.
    mutable juce::SpinLock specLock;

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

    // Also guarded by specLock, same reasoning as currentBoundarySpec: the
    // specsEqual() dedup below says nothing about Manual/Auto tap-count
    // mode, so a user who only flips "tapCountAuto" or drags
    // "manualTapCount" - touching no other parameter - must still force a
    // fresh redesign, not be silently absorbed by the "nothing really
    // changed" early return. requestedTapCount only matters while
    // currentBoundaryManualTapCountOn is true (comparing it while Auto is
    // on would force a spurious redesign every time the manual slider is
    // nudged with the mouse even though Auto mode ignores it entirely -
    // see requestBoundaryRedesign()).
    bool currentBoundaryManualTapCountOn = false;
    int currentBoundaryRequestedTapCount = 0;

    // Guarded by specLock (not message-thread-only: requestBoundaryRedesign()
    // - which searches this - can run on the audio thread too, same as
    // currentBoundarySpec above). Loaded once in the constructor and
    // updated by saveTopCandidateAsOverride(); kept in memory so a lookup
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

    // Backs requestStopSearch()/getSearchProgressForUI() above. Reset to
    // false at the start of every task run() picks up off taskQueue, so a
    // Stop click can never affect a later, different search. Checked from
    // the background design thread only (via SearchConcurrencyHooks::
    // shouldStopEarly - see run()); written from the message thread only
    // (requestStopSearch()) - a plain atomic bool needs nothing more.
    std::atomic<bool> stopSearchRequested { false };

    // Guarded by its own lock rather than reusing resultLock/specLock:
    // ParametricFIR.h's SearchConcurrencyHooks::onProgress (see run()) can
    // fire once per candidate M, many times a second during a fast search,
    // and giving it a dedicated lock keeps that frequent write from
    // contending with the audio thread's own tryEnter() on resultLock or
    // requestBoundaryRedesign()'s use of specLock.
    mutable juce::SpinLock searchProgressLock;
    SearchProgressSnapshot searchProgress;

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
