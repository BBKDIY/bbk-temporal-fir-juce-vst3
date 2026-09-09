#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <atomic>
#include <deque>
#include <vector>
#include "DetachedPoleFilter.h"
#include "ParametricFIR.h"

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
        bool fromPresetBank = false; // instant lookup (see requestBoundaryRedesign), not a live search
        std::vector<double> taps; // the actual (unpadded) symmetric taps

        // Temporal-concentration metrics from the article ("Impulse-
        // Response Ringing in Digital Reconstruction Filtering"), computed
        // directly from the taps above by
        // bbk::parametric::computeTemporalMetrics() - see ParametricFIR.h
        // for exact definitions and the Case C reference validation.
        bbk::parametric::TemporalMetrics temporal;
    };
    DesignSnapshot getDesignSnapshotForUI() const;

private:
    void run() override; // juce::Thread - background redesign worker

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
    // published design. fromPresetBank just tags the UI snapshot (see
    // DesignSnapshot) - it doesn't change how the result is applied.
    void publishResult (const bbk::parametric::FilterSpec& spec,
                         const bbk::parametric::DesignResult& result,
                         bool fromPresetBank = false);

    // Called when the "presetMode" parameter turns on (see ParamListener
    // below): forces cutoff/stopband/sidelobeDecay to the exact operating
    // point the preset bank was swept at, via setValueNotifyingHost - the
    // same "processor drives another parameter's value directly" pattern
    // already used for the Auto Headroom ratchet (see process()). This
    // makes the greyed-out sliders in Default mode show the values that
    // are actually in effect, rather than whatever Custom-mode position
    // they were last left at, and means specFromParameters() never needs
    // its own separate preset/custom branch - it just always reads
    // whatever the parameters currently hold.
    void forcePresetOperatingPoint();

    juce::AudioProcessorValueTreeState parameters;

    struct ParamListener final : juce::AudioProcessorValueTreeState::Listener
    {
        BBKDetachedPoleAudioProcessor& owner;
        explicit ParamListener (BBKDetachedPoleAudioProcessor& o) : owner (o) {}
        void parameterChanged (const juce::String& parameterID, float newValue) override
        {
            // Order matters: force cutoff/stopband/decay to the preset
            // operating point BEFORE requesting a redesign below, so that
            // redesign already sees the corrected spec instead of one
            // that's about to be superseded a moment later by the
            // parameter changes forcePresetOperatingPoint() itself
            // triggers (each of which re-enters this same listener and
            // requests its own redesign in turn - harmless, see that
            // method's own comment, just a couple of extra superseded
            // requests exactly like a fast slider drag already causes).
            if (parameterID == "presetMode" && newValue > 0.5f)
                owner.forcePresetOperatingPoint();
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
    struct DesignTask
    {
        bbk::parametric::FilterSpec spec;
        int epoch = 0;
    };
    std::deque<DesignTask> taskQueue;          // guarded by specLock
    int boundaryEpoch = 0;                     // guarded by specLock
    bbk::parametric::FilterSpec currentBoundarySpec; // guarded by specLock

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

    // Message-thread-only snapshot of the latest completed design, kept
    // separately from the audio-thread hand-off above so the UI never has
    // to contend with the audio thread for it.
    mutable juce::SpinLock uiSnapshotLock;
    DesignSnapshot uiSnapshot;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessor)
};
