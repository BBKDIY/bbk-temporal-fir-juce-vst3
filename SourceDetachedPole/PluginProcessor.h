#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <atomic>
#include <vector>
#include "DetachedPoleFilter.h"
#include "ParametricFIR.h"

// BBK Detached Pole: a single parametric constrained-least-squares FIR
// lowpass (see ParametricFIR.h for the design method). Three user-facing
// controls - cutoff, attenuation at cutoff, and minimum stopband
// rejection - are handed straight to bbk::parametric::designParametricFIR().
// The plugin auto-detects the host sample rate (44.1/48/96/192 kHz and
// anything else the host reports) rather than hard-locking to 192 kHz.
//
// A redesign never runs on the audio thread: it happens either
// synchronously in prepareToPlay() (cold start / sample-rate change - the
// host already expects a pause there) or on a dedicated background
// juce::Thread when a slider changes during playback, with the audio
// thread crossfading smoothly from the previous design into the new one
// (same 15 ms linear crossfade mechanism this plugin has always used for
// its mode switches, now generalised to two arbitrary tap sets instead of
// four fixed ones) so a live redesign never clicks.
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

    const juce::String getName() const override { return "BBK Detached Pole"; }
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

    // A snapshot of the most recently completed design, safe to read from
    // the message thread at any time (used by the editor's metrics
    // readout and the "show coefficients" popup).
    struct DesignSnapshot
    {
        double sampleRateHz = 0.0;
        double cutoffHz = 0.0;
        double attenuationAtCutoffDb = 0.0;
        double stopbandRejectionDb = 0.0;
        int tapCount = 0;
        double achievedStopbandDb = 0.0;
        bool constraintsMet = false;
        int designAttempts = 0;
        std::vector<double> taps; // the actual (unpadded) symmetric taps
    };
    DesignSnapshot getDesignSnapshotForUI() const;

private:
    void run() override; // juce::Thread - background redesign worker

    template <typename SampleType>
    void process (juce::AudioBuffer<SampleType>& buffer);

    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    bbk::parametric::FilterSpec specFromParameters() const;
    void redesignSynchronously (const bbk::parametric::FilterSpec& spec);
    void requestBackgroundRedesign();

    juce::AudioProcessorValueTreeState parameters;

    struct ParamListener final : juce::AudioProcessorValueTreeState::Listener
    {
        BBKDetachedPoleAudioProcessor& owner;
        explicit ParamListener (BBKDetachedPoleAudioProcessor& o) : owner (o) {}
        void parameterChanged (const juce::String&, float) override { owner.requestBackgroundRedesign(); }
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

    // Background design thread hand-off. The audio thread never runs
    // designParametricFIR() itself - it can take tens to low hundreds of
    // milliseconds for demanding specs, fine off the audio thread, fatal
    // on it. Both locks are only ever held for a very short copy (a
    // FilterSpec/DesignResult, at most maxTapCount doubles), so a
    // best-effort tryEnter() from the audio thread is safe in practice.
    juce::SpinLock specLock;
    bbk::parametric::FilterSpec requestedSpec;
    int requestedVersion = 0;

    juce::SpinLock resultLock;
    bbk::parametric::DesignResult latestResult;
    bbk::parametric::FilterSpec latestSpec;
    int latestVersion = 0;
    int consumedVersion = 0; // audio-thread-only: last version picked up

    int versionCounter = 0; // guarded by specLock; shared source of ever-increasing version numbers

    std::atomic<double> currentSampleRate { 0.0 };
    double lastPreparedSampleRate = 0.0;
    std::atomic<bool> hasPrepared { false };

    // Message-thread-only snapshot of the latest completed design, kept
    // separately from the audio-thread hand-off above so the UI never has
    // to contend with the audio thread for it.
    mutable juce::SpinLock uiSnapshotLock;
    DesignSnapshot uiSnapshot;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessor)
};
