#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <atomic>
#include <vector>
#include "DetachedPoleFilter.h"

// BBK Detached Pole: an A/B/C/F loopback-comparison tool built around the
// accompanying article's Case B and Case C designs (Part Two, "A
// Practical Challenge to the FIR Frontier: The Detached Real Pole"),
// plus an externally-supplied Case F candidate FIR used here to validate
// that candidate against Case B and Case C through the same physical
// signal chain. Case B and Case C share the exact same 19-tap FIR stage
// (identical scipy.signal.remez call, same 20/76 kHz edges); Case C
// additionally cascades that FIR through a single real IIR pole whose
// cut-off (47 kHz) is decoupled from the FIR's own passband edge. Case F
// is a separate, independently-optimized 19-tap FIR supplied as a fixed
// coefficient set (no pole, no redesign inside this plugin). See
// DetachedPoleFilter.h for all coefficients and their provenance.
//
// The plugin exposes one 4-way MODE selector rather than a bypass toggle:
//   0 = Bypass   (dry, latency-matched)
//   1 = Case B   (19-tap FIR only)
//   2 = Case C   (19-tap FIR + decoupled 47 kHz real pole)
//   3 = Case F   (19-tap joint passband/transition-optimized FIR, no pole)
// so all four can be measured through the same physical signal chain
// (e.g. a loopback interface) and compared at matched level. Each mode's
// own signal is computed independently every sample from its own
// coefficients (and, for Case C only, its own separate pole state) - no
// mode ever reads another mode's coefficients, pole state, or delay
// buffer contents, so switching modes can never leak state between
// cases (in particular, Case F never touches the pole state used by
// Case C).
class BBKDetachedPoleAudioProcessor final : public juce::AudioProcessor
{
public:
    enum class Mode { bypass = 0, caseB = 1, caseC = 2, caseF = 3 };

    BBKDetachedPoleAudioProcessor();
    ~BBKDetachedPoleAudioProcessor() override = default;

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
    double getCurrentSampleRateForUI() const noexcept { return currentSampleRate.load(); }
    bool isRateValidForUI() const noexcept { return valid192k.load(); }
    Mode getModeForUI() const noexcept;

private:
    struct ChannelState
    {
        std::array<double, bbk::detachedpole::historyLength> history {};
        int writeIndex = 0;
        double poleW1 = 0.0; // FIR stage's own output one sample ago, w[n-1] (Case C only)
        double poleY1 = 0.0; // pole stage's own output one sample ago, y[n-1] (Case C only)

        void clear() noexcept
        {
            history.fill (0.0);
            writeIndex = 0;
            poleW1 = 0.0;
            poleY1 = 0.0;
        }
    };

    template <typename SampleType>
    void process (juce::AudioBuffer<SampleType>& buffer);

    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

    juce::AudioProcessorValueTreeState parameters;
    std::vector<ChannelState> channels;

    // Mode-select crossfade: the currently-audible mode's own signal
    // ("from") keeps playing uninterrupted while a newly-selected mode's
    // signal ("to") fades in - every mode's own signal (dry/wetB/wetC/wetF)
    // is computed every sample regardless of which is selected, so the
    // pole stage's own state (used only by Case C) is always warm and a
    // switch into Case C is never a cold-start transient. Case F carries
    // no state at all beyond the shared raw-input history, so it is
    // always bit-identical to a fresh convolution regardless of which
    // other modes have been active.
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Linear> modeCrossfade;
    Mode fromMode = Mode::bypass;
    Mode toMode = Mode::bypass;
    Mode lastSeenMode = Mode::bypass;

    std::atomic<double> currentSampleRate { 0.0 };
    std::atomic<bool> valid192k { false };

    double lastPreparedSampleRate = 0.0;
    bool hasPrepared = false;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessor)
};
