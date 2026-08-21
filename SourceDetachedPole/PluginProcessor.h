#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <atomic>
#include <vector>
#include "DetachedPoleFilter.h"

// BBK Detached Pole: a real-time reproduction of the accompanying
// article's Case C (Part Two, "A Practical Challenge to the FIR
// Frontier: The Detached Real Pole") - a 19-tap linear-phase FIR cascaded
// with a single real IIR pole whose cut-off is deliberately decoupled
// from the FIR's own passband edge. See DetachedPoleFilter.h for the
// exact design, its coefficients, and its provenance.
//
// Unlike BBK Temporal FIR, this is a single fixed design point (Case C
// is not swept), so the plugin exposes only a BYPASS/ACTIVE toggle - no
// trade-off slider.
class BBKDetachedPoleAudioProcessor final : public juce::AudioProcessor
{
public:
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
    bool isEnabledForUI() const noexcept;

private:
    struct ChannelState
    {
        std::array<double, bbk::detachedpole::historyLength> history {};
        int writeIndex = 0;
        double poleW1 = 0.0; // FIR stage's own output one sample ago, w[n-1]
        double poleY1 = 0.0; // pole stage's own output one sample ago, y[n-1]

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

    // enabled/bypass crossfade (dry <-> filtered), same pattern as BBK Temporal FIR.
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Linear> wetMix;

    std::atomic<double> currentSampleRate { 0.0 };
    std::atomic<bool> valid192k { false };

    double lastPreparedSampleRate = 0.0;
    bool hasPrepared = false;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessor)
};
