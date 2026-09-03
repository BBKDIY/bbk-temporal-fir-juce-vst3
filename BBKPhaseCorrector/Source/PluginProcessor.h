#pragma once

#include <JuceHeader.h>
#include "PhaseFilterDesigner.h"

class BBKPhaseCorrectorAudioProcessor final : public juce::AudioProcessor
{
public:
    BBKPhaseCorrectorAudioProcessor();
    ~BBKPhaseCorrectorAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return JucePlugin_Name; }
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

    juce::AudioProcessorValueTreeState parameters;

    double getCurrentSampleRateForUI() const noexcept { return currentSampleRate.load(); }
    int getPhaseLatencySamplesForUI() const noexcept { return phaseLatencySamples.load(); }

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    static juce::AudioBuffer<float> makeImpulseBuffer (const std::vector<float>& impulse);

    void initialiseDryDelay (int channels, int latencySamples);
    void processDryDelay (const juce::dsp::AudioBlock<const float>& input,
                          juce::dsp::AudioBlock<float>& output) noexcept;
    void updateModeTargets (int mode);

    juce::dsp::Convolution minimumConvolution;
    juce::dsp::Convolution linearConvolution;

    juce::AudioBuffer<float> minimumBuffer;
    juce::AudioBuffer<float> linearBuffer;
    juce::AudioBuffer<float> delayedDryBuffer;

    std::vector<std::vector<float>> dryDelayRing;
    int dryDelayWritePosition = 0;
    int dryDelayLength = 0;

    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> bypassMix { 1.0f };
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> minimumMix { 0.0f };
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> linearMix { 0.0f };
    int lastMode = -1;

    std::atomic<double> currentSampleRate { 0.0 };
    std::atomic<int> phaseLatencySamples { 0 };

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKPhaseCorrectorAudioProcessor)
};
