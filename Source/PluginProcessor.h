#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include <atomic>
#include <vector>
#include "TemporalFIRBank.h"

// BBK Temporal FIR: a real-time A/B tool for the time-domain Pareto
// trade-off between peak sidelobe amplitude and settling duration,
// described in the accompanying article and extended for this plugin's
// 192 kHz / 0-20 kHz pass / 76-96 kHz stop geometry (see
// TemporalFIRBank.h and the offline sweep scripts kept alongside this
// project). The TEMPORAL TRADE-OFF slider snaps only to the
// Pareto-efficient FIR designs the offline sweep produced - it never
// interpolates coefficients between tap counts.
//
// Every selectable filter (and bypass) is delay-matched to exactly
// bbk::temporalfir::maxGroupDelaySamples (63 samples at 192 kHz), so
// switching between filters or toggling bypass never changes the
// plugin's reported host latency.
class BBKTemporalFIRAudioProcessor final : public juce::AudioProcessor
{
public:
    BBKTemporalFIRAudioProcessor();
    ~BBKTemporalFIRAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    void processBlock (juce::AudioBuffer<double>&, juce::MidiBuffer&) override;
    bool supportsDoublePrecisionProcessing() const override { return false; }

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return "BBK Temporal FIR"; }
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
    int getSelectedIndexForUI() const noexcept;

    // Real-time-safe: reads only immutable, precomputed data.
    static const bbk::temporalfir::FilterPoint& filterPointFor (int index) noexcept
    {
        const auto& bank = bbk::temporalfir::filterBank();
        return bank[static_cast<std::size_t> (juce::jlimit (0, bbk::temporalfir::numFilterPoints - 1, index))];
    }

private:
    struct ChannelState
    {
        std::array<double, bbk::temporalfir::maxN> history {};
        int writeIndex = 0;

        void clear() noexcept
        {
            history.fill (0.0);
            writeIndex = 0;
        }
    };

    template <typename SampleType>
    void process (juce::AudioBuffer<SampleType>& buffer);

    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    static int defaultIndexForBaseline() noexcept;

    juce::AudioProcessorValueTreeState parameters;
    std::vector<ChannelState> channels;

    // enabled/bypass crossfade (dry <-> filtered), same pattern as BBK Black-19.
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Linear> wetMix;

    // filter-select crossfade (old filter's wet <-> new filter's wet).
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Linear> filterCrossfade;
    int fromFilterIndex = 0;
    int toFilterIndex = 0;
    int lastSeenParamIndex = 0;

    std::atomic<double> currentSampleRate { 0.0 };
    std::atomic<bool> valid192k { false };

    double lastPreparedSampleRate = 0.0;
    bool hasPrepared = false;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKTemporalFIRAudioProcessor)
};
