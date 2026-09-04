#pragma once

#include <JuceHeader.h>
#include "PhaseFilterDesigner.h"

class BBKPhaseCorrectorAudioProcessor final : public juce::AudioProcessor,
                                               private juce::Timer
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

    // PhaseFilterDesigner::design() throws for sample rates below 40 kHz
    // (the measured phase/magnitude model was captured for a 20 Hz-19 kHz
    // passband, which leaves no sensible transition band above 19 kHz at
    // lower rates). prepareToPlay() catches that and falls back to an
    // identity pass-through rather than letting the exception escape
    // across the host boundary - see prepareToPlay() for details. This
    // reports that fallback state so the editor can warn the user rather
    // than silently showing MIN PHASE/LINEAR PHASE as if they were doing
    // real correction.
    bool isSampleRateSupportedForUI() const noexcept { return sampleRateSupported.load(); }

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    static juce::AudioBuffer<float> makeImpulseBuffer (const std::vector<float>& impulse);

    void initialiseDryDelay (int channels, int latencySamples);
    void processDryDelay (const juce::dsp::AudioBlock<const float>& input,
                          juce::dsp::AudioBlock<float>& output) noexcept;
    void updateModeTargets (int mode);

    // Polls "correctionDepth" on the message thread (NOT the audio thread)
    // and, if it changed since the last time we applied it, redesigns both
    // impulse responses at the new depth and hot-swaps them in via
    // loadImpulseResponse(). That call is safe to make anytime after
    // prepare() - JUCE defers the actual FFT-based engine build to
    // Convolution's own background thread and crossfades to it over ~50ms,
    // so this never blocks or glitches processBlock() even though a
    // recompute of a 0.68s impulse response is too heavy to do inline on
    // the audio thread. fftSize/latency never change with depth (only
    // sampleRate affects those), so the dry-delay length and reported
    // plugin latency stay valid across a depth change with no other
    // bookkeeping needed.
    void timerCallback() override;

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
    std::atomic<bool> sampleRateSupported { true };

    // Auto-headroom calibration state - audio-thread only, no need for atomics.
    // clipEnvelope is a leaky peak-hold of how far the corrected (MIN/LINEAR
    // PHASE) signal has recently pushed past the soft-clip knee; when it
    // stays above autoHeadroomTriggerLinear for a sustained period AND the
    // cooldown has elapsed, the "headroom" parameter is ratcheted one step
    // more negative via setValueNotifyingHost(). One-way only - it never
    // loosens itself back up - see PluginProcessor.cpp for the full design
    // rationale.
    float clipEnvelope = 0.0f;
    int samplesUntilNextAutoAdjust = 0;
    int autoAdjustCooldownSamples = 0;

    // Message-thread only: last correction depth (0..1) actually baked into
    // the currently-loaded impulse responses, so timerCallback() only
    // triggers a redesign when the "correctionDepth" parameter has actually
    // moved. -1 sentinel forces the first check (right after prepareToPlay)
    // to be a no-op, since prepareToPlay() already designs at the current
    // depth itself.
    float lastAppliedCorrectionDepth = -1.0f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKPhaseCorrectorAudioProcessor)
};
