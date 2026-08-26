#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include "PluginProcessor.h"

class BBKDetachedPoleAudioProcessorEditor final : public juce::AudioProcessorEditor,
                                                   private juce::Timer
{
public:
    explicit BBKDetachedPoleAudioProcessorEditor (BBKDetachedPoleAudioProcessor&);
    ~BBKDetachedPoleAudioProcessorEditor() override = default;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    void timerCallback() override;
    void toggleCoefficientsPopup();
    void refreshCoefficientsText (const BBKDetachedPoleAudioProcessor::DesignSnapshot&);

    BBKDetachedPoleAudioProcessor& processor;

    juce::Label title;
    juce::Label subtitle;
    juce::Label sampleRate;
    juce::ToggleButton bypassButton { "Bypass" };
    juce::ToggleButton freeTransitionButton { "Free Transition (fc to Nyquist)" };

    juce::Label cutoffLabel;
    juce::Slider cutoffSlider;
    juce::Label attenuationLabel;
    juce::Slider attenuationSlider;
    juce::Label stopbandLabel;
    juce::Slider stopbandSlider;

    juce::Label metricsReadout;
    juce::TextButton coefficientsButton { "Show Coefficients" };
    juce::TextEditor coefficientsBox;
    bool coefficientsVisible = false;

    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> cutoffAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> attenuationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> stopbandAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> bypassAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> freeTransitionAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessorEditor)
};
