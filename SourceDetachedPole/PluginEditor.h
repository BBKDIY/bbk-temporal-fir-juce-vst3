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
    juce::ToggleButton amplitudeRelaxationButton { "Amplitude Relaxation" };

    // On: cutoff/stopband/sidelobeDecay are forced to the precomputed
    // preset-bank operating point (18.5 kHz / 95 dB / decay 1.0) and the
    // matching filter for the current sample rate + attenuation step loads
    // instantly, no background search. Those three sliders are greyed out
    // (still showing the forced values, per the chosen UI - see
    // timerCallback()) while this is on; Attenuation stays live since it's
    // what selects which of the 10 precomputed steps is used.
    juce::ToggleButton defaultModeButton { "Default (instant, prebuilt)" };

    juce::Label cutoffLabel;
    juce::Slider cutoffSlider;
    juce::Label attenuationLabel;
    juce::Slider attenuationSlider;
    juce::Label stopbandLabel;
    juce::Slider stopbandSlider;
    juce::Label sidelobeDecayLabel;
    juce::Slider sidelobeDecaySlider;

    // Manual/Auto tap-count selector - see "tapCountAuto"/"manualTapCount"
    // in PluginProcessor.cpp::createParameterLayout(). manualTapCountSlider
    // is greyed out (see timerCallback()) whenever tapCountAutoButton is
    // checked (Auto mode ignores it entirely) or Default mode is on (same
    // reasoning as cutoffSlider/stopbandSlider/sidelobeDecaySlider above).
    juce::Label manualTapCountLabel;
    juce::Slider manualTapCountSlider;
    juce::ToggleButton tapCountAutoButton { "Auto" };

    juce::Label headroomCaption;
    juce::Slider headroomSlider;
    juce::ToggleButton autoHeadroomButton { "Auto" };
    juce::Label clipIndicator; // lit red for a short hold time after the soft-clip backstop engages

    juce::Label metricsReadout;
    juce::TextButton coefficientsButton { "Show Coefficients" };
    juce::TextEditor coefficientsBox;
    bool coefficientsVisible = false;

    // Persists whatever is currently published (Default, Custom, or an
    // existing override) as a user override for its own exact spec - see
    // BBKDetachedPoleAudioProcessor::saveCurrentAsOverride(). Always
    // enabled; saving an already-instant result is harmless, just pointless.
    juce::TextButton saveAsDefaultButton { "Save as Default" };

    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> cutoffAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> attenuationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> stopbandAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> sidelobeDecayAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> manualTapCountAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> tapCountAutoAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> bypassAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> amplitudeRelaxationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> defaultModeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> headroomAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> autoHeadroomAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessorEditor)
};
