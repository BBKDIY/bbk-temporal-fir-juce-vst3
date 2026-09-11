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

    // Lit whenever processor.isSearchInProgressForUI() is true - see its
    // own comment in PluginProcessor.h. Updated every timerCallback() tick,
    // same as clipIndicator below; blank/dim the rest of the time so it
    // doesn't clutter the top row when nothing is actually being computed.
    juce::Label searchIndicator;

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

    // Custom-mode search safety net (see "maxSearchTimeValue"/
    // "maxSearchTimeIsHours" in PluginProcessor.cpp::createParameterLayout()
    // and run()'s own comment): a typable numeric value (styled like
    // headroomSlider below - click the number to edit directly, or use the
    // +/- arrows) plus an Hours toggle, defaulting to "5" / minutes. This
    // bounds how long an unattended search can run - the search itself no
    // longer has any patience limit of its own and otherwise keeps
    // searching toward the tap-count ceiling until it's stopped, either by
    // this deadline or by stopSearchButton below.
    juce::Label maxSearchTimeLabel;
    juce::Slider maxSearchTimeSlider;
    juce::ToggleButton maxSearchTimeHoursButton { "Hours" };

    // Calls processor.requestStopSearch() - immediately loads whatever the
    // best result found so far is, exactly as if the search had hit its own
    // deadline (see PluginProcessor.h). Enabled only while
    // processor.isSearchInProgressForUI() is true - see timerCallback().
    juce::TextButton stopSearchButton { "Stop" };

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
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> maxSearchTimeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> maxSearchTimeHoursAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> bypassAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> amplitudeRelaxationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> defaultModeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> headroomAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> autoHeadroomAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessorEditor)
};
