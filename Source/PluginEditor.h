#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include "PluginProcessor.h"

class BBKTemporalFIRAudioProcessorEditor final : public juce::AudioProcessorEditor,
                                                  private juce::Timer
{
public:
    explicit BBKTemporalFIRAudioProcessorEditor (BBKTemporalFIRAudioProcessor&);
    ~BBKTemporalFIRAudioProcessorEditor() override = default;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    void timerCallback() override;

    BBKTemporalFIRAudioProcessor& processor;

    juce::Label title;
    juce::Label sampleRate;
    juce::Label status;
    juce::ToggleButton enable { "ACTIVE" };

    juce::Slider tradeoffSlider;
    juce::Label leftEndLabel;   // "SHORTEST RESPONSE"
    juce::Label rightEndLabel;  // "LOWEST PEAK SIDELOBE"
    juce::Label tradeoffTitle;  // "TEMPORAL TRADE-OFF"

    juce::Label metricsReadout;

    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> enableAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> sliderAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKTemporalFIRAudioProcessorEditor)
};
