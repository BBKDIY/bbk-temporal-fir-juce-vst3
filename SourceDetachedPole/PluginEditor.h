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

    BBKDetachedPoleAudioProcessor& processor;

    juce::Label title;
    juce::Label subtitle;
    juce::Label sampleRate;
    juce::Label status;
    juce::ComboBox modeBox;

    juce::Label metricsReadout;

    std::unique_ptr<juce::AudioProcessorValueTreeState::ComboBoxAttachment> modeAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessorEditor)
};
