#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

class BBKPhaseCorrectorAudioProcessorEditor final : public juce::AudioProcessorEditor,
                                                     private juce::Timer
{
public:
    explicit BBKPhaseCorrectorAudioProcessorEditor (BBKPhaseCorrectorAudioProcessor&);
    ~BBKPhaseCorrectorAudioProcessorEditor() override = default;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    void timerCallback() override;
    void setModeFromUI (int modeIndex);

    BBKPhaseCorrectorAudioProcessor& processor;

    juce::Label titleLabel;
    juce::Label infoLabel;
    juce::Label latencyLabel;
    juce::TextButton bypassButton { "BYPASS" };
    juce::TextButton minimumButton { "MIN PHASE" };
    juce::TextButton linearButton { "LINEAR PHASE" };

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKPhaseCorrectorAudioProcessorEditor)
};
