#include "PluginEditor.h"

#include <cmath>

BBKPhaseCorrectorAudioProcessorEditor::BBKPhaseCorrectorAudioProcessorEditor (BBKPhaseCorrectorAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    setSize (560, 230);

    titleLabel.setText ("BBK PHASE CORRECTOR", juce::dontSendNotification);
    titleLabel.setJustificationType (juce::Justification::centred);
    titleLabel.setFont (juce::FontOptions (24.0f, juce::Font::bold));
    addAndMakeVisible (titleLabel);

    infoLabel.setText ("Phase-only correction  |  magnitude EQ: none", juce::dontSendNotification);
    infoLabel.setJustificationType (juce::Justification::centred);
    addAndMakeVisible (infoLabel);

    for (auto* button : { &bypassButton, &minimumButton, &linearButton })
    {
        button->setClickingTogglesState (true);
        button->setRadioGroupId (1701);
        addAndMakeVisible (*button);
    }

    bypassButton.onClick = [this] { setModeFromUI (0); };
    minimumButton.onClick = [this] { setModeFromUI (1); };
    linearButton.onClick = [this] { setModeFromUI (2); };

    latencyLabel.setJustificationType (juce::Justification::centred);
    addAndMakeVisible (latencyLabel);

    startTimerHz (10);
    timerCallback();
}

void BBKPhaseCorrectorAudioProcessorEditor::paint (juce::Graphics& g)
{
    g.fillAll (juce::Colour::fromRGB (24, 24, 26));

    g.setColour (juce::Colour::fromRGB (62, 62, 66));
    g.drawRoundedRectangle (getLocalBounds().toFloat().reduced (12.0f), 10.0f, 1.0f);
}

void BBKPhaseCorrectorAudioProcessorEditor::resized()
{
    auto area = getLocalBounds().reduced (24);
    titleLabel.setBounds (area.removeFromTop (38));
    infoLabel.setBounds (area.removeFromTop (28));
    area.removeFromTop (20);

    auto buttons = area.removeFromTop (52);
    const auto gap = 10;
    const auto buttonWidth = (buttons.getWidth() - 2 * gap) / 3;
    bypassButton.setBounds (buttons.removeFromLeft (buttonWidth));
    buttons.removeFromLeft (gap);
    minimumButton.setBounds (buttons.removeFromLeft (buttonWidth));
    buttons.removeFromLeft (gap);
    linearButton.setBounds (buttons);

    area.removeFromTop (18);
    latencyLabel.setBounds (area.removeFromTop (28));
}

void BBKPhaseCorrectorAudioProcessorEditor::setModeFromUI (int modeIndex)
{
    if (auto* parameter = processor.parameters.getParameter ("mode"))
    {
        parameter->beginChangeGesture();
        parameter->setValueNotifyingHost (parameter->convertTo0to1 (static_cast<float> (modeIndex)));
        parameter->endChangeGesture();
    }
}

void BBKPhaseCorrectorAudioProcessorEditor::timerCallback()
{
    const auto mode = juce::jlimit (0, 2,
        static_cast<int> (std::lround (processor.parameters.getRawParameterValue ("mode")->load())));

    bypassButton.setToggleState (mode == 0, juce::dontSendNotification);
    minimumButton.setToggleState (mode == 1, juce::dontSendNotification);
    linearButton.setToggleState (mode == 2, juce::dontSendNotification);

    const auto sr = processor.getCurrentSampleRateForUI();
    const auto latency = processor.getPhaseLatencySamplesForUI();

    if (! processor.isSampleRateSupportedForUI())
    {
        latencyLabel.setText ("Sample rate not supported (needs >= 40 kHz) - "
                              "MIN/LINEAR PHASE are passing through unmodified",
                              juce::dontSendNotification);
    }
    else if (sr > 0.0 && latency > 0)
    {
        latencyLabel.setText ("Matched A/B latency: "
                              + juce::String (1000.0 * static_cast<double> (latency) / sr, 1)
                              + " ms",
                              juce::dontSendNotification);
    }
    else
    {
        latencyLabel.setText ("Latency is matched in all three modes", juce::dontSendNotification);
    }
}
