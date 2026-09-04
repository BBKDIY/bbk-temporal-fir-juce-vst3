#include "PluginEditor.h"

#include <cmath>

BBKPhaseCorrectorAudioProcessorEditor::BBKPhaseCorrectorAudioProcessorEditor (BBKPhaseCorrectorAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    setSize (560, 280);

    titleLabel.setText ("BBK PHASE CORRECTOR", juce::dontSendNotification);
    titleLabel.setJustificationType (juce::Justification::centred);
    titleLabel.setFont (juce::FontOptions (24.0f, juce::Font::bold));
    addAndMakeVisible (titleLabel);

    infoLabel.setText ("Phase-only correction  |  magnitude EQ: none  |  MIN/LINEAR PHASE padded for peak headroom",
                       juce::dontSendNotification);
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

    headroomCaption.setText ("Headroom (dB)", juce::dontSendNotification);
    headroomCaption.setJustificationType (juce::Justification::centredRight);
    addAndMakeVisible (headroomCaption);

    // IncDecButtons gives a typeable numeric box (click the number to edit
    // it directly, or use the +/- arrows) rather than a drag slider - this
    // is what was asked for: a plain "type in a number" field, not a knob.
    headroomSlider.setSliderStyle (juce::Slider::IncDecButtons);
    headroomSlider.setTextBoxStyle (juce::Slider::TextBoxLeft, false, 64, 24);
    headroomSlider.setIncDecButtonsMode (juce::Slider::incDecButtonsDraggable_Vertical);
    addAndMakeVisible (headroomSlider);
    headroomAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.parameters, "headroom", headroomSlider);

    // SliderAttachment syncs parameter -> UI with dontSendNotification, so
    // onValueChange only fires for genuine user interaction (typing,
    // clicking the arrows), never for a value pushed in from outside (host
    // automation or the processor's own auto-headroom ratchet). That makes
    // this the right hook for "typing overrides Auto": any real user edit
    // turns Auto off so the typed value sticks.
    headroomSlider.onValueChange = [this]
    {
        if (auto* autoParam = processor.parameters.getParameter ("autoHeadroom"))
        {
            if (autoParam->getValue() > 0.5f)
            {
                autoParam->beginChangeGesture();
                autoParam->setValueNotifyingHost (0.0f);
                autoParam->endChangeGesture();
            }
        }
    };

    autoHeadroomButton.setButtonText ("Auto");
    addAndMakeVisible (autoHeadroomButton);
    autoHeadroomAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.parameters, "autoHeadroom", autoHeadroomButton);

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

    auto headroomRow = area.removeFromTop (28);
    headroomCaption.setBounds (headroomRow.removeFromLeft (120));
    headroomRow.removeFromLeft (10);
    headroomSlider.setBounds (headroomRow.removeFromLeft (130));
    headroomRow.removeFromLeft (16);
    autoHeadroomButton.setBounds (headroomRow.removeFromLeft (70));

    area.removeFromTop (14);
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
