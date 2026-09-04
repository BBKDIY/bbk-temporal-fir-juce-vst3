#include "PluginEditor.h"

#include <cmath>

BBKPhaseCorrectorAudioProcessorEditor::BBKPhaseCorrectorAudioProcessorEditor (BBKPhaseCorrectorAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    setSize (560, 320);

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

    // IMPORTANT: this is onDragStart, not onValueChange. Checked against the
    // actual JUCE source: SliderParameterAttachment pushes parameter->UI
    // updates via slider.setValue(v, sendNotificationSync), which DOES fire
    // onValueChange - so onValueChange fires for every change regardless of
    // origin, including the processor's own auto-headroom ratchet, which
    // would immediately (and wrongly) look like a user edit and turn Auto
    // back off after its very first adjustment. onDragStart is different:
    // JUCE wraps every genuine user gesture - mouse drag, IncDecButtons
    // clicks, and committing typed text - in a ScopedDragNotification that
    // fires onDragStart/onDragEnd, but SliderParameterAttachment's
    // programmatic pushes do not go through that path at all. So
    // onDragStart is the correct signal for "the user just started
    // editing this control themselves."
    headroomSlider.onDragStart = [this]
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

    depthCaption.setText ("Correction Depth (%)", juce::dontSendNotification);
    depthCaption.setJustificationType (juce::Justification::centredRight);
    addAndMakeVisible (depthCaption);

    // How much of the measured phase correction to actually apply - manual
    // only (no auto-calibration for this one, unlike headroom): how much
    // correction is worth its side effects is a judgment call to make by
    // ear for your own room/system/material, the same way Acourate's own
    // correction-strength control works. Lower this if MIN/LINEAR PHASE
    // still hits the soft-clip backstop even with a lot of Headroom pad.
    depthSlider.setSliderStyle (juce::Slider::IncDecButtons);
    depthSlider.setTextBoxStyle (juce::Slider::TextBoxLeft, false, 64, 24);
    depthSlider.setIncDecButtonsMode (juce::Slider::incDecButtonsDraggable_Vertical);
    addAndMakeVisible (depthSlider);
    depthAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.parameters, "correctionDepth", depthSlider);

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

    area.removeFromTop (12);

    auto depthRow = area.removeFromTop (28);
    depthCaption.setBounds (depthRow.removeFromLeft (120));
    depthRow.removeFromLeft (10);
    depthSlider.setBounds (depthRow.removeFromLeft (130));

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
