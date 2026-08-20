#include "PluginEditor.h"

namespace
{
void prepareLabel (juce::Label& label, float size = 14.0f, bool bold = false)
{
    label.setJustificationType (juce::Justification::centred);
    label.setColour (juce::Label::textColourId, juce::Colours::white);
    label.setFont (juce::Font (size, bold ? juce::Font::bold : juce::Font::plain));
}
}

BBKTemporalFIRAudioProcessorEditor::BBKTemporalFIRAudioProcessorEditor (BBKTemporalFIRAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    prepareLabel (title, 22.0f, true);
    title.setText ("BBK Temporal FIR", juce::dontSendNotification);
    addAndMakeVisible (title);

    prepareLabel (sampleRate);
    addAndMakeVisible (sampleRate);

    prepareLabel (status);
    addAndMakeVisible (status);

    enable.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (enable);
    enableAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "enabled", enable);

    prepareLabel (tradeoffTitle, 14.0f, true);
    tradeoffTitle.setText ("TEMPORAL TRADE-OFF", juce::dontSendNotification);
    addAndMakeVisible (tradeoffTitle);

    tradeoffSlider.setSliderStyle (juce::Slider::LinearHorizontal);
    tradeoffSlider.setTextBoxStyle (juce::Slider::NoTextBox, false, 0, 0);
    tradeoffSlider.setRange (0, bbk::temporalfir::numFilterPoints - 1, 1);
    tradeoffSlider.setColour (juce::Slider::trackColourId, juce::Colour (0xff3a7bd5));
    tradeoffSlider.setColour (juce::Slider::thumbColourId, juce::Colours::white);
    addAndMakeVisible (tradeoffSlider);
    sliderAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "tradeoffIndex", tradeoffSlider);

    prepareLabel (leftEndLabel, 11.0f);
    leftEndLabel.setText ("SHORTEST RESPONSE", juce::dontSendNotification);
    leftEndLabel.setJustificationType (juce::Justification::centredLeft);
    addAndMakeVisible (leftEndLabel);

    prepareLabel (rightEndLabel, 11.0f);
    rightEndLabel.setText ("LOWEST PEAK SIDELOBE", juce::dontSendNotification);
    rightEndLabel.setJustificationType (juce::Justification::centredRight);
    addAndMakeVisible (rightEndLabel);

    prepareLabel (metricsReadout, 13.0f);
    metricsReadout.setJustificationType (juce::Justification::centredLeft);
    addAndMakeVisible (metricsReadout);

    setSize (620, 340);
    startTimerHz (4);
    timerCallback();
}

void BBKTemporalFIRAudioProcessorEditor::paint (juce::Graphics& g)
{
    g.fillAll (juce::Colour (0xff171717));
    g.setColour (juce::Colour (0xff505050));
    g.drawRoundedRectangle (getLocalBounds().toFloat().reduced (8.0f), 8.0f, 1.0f);
}

void BBKTemporalFIRAudioProcessorEditor::resized()
{
    auto area = getLocalBounds().reduced (20);
    title.setBounds (area.removeFromTop (34));
    sampleRate.setBounds (area.removeFromTop (24));
    status.setBounds (area.removeFromTop (24));
    area.removeFromTop (8);
    enable.setBounds (area.removeFromTop (30).withSizeKeepingCentre (140, 28));
    area.removeFromTop (16);

    tradeoffTitle.setBounds (area.removeFromTop (22));
    area.removeFromTop (4);
    tradeoffSlider.setBounds (area.removeFromTop (34));
    auto labelRow = area.removeFromTop (18);
    leftEndLabel.setBounds (labelRow.removeFromLeft (labelRow.getWidth() / 2));
    rightEndLabel.setBounds (labelRow);

    area.removeFromTop (18);
    metricsReadout.setBounds (area);
}

void BBKTemporalFIRAudioProcessorEditor::timerCallback()
{
    const double sr = processor.getCurrentSampleRateForUI();
    sampleRate.setText ("Host sample rate: " + juce::String (sr, 0) + " Hz",
                        juce::dontSendNotification);

    const bool rateValid = processor.isRateValidForUI();
    enable.setEnabled (rateValid);
    tradeoffSlider.setEnabled (rateValid);

    if (! rateValid)
    {
        status.setText ("REQUIRES 192 kHz", juce::dontSendNotification);
        metricsReadout.setText ({}, juce::dontSendNotification);
        return;
    }

    status.setText (processor.isEnabledForUI() ? "ACTIVE" : "BYPASS - latency matched",
                    juce::dontSendNotification);

    const auto& fp = BBKTemporalFIRAudioProcessor::filterPointFor (processor.getSelectedIndexForUI());
    juce::String text;
    text << "Tap count: " << fp.trueTapCount << "\n"
         << "Peak sidelobe: " << juce::String (fp.peakSidelobePercent, 2) << " %\n"
         << "Ringing energy: " << juce::String (fp.totalRingingEnergyPercent, 2) << " %\n"
         << "Settling: " << juce::String (fp.settlingMicroseconds, 1) << " " << juce::String (juce::CharPointer_UTF8 ("\xc2\xb5")) << "s\n"
         << "Stopband: " << juce::String (fp.worstStopbandLevelDb, 2) << " dB\n"
         << "Host rate: " << juce::String (sr, 0) << " Hz";
    metricsReadout.setText (text, juce::dontSendNotification);
}
