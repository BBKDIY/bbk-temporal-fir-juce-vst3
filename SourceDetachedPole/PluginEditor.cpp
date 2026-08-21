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

BBKDetachedPoleAudioProcessorEditor::BBKDetachedPoleAudioProcessorEditor (BBKDetachedPoleAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    prepareLabel (title, 22.0f, true);
    title.setText ("BBK Detached Pole", juce::dontSendNotification);
    addAndMakeVisible (title);

    prepareLabel (subtitle, 12.0f);
    subtitle.setText ("A/B/C loopback comparator: Bypass vs Case B (FIR only) vs Case C (FIR + pole)",
                       juce::dontSendNotification);
    addAndMakeVisible (subtitle);

    prepareLabel (sampleRate);
    addAndMakeVisible (sampleRate);

    prepareLabel (status);
    addAndMakeVisible (status);

    modeBox.addItem ("BYPASS", 1);
    modeBox.addItem ("CASE B - FIR only (19 taps)", 2);
    modeBox.addItem ("CASE C - FIR + 47 kHz pole", 3);
    modeBox.setJustificationType (juce::Justification::centred);
    addAndMakeVisible (modeBox);
    modeAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ComboBoxAttachment> (
        processor.getAPVTS(), "mode", modeBox);

    prepareLabel (metricsReadout, 13.0f);
    metricsReadout.setJustificationType (juce::Justification::centredLeft);
    addAndMakeVisible (metricsReadout);

    setSize (640, 340);
    startTimerHz (4);
    timerCallback();
}

void BBKDetachedPoleAudioProcessorEditor::paint (juce::Graphics& g)
{
    g.fillAll (juce::Colour (0xff171717));
    g.setColour (juce::Colour (0xff505050));
    g.drawRoundedRectangle (getLocalBounds().toFloat().reduced (8.0f), 8.0f, 1.0f);
}

void BBKDetachedPoleAudioProcessorEditor::resized()
{
    auto area = getLocalBounds().reduced (20);

    title.setBounds (area.removeFromTop (32));
    subtitle.setBounds (area.removeFromTop (20));
    area.removeFromTop (10);

    sampleRate.setBounds (area.removeFromTop (20));
    status.setBounds (area.removeFromTop (20));
    area.removeFromTop (10);

    modeBox.setBounds (area.removeFromTop (28).reduced (60, 0));
    area.removeFromTop (14);

    metricsReadout.setBounds (area);
}

void BBKDetachedPoleAudioProcessorEditor::timerCallback()
{
    const double sr = processor.getCurrentSampleRateForUI();
    sampleRate.setText ("Sample rate: " + juce::String (sr, 0) + " Hz", juce::dontSendNotification);

    const bool valid = processor.isRateValidForUI();
    if (! valid)
    {
        status.setColour (juce::Label::textColourId, juce::Colours::orangered);
        status.setText ("REQUIRES 192 kHz - plugin is hard-bypassed at this rate", juce::dontSendNotification);
        metricsReadout.setText ({}, juce::dontSendNotification);
        return;
    }

    status.setColour (juce::Label::textColourId, juce::Colours::lightgreen);
    status.setText ("Active at 192 kHz", juce::dontSendNotification);

    using namespace bbk::detachedpole;
    const auto mode = processor.getModeForUI();

    juce::String text;
    switch (mode)
    {
        case BBKDetachedPoleAudioProcessor::Mode::bypass:
            text = "BYPASS: dry signal, delayed to match the other two modes (no filtering).";
            break;

        case BBKDetachedPoleAudioProcessor::Mode::caseB:
            text << "CASE B - 19-tap FIR only (no pole):\n"
                 << "Peak sidelobe " << juce::String (caseBPeakSidelobePercent, 2) << "%   "
                 << "Total ringing energy " << juce::String (caseBTotalRingingEnergyPercent, 2) << "%\n"
                 << "Duration " << caseBDurationSamples << " samples   "
                 << "Worst stopband " << juce::String (caseBWorstStopbandLevelDb, 1) << " dB   "
                 << "Group delay " << caseBGroupDelaySamples << " samples (exact, no droop)";
            break;

        case BBKDetachedPoleAudioProcessor::Mode::caseC:
            text << "CASE C - 19-tap FIR + decoupled 47 kHz real pole:\n"
                 << "Peak sidelobe " << juce::String (peakSidelobePercent, 2) << "%   "
                 << "Total ringing energy " << juce::String (totalRingingEnergyPercent, 2) << "%\n"
                 << "Duration " << durationSamples << " samples (" << juce::String (durationMicroseconds, 3) << " us)   "
                 << "Worst stopband " << juce::String (worstStopbandLevelDb, 1) << " dB\n"
                 << "Droop at 20 kHz " << juce::String (passbandDroopDbAt20k, 2) << " dB   "
                 << "Mean group delay " << juce::String (groupDelaySamples, 3) << " samples (flat +/-0.002)";
            break;
    }

    metricsReadout.setText (text, juce::dontSendNotification);
}
