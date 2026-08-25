#include "PluginEditor.h"

namespace
{
void prepareLabel (juce::Label& label, float size = 14.0f, bool bold = false,
                    juce::Justification justification = juce::Justification::centred)
{
    label.setJustificationType (justification);
    label.setColour (juce::Label::textColourId, juce::Colours::white);
    label.setFont (juce::Font (size, bold ? juce::Font::bold : juce::Font::plain));
}

void prepareSlider (juce::Slider& slider)
{
    slider.setSliderStyle (juce::Slider::LinearHorizontal);
    slider.setTextBoxStyle (juce::Slider::TextBoxRight, false, 90, 22);
    slider.setColour (juce::Slider::trackColourId, juce::Colour (0xff4a90d9));
}
}

BBKDetachedPoleAudioProcessorEditor::BBKDetachedPoleAudioProcessorEditor (BBKDetachedPoleAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    prepareLabel (title, 22.0f, true);
    title.setText ("BBK Parametric FIR", juce::dontSendNotification);
    addAndMakeVisible (title);

    prepareLabel (subtitle, 12.0f);
    subtitle.setText ("Parametric constrained-least-squares FIR lowpass - auto-detects sample rate (44.1/48/96/192 kHz)",
                       juce::dontSendNotification);
    addAndMakeVisible (subtitle);

    prepareLabel (sampleRate);
    addAndMakeVisible (sampleRate);

    prepareLabel (cutoffLabel, 13.0f, false, juce::Justification::centredLeft);
    cutoffLabel.setText ("Cutoff", juce::dontSendNotification);
    addAndMakeVisible (cutoffLabel);
    prepareSlider (cutoffSlider);
    addAndMakeVisible (cutoffSlider);
    cutoffAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "cutoff", cutoffSlider);

    prepareLabel (attenuationLabel, 13.0f, false, juce::Justification::centredLeft);
    attenuationLabel.setText ("Attenuation at Cutoff", juce::dontSendNotification);
    addAndMakeVisible (attenuationLabel);
    prepareSlider (attenuationSlider);
    addAndMakeVisible (attenuationSlider);
    attenuationAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "attenuation", attenuationSlider);

    prepareLabel (stopbandLabel, 13.0f, false, juce::Justification::centredLeft);
    stopbandLabel.setText ("Min. Stopband Rejection", juce::dontSendNotification);
    addAndMakeVisible (stopbandLabel);
    prepareSlider (stopbandSlider);
    addAndMakeVisible (stopbandSlider);
    stopbandAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "stopband", stopbandSlider);

    prepareLabel (metricsReadout, 13.0f, false, juce::Justification::centredLeft);
    addAndMakeVisible (metricsReadout);

    coefficientsButton.onClick = [this] { toggleCoefficientsPopup(); };
    addAndMakeVisible (coefficientsButton);

    coefficientsBox.setMultiLine (true);
    coefficientsBox.setReadOnly (true);
    coefficientsBox.setScrollbarsShown (true);
    coefficientsBox.setCaretVisible (false);
    coefficientsBox.setFont (juce::Font (juce::Font::getDefaultMonospacedFontName(), 12.0f, juce::Font::plain));
    coefficientsBox.setColour (juce::TextEditor::backgroundColourId, juce::Colour (0xff0d0d0d));
    coefficientsBox.setColour (juce::TextEditor::textColourId, juce::Colours::lightgreen);
    coefficientsBox.setVisible (false);
    addChildComponent (coefficientsBox);

    setSize (680, 480);
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

    title.setBounds (area.removeFromTop (30));
    subtitle.setBounds (area.removeFromTop (18));
    area.removeFromTop (6);

    sampleRate.setBounds (area.removeFromTop (20));
    area.removeFromTop (10);

    auto sliderRow = [&] (juce::Label& label, juce::Slider& slider)
    {
        auto row = area.removeFromTop (26);
        label.setBounds (row.removeFromLeft (170));
        slider.setBounds (row);
        area.removeFromTop (6);
    };

    sliderRow (cutoffLabel, cutoffSlider);
    sliderRow (attenuationLabel, attenuationSlider);
    sliderRow (stopbandLabel, stopbandSlider);

    area.removeFromTop (10);
    metricsReadout.setBounds (area.removeFromTop (110));

    area.removeFromTop (8);
    coefficientsButton.setBounds (area.removeFromTop (26).removeFromLeft (200));

    area.removeFromTop (8);
    coefficientsBox.setBounds (area);
}

void BBKDetachedPoleAudioProcessorEditor::toggleCoefficientsPopup()
{
    coefficientsVisible = ! coefficientsVisible;
    coefficientsBox.setVisible (coefficientsVisible);
    coefficientsButton.setButtonText (coefficientsVisible ? "Hide Coefficients" : "Show Coefficients");
    if (coefficientsVisible)
        refreshCoefficientsText (processor.getDesignSnapshotForUI());
}

void BBKDetachedPoleAudioProcessorEditor::refreshCoefficientsText (const BBKDetachedPoleAudioProcessor::DesignSnapshot& snap)
{
    juce::String text;
    text << snap.tapCount << "-tap symmetric FIR (unity DC gain), designed for "
         << juce::String (snap.sampleRateHz, 0) << " Hz:\n\n";

    for (int i = 0; i < static_cast<int> (snap.taps.size()); ++i)
    {
        text << juce::String (snap.taps[static_cast<std::size_t> (i)], 12);
        if (i != static_cast<int> (snap.taps.size()) - 1)
            text << ",";
        text << ((i % 4 == 3) ? "\n" : "  ");
    }

    coefficientsBox.setText (text, juce::dontSendNotification);
}

void BBKDetachedPoleAudioProcessorEditor::timerCallback()
{
    const auto snap = processor.getDesignSnapshotForUI();

    // Read directly from the host, not from the design snapshot - the
    // snapshot only updates when a redesign finishes, which lagged behind
    // (or in some host round-trips, never actually reflected) the real
    // current sample rate.
    sampleRate.setText ("Sample rate: " + juce::String (processor.getCurrentSampleRateForUI(), 0) + " Hz", juce::dontSendNotification);

    if (snap.tapCount == 0)
    {
        metricsReadout.setText ("Designing initial filter...", juce::dontSendNotification);
        return;
    }

    const double nyquist = snap.sampleRateHz * 0.5;

    juce::String text;
    text << "Design: " << snap.tapCount << " taps, group delay "
         << bbk::detachedpole::latencySamples << " samples fixed (host-reported latency never changes)\n"
         << "Target: cutoff " << juce::String (snap.cutoffHz, 0) << " Hz, "
         << juce::String (snap.attenuationAtCutoffDb, 2) << " dB at cutoff, "
         << juce::String (snap.stopbandRejectionDb, 1) << " dB min. stopband rejection\n"
         << "Achieved worst-case stopband level: " << juce::String (snap.achievedStopbandDb, 2) << " dB (Nyquist = "
         << juce::String (nyquist, 0) << " Hz)\n"
         << "Design attempts (tap-count search): " << snap.designAttempts << "\n";

    if (snap.constraintsMet)
    {
        text << "Status: targets met.";
    }
    else
    {
        text << "Status: best effort at the " << bbk::detachedpole::maxTapCount
             << "-tap cap - targets not fully reached. Try relaxing a slider (lower cutoff, more "
                "attenuation headroom, or a shallower stopband floor).";
    }

    metricsReadout.setText (text, juce::dontSendNotification);

    if (coefficientsVisible)
        refreshCoefficientsText (snap);
}
