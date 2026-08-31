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
    subtitle.setText ("Parametric constrained-least-squares FIR lowpass - auto-detects sample rate "
                       "(44.1/48/88.2/96/176.4/192/384 kHz, or any other rate the host reports)",
                       juce::dontSendNotification);
    addAndMakeVisible (subtitle);

    prepareLabel (sampleRate);
    addAndMakeVisible (sampleRate);

    bypassButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (bypassButton);
    bypassAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "bypass", bypassButton);

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
    // 4 decimal places (not the default 2) and a wider text box: the
    // parameter's own step is now 0.0001 dB (see
    // PluginProcessor.cpp::createParameterLayout()) specifically so
    // research operating points like 0.0027 dB are reachable - a 2-decimal
    // display would visually round that right back to "0.00" even though
    // the stored value is exact, which is confusing to type against.
    attenuationSlider.setNumDecimalPlacesToDisplay (4);
    attenuationSlider.setTextBoxStyle (juce::Slider::TextBoxRight, false, 110, 22);
    addAndMakeVisible (attenuationSlider);
    attenuationAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "attenuation", attenuationSlider);

    // On: the slider above is used as typed (spectrally relaxed, Case
    // C-style). Off: the slider is ignored entirely and the exact
    // calibrated near-flat Case B point is used instead - deterministic,
    // not dependent on typing or host rounding at 0.0001 dB precision
    // (see DetachedPoleFilter.h::caseBNearFlatAttenuationDb).
    amplitudeRelaxationButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (amplitudeRelaxationButton);
    amplitudeRelaxationAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "amplitudeRelaxation", amplitudeRelaxationButton);

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

    setSize (680, 575);
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

    {
        auto row = area.removeFromTop (24);
        bypassButton.setBounds (row.removeFromRight (100));
        sampleRate.setBounds (row);
    }
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

    amplitudeRelaxationButton.setBounds (area.removeFromTop (24));
    area.removeFromTop (6);

    sliderRow (stopbandLabel, stopbandSlider);

    area.removeFromTop (10);
    metricsReadout.setBounds (area.removeFromTop (175));

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

    // The attenuation slider is a no-op while relaxation is off (the
    // engine uses the fixed calibrated constant instead - see
    // specFromParameters()), so grey it out rather than leave it looking
    // live and misleading.
    attenuationSlider.setEnabled (snap.amplitudeRelaxationOn);

    juce::String text;
    if (auto* bypassParam = processor.getAPVTS().getRawParameterValue ("bypass"))
        if (bypassParam->load() > 0.5f)
            text << "BYPASSED (dry signal, delay-matched - no filtering audible)\n";

    text << "Design: " << snap.tapCount << " taps, group delay "
         << bbk::detachedpole::latencySamples << " samples fixed (host-reported latency never changes)\n"
         << "Amplitude relaxation: " << (snap.amplitudeRelaxationOn
              ? "ON - attenuation slider used as set (Case C-style spectral relaxation)"
              : "OFF - attenuation slider ignored, fixed at the calibrated near-flat Case B "
                "point (deterministic, not typed)")
         << "\n"
         << "Target: cutoff " << juce::String (snap.cutoffHz, 0) << " Hz, "
         << juce::String (snap.attenuationAtCutoffDb, 4) << " dB at cutoff, "
         << juce::String (snap.stopbandRejectionDb, 1) << " dB min. stopband rejection\n"
         << "Stopband mode: free transition, cutoff to Nyquist - only a narrow guard band right at "
            "Nyquist is held to the floor; most of that span may sit well above it. Safe only if "
            "nothing downstream can fold that energy back into the audible band.\n"
         << "Achieved worst-case level in the enforced region: " << juce::String (snap.achievedStopbandDb, 2)
         << " dB (Nyquist = " << juce::String (nyquist, 0) << " Hz)\n"
         << "Design attempts (tap-count search): " << snap.designAttempts << "\n"
         << "Temporal concentration (from the article's own metrics):\n"
         << "  R_peak " << juce::String (snap.temporal.rPeakPercent, 2) << "%  |  E_ZC "
         << juce::String (snap.temporal.eZcPercent, 3) << "%  |  T_0.1% "
         << juce::String (snap.temporal.settlingMs, 4) << " ms (" << snap.temporal.settlingSampleSpan << " samples)\n";

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
