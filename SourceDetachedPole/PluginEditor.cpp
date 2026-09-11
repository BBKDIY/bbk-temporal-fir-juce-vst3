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

    // Lit whenever a background design is actually running (see
    // processor.isSearchInProgressForUI()'s own comment) - text/colour set
    // live in timerCallback(), same polling pattern as clipIndicator below.
    searchIndicator.setJustificationType (juce::Justification::centred);
    searchIndicator.setColour (juce::Label::textColourId, juce::Colours::white);
    searchIndicator.setFont (juce::Font (12.0f, juce::Font::bold));
    addAndMakeVisible (searchIndicator);

    bypassButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (bypassButton);
    bypassAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "bypass", bypassButton);

    // Default vs Custom: see the member comment on defaultModeButton. Grey-
    // out of the affected sliders happens in timerCallback() (they need to
    // track the parameter live, not just this button's own clicks - e.g.
    // host automation of "presetMode").
    defaultModeButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (defaultModeButton);
    defaultModeAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "presetMode", defaultModeButton);

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

    prepareLabel (sidelobeDecayLabel, 13.0f, false, juce::Justification::centredLeft);
    sidelobeDecayLabel.setText ("Sidelobe Decay", juce::dontSendNotification);
    addAndMakeVisible (sidelobeDecayLabel);
    prepareSlider (sidelobeDecaySlider);
    sidelobeDecaySlider.setNumDecimalPlacesToDisplay (3);
    addAndMakeVisible (sidelobeDecaySlider);
    sidelobeDecayAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "sidelobeDecay", sidelobeDecaySlider);

    // Manual/Auto tap-count selector - greying handled in timerCallback()
    // (needs to track the parameter live, e.g. host automation of
    // "tapCountAuto", same reasoning as defaultModeButton above).
    prepareLabel (manualTapCountLabel, 13.0f, false, juce::Justification::centredLeft);
    manualTapCountLabel.setText ("Manual Tap Count", juce::dontSendNotification);
    addAndMakeVisible (manualTapCountLabel);
    prepareSlider (manualTapCountSlider);
    manualTapCountSlider.setNumDecimalPlacesToDisplay (0); // always a whole (odd) tap count - see the parameter's own step
    addAndMakeVisible (manualTapCountSlider);
    manualTapCountAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "manualTapCount", manualTapCountSlider);

    tapCountAutoButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (tapCountAutoButton);
    tapCountAutoAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "tapCountAuto", tapCountAutoButton);

    // Custom-mode search safety net - see the member comment in
    // PluginEditor.h. Same typeable IncDecButtons style as headroomSlider
    // below (click the number to edit directly, or use the +/- arrows).
    prepareLabel (maxSearchTimeLabel, 13.0f, false, juce::Justification::centredLeft);
    maxSearchTimeLabel.setText ("Max Search Time", juce::dontSendNotification);
    addAndMakeVisible (maxSearchTimeLabel);
    maxSearchTimeSlider.setSliderStyle (juce::Slider::IncDecButtons);
    maxSearchTimeSlider.setTextBoxStyle (juce::Slider::TextBoxLeft, false, 60, 22);
    maxSearchTimeSlider.setIncDecButtonsMode (juce::Slider::incDecButtonsDraggable_Vertical);
    maxSearchTimeSlider.setNumDecimalPlacesToDisplay (1);
    addAndMakeVisible (maxSearchTimeSlider);
    maxSearchTimeAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "maxSearchTimeValue", maxSearchTimeSlider);

    maxSearchTimeHoursButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (maxSearchTimeHoursButton);
    maxSearchTimeHoursAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "maxSearchTimeIsHours", maxSearchTimeHoursButton);

    // Loads whatever the best result found so far is, right away - see
    // processor.requestStopSearch()'s own comment. Enabled state tracks
    // processor.isSearchInProgressForUI() live in timerCallback(), same
    // pattern as the greyed-out sliders above.
    stopSearchButton.onClick = [this] { processor.requestStopSearch(); };
    addAndMakeVisible (stopSearchButton);

    prepareLabel (headroomCaption, 13.0f, false, juce::Justification::centredLeft);
    headroomCaption.setText ("Headroom (dB)", juce::dontSendNotification);
    addAndMakeVisible (headroomCaption);

    // IncDecButtons: a typeable numeric box (click the number to edit
    // directly, or use the +/- arrows), not a drag knob.
    headroomSlider.setSliderStyle (juce::Slider::IncDecButtons);
    headroomSlider.setTextBoxStyle (juce::Slider::TextBoxLeft, false, 60, 22);
    headroomSlider.setIncDecButtonsMode (juce::Slider::incDecButtonsDraggable_Vertical);
    addAndMakeVisible (headroomSlider);
    headroomAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "headroom", headroomSlider);

    // onDragStart, NOT onValueChange: SliderParameterAttachment pushes
    // parameter->UI updates via slider.setValue(v, sendNotificationSync),
    // so onValueChange fires for every change regardless of origin,
    // including the processor's own auto-headroom ratchet - which would
    // make Auto look like it's doing nothing (it ratchets once, that looks
    // like a user edit, Auto turns itself back off). onDragStart only
    // fires for genuine user gestures - mouse drag, IncDecButtons clicks,
    // and committing typed text all wrap in a ScopedDragNotification that
    // fires it; the attachment's programmatic pushes do not go through
    // that path at all.
    headroomSlider.onDragStart = [this]
    {
        if (auto* autoParam = processor.getAPVTS().getParameter ("autoHeadroom"))
        {
            if (autoParam->getValue() > 0.5f)
            {
                autoParam->beginChangeGesture();
                autoParam->setValueNotifyingHost (0.0f);
                autoParam->endChangeGesture();
            }
        }
    };

    autoHeadroomButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    addAndMakeVisible (autoHeadroomButton);
    autoHeadroomAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "autoHeadroom", autoHeadroomButton);

    // Lit red for a short hold after the soft-clip backstop actually
    // engages (see timerCallback()) - a deliberately gentle backstop can
    // be hard to hear when it fires, which is exactly when you're trying
    // to tune Headroom down by ear. This gives an objective signal
    // instead, driven by the same detection that feeds Auto Headroom.
    clipIndicator.setText ("CLIP", juce::dontSendNotification);
    clipIndicator.setJustificationType (juce::Justification::centred);
    clipIndicator.setColour (juce::Label::textColourId, juce::Colours::white);
    clipIndicator.setColour (juce::Label::backgroundColourId, juce::Colour (0xff3a3a3a));
    clipIndicator.setFont (juce::Font (12.0f, juce::Font::bold));
    addAndMakeVisible (clipIndicator);

    // topLeft, not centredLeft: a Label vertically CENTRES its text within
    // its own bounds, so when the (wrapped, multi-line) metrics text is
    // taller than the fixed area below it gives it, centring clips BOTH
    // the top and bottom lines symmetrically - reported directly as "the
    // central part with the text is not big enough to show full text".
    // topLeft means any remaining shortfall only ever clips the bottom,
    // which is far less confusing, and pairs with generously sizing that
    // area in resized() below so clipping shouldn't normally happen at all.
    prepareLabel (metricsReadout, 13.0f, false, juce::Justification::topLeft);
    addAndMakeVisible (metricsReadout);

    coefficientsButton.onClick = [this] { toggleCoefficientsPopup(); };
    addAndMakeVisible (coefficientsButton);

    // Saves whichever row is CURRENTLY ACTIVE (snap.selectedIndex) - see
    // this button's own header comment. Re-reads the snapshot at click
    // time rather than capturing anything, so it always reflects whatever
    // is actually playing right now, including a switch made moments ago
    // via a row's own Use button below.
    saveAsDefaultButton.onClick = [this]
    {
        const auto snap = processor.getDesignSnapshotForUI();
        processor.saveTopCandidateAsOverride (snap.selectedIndex);
    };
    addAndMakeVisible (saveAsDefaultButton);

    // Forces a genuinely fresh search regardless of any existing cache/
    // override entry for the current spec - see requestFreshSearch()'s own
    // comment.
    reSearchButton.onClick = [this] { processor.requestFreshSearch(); };
    addAndMakeVisible (reSearchButton);

    prepareLabel (topCandidatesHeader, 13.0f, true, juce::Justification::centredLeft);
    topCandidatesHeader.setText ("Top Results (best R_peak first) - Custom/Auto only", juce::dontSendNotification);
    addAndMakeVisible (topCandidatesHeader);

    // One row per possible ranked candidate (see DesignSnapshot::
    // topCandidates) - text and visibility for each are filled in per-tick
    // by timerCallback() based on how many candidates the current snapshot
    // actually has; a row with nothing to show is simply hidden rather than
    // left blank, so Manual-mode/Default-mode results (which have none)
    // don't leave 5 empty rows sitting on screen.
    for (int i = 0; i < bbk::parametric::topCandidateCount; ++i)
    {
        auto& rowLabel = candidateRowLabels[static_cast<std::size_t> (i)];
        prepareLabel (rowLabel, 12.0f, false, juce::Justification::centredLeft);
        addAndMakeVisible (rowLabel);

        auto& useButton = useCandidateButtons[static_cast<std::size_t> (i)];
        useButton.setButtonText ("Use");
        useButton.onClick = [this, i] { processor.selectTopCandidate (i); };
        addAndMakeVisible (useButton);

        auto& saveButton = saveCandidateButtons[static_cast<std::size_t> (i)];
        saveButton.setButtonText ("Save");
        saveButton.onClick = [this, i] { processor.saveTopCandidateAsOverride (i); };
        addAndMakeVisible (saveButton);
    }

    coefficientsBox.setMultiLine (true);
    coefficientsBox.setReadOnly (true);
    coefficientsBox.setScrollbarsShown (true);
    coefficientsBox.setCaretVisible (false);
    coefficientsBox.setFont (juce::Font (juce::Font::getDefaultMonospacedFontName(), 12.0f, juce::Font::plain));
    coefficientsBox.setColour (juce::TextEditor::backgroundColourId, juce::Colour (0xff0d0d0d));
    coefficientsBox.setColour (juce::TextEditor::textColourId, juce::Colours::lightgreen);
    coefficientsBox.setVisible (false);
    addChildComponent (coefficientsBox);

    // Grew by 190 from the previous 971 to fit the top-N results table
    // (header + up to bbk::parametric::topCandidateCount rows) added
    // between the metrics readout and the coefficients/save-as-default row
    // - see resized() for the exact layout this accounts for.
    setSize (680, 1161);
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
        defaultModeButton.setBounds (row.removeFromRight (220));
        searchIndicator.setBounds (row.removeFromRight (130));
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
    sliderRow (sidelobeDecayLabel, sidelobeDecaySlider);

    {
        auto row = area.removeFromTop (26);
        manualTapCountLabel.setBounds (row.removeFromLeft (170));
        manualTapCountSlider.setBounds (row.removeFromLeft (140));
        row.removeFromLeft (16);
        tapCountAutoButton.setBounds (row.removeFromLeft (70));
    }
    area.removeFromTop (6);

    {
        auto row = area.removeFromTop (26);
        maxSearchTimeLabel.setBounds (row.removeFromLeft (170));
        maxSearchTimeSlider.setBounds (row.removeFromLeft (100));
        row.removeFromLeft (16);
        maxSearchTimeHoursButton.setBounds (row.removeFromLeft (70));
        row.removeFromLeft (16);
        stopSearchButton.setBounds (row.removeFromLeft (90));
        row.removeFromLeft (16);
        reSearchButton.setBounds (row.removeFromLeft (100));
    }
    area.removeFromTop (6);

    {
        auto row = area.removeFromTop (26);
        headroomCaption.setBounds (row.removeFromLeft (170));
        headroomSlider.setBounds (row.removeFromLeft (120));
        row.removeFromLeft (16);
        autoHeadroomButton.setBounds (row.removeFromLeft (70));
        row.removeFromLeft (16);
        clipIndicator.setBounds (row.removeFromLeft (56));
    }
    area.removeFromTop (6);

    area.removeFromTop (10);
    // Sized generously (up from 215) for the metrics text's actual worst-
    // case line count: the "Design method"/"Stopband mode"/"Center-tap
    // gain" lines alone wrap to 2-3 lines each at this width, plus the
    // live search-progress addendum (see timerCallback()) adds up to 2
    // more while a search is running - undersizing this clipped real
    // content, reported directly ("the central part with the text is not
    // big enough to show full text"). See setSize() below, which grew by
    // the same amount this took from the window.
    metricsReadout.setBounds (area.removeFromTop (380));

    area.removeFromTop (10);
    topCandidatesHeader.setBounds (area.removeFromTop (20));
    area.removeFromTop (4);
    for (int i = 0; i < bbk::parametric::topCandidateCount; ++i)
    {
        auto row = area.removeFromTop (24);
        auto& useButton = useCandidateButtons[static_cast<std::size_t> (i)];
        auto& saveButton = saveCandidateButtons[static_cast<std::size_t> (i)];
        useButton.setBounds (row.removeFromRight (60));
        row.removeFromRight (6);
        saveButton.setBounds (row.removeFromRight (60));
        row.removeFromRight (10);
        candidateRowLabels[static_cast<std::size_t> (i)].setBounds (row);
        area.removeFromTop (4);
    }

    area.removeFromTop (6);
    auto buttonRow = area.removeFromTop (26);
    coefficientsButton.setBounds (buttonRow.removeFromLeft (200));
    saveAsDefaultButton.setBounds (buttonRow.removeFromRight (160));

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

    // Clip indicator: held lit for a short time (600 ms) after the last
    // detected engagement so a single brief event is actually visible at
    // this timer's 4 Hz poll rate, not just flickering for one frame.
    constexpr juce::uint32 clipHoldMs = 600;
    const auto now = juce::Time::getMillisecondCounter();
    const auto sinceLastClip = now - processor.getLastClipTimeMsForUI();
    const bool clipping = sinceLastClip < clipHoldMs;
    clipIndicator.setColour (juce::Label::backgroundColourId,
                              clipping ? juce::Colours::red : juce::Colour (0xff3a3a3a));

    // Read directly from the host, not from the design snapshot - the
    // snapshot only updates when a redesign finishes, which lagged behind
    // (or in some host round-trips, never actually reflected) the real
    // current sample rate.
    sampleRate.setText ("Sample rate: " + juce::String (processor.getCurrentSampleRateForUI(), 0) + " Hz", juce::dontSendNotification);

    // See processor.isSearchInProgressForUI()'s own comment: true whenever
    // a background design is actually queued or running right now, whether
    // or not a previous result is already showing (that "previous result
    // still showing while a newer one computes" case is exactly what this
    // indicator exists for - the tapCount == 0 branch just below already
    // covers the separate "nothing has ever finished yet" case with its own
    // message). Blank the rest of the time so it doesn't clutter this row
    // when nothing is actually being computed.
    const bool searching = processor.isSearchInProgressForUI();
    searchIndicator.setText (searching ? "SEARCHING..." : "", juce::dontSendNotification);
    searchIndicator.setColour (juce::Label::textColourId,
                                searching ? juce::Colour (0xffd9a34a) : juce::Colours::transparentWhite);

    // Only meaningful while searching is true - see requestStopSearch()'s
    // own comment. Disabled the rest of the time so there's nothing to
    // click (and nothing to visually suggest a search could be running)
    // when none actually is.
    stopSearchButton.setEnabled (searching);

    // Appended to whatever metrics text follows below (or shown alone, in
    // the tapCount == 0 branch just below, when nothing has ever finished
    // designing yet) whenever a background search is actually running -
    // see getSearchProgressForUI()'s own comment. Deliberately layered
    // ALONGSIDE the still-playing previous result's own metrics rather than
    // replacing them: per the chosen design, the audio (and this readout's
    // main "Design:"/"Target:"/etc. section) stays on the last published
    // result for the whole duration of a search and only switches once,
    // either when the search finishes on its own, Stop is clicked, or the
    // safety-net deadline is hit.
    juce::String searchProgressText;
    if (searching)
    {
        const auto progress = processor.getSearchProgressForUI();
        searchProgressText << "\nSearching for a better result in the background ("
                            << progress.attemptsSoFar << " candidate(s) tried so far";
        if (progress.haveBest)
        {
            searchProgressText << ", best so far " << progress.bestTapCount << " taps, R_peak "
                                << juce::String (progress.bestRPeakPercent, 2) << "%, achieved "
                                << juce::String (progress.bestAchievedStopbandDb, 2) << " dB)";
        }
        else
        {
            searchProgressText << ", no feasible candidate yet)";
        }
        searchProgressText << " - click Stop to load it immediately, or let it keep looking.\n";
    }

    if (snap.tapCount == 0)
    {
        // Shown at cold start and again on every sample-rate change - the
        // redesign for the new rate runs entirely in the background and
        // never blocks playback; audio passes through unfiltered (delay-
        // matched, no clicks) until it completes and crossfades in. In
        // Default mode this resolves near-instantly (an instant bank
        // lookup, not a search) unless the rate isn't one of the 7 the
        // bank covers; in Custom mode the live search can now run
        // indefinitely (see requestBoundaryRedesign()'s own comment and
        // the Max Search Time safety net) rather than blocking playback.
        metricsReadout.setText ("Designing filter for " + juce::String (processor.getCurrentSampleRateForUI(), 0)
                                 + " Hz... (unfiltered pass-through meanwhile)" + searchProgressText,
                                 juce::dontSendNotification);

        // Nothing has ever finished designing yet - hide every row rather
        // than show 5 blank, still-clickable Use/Save buttons with nothing
        // behind them.
        for (int i = 0; i < bbk::parametric::topCandidateCount; ++i)
        {
            candidateRowLabels[static_cast<std::size_t> (i)].setVisible (false);
            useCandidateButtons[static_cast<std::size_t> (i)].setVisible (false);
            saveCandidateButtons[static_cast<std::size_t> (i)].setVisible (false);
        }
        return;
    }

    const double nyquist = snap.sampleRateHz * 0.5;

    // The attenuation slider is a no-op while relaxation is off (the
    // engine uses the fixed calibrated constant instead - see
    // specFromParameters()), so grey it out rather than leave it looking
    // live and misleading.
    attenuationSlider.setEnabled (snap.amplitudeRelaxationOn);

    // Default mode: cutoff/stopband/sidelobeDecay are forced to an
    // operating point the instant Default turns on (the factory bank's
    // fixed point, or a saved override's own point if one exists for this
    // sample rate - see forcePresetOperatingPoint()) and ignored by the
    // design itself, so grey them out - per the chosen UI, they stay
    // visible and keep showing the forced values (the sliders themselves
    // already reflect those values via their own attachments, since
    // forcePresetOperatingPoint() writes through the real parameters).
    // Attenuation stays enabled in both modes too - forcePresetOperatingPoint()
    // snaps it once at the moment Default turns on (to the override's own
    // attenuation if one exists, otherwise left as-is so it still picks
    // among the 10 factory steps), but afterwards it's still live: you can
    // drag it to browse the other factory steps even with Default checked.
    const bool presetModeOn = processor.getAPVTS().getRawParameterValue ("presetMode")->load() > 0.5f;
    cutoffSlider.setEnabled (! presetModeOn);
    stopbandSlider.setEnabled (! presetModeOn);
    sidelobeDecaySlider.setEnabled (! presetModeOn);

    // Manual Tap Count is a no-op while Auto is on (the engine's own
    // M-search picks the tap count instead - see run() in
    // PluginProcessor.cpp) and, same reasoning as the three sliders just
    // above, while Default mode is on (an instant bank/override lookup,
    // not a fresh design at all) - grey it out in either case so it never
    // looks live and editable when it wouldn't actually do anything.
    const bool tapCountAutoOn = processor.getAPVTS().getRawParameterValue ("tapCountAuto")->load() > 0.5f;
    manualTapCountSlider.setEnabled (! presetModeOn && ! tapCountAutoOn);

    juce::String text;
    if (auto* bypassParam = processor.getAPVTS().getRawParameterValue ("bypass"))
        if (bypassParam->load() > 0.5f)
            text << "BYPASSED (dry signal, delay-matched - no filtering audible)\n";

    juce::String designMethodText;
    switch (snap.source)
    {
        using ResultSource = BBKDetachedPoleAudioProcessor::ResultSource;
        case ResultSource::PresetBank:
            designMethodText = "Default - instant lookup in a precomputed filter bank (18.5 kHz cutoff, 95 dB "
                                "stopband, sidelobe decay 1.0, one of 10 attenuation steps chosen by the slider "
                                "above); no background search.";
            break;
        case ResultSource::UserOverride:
            designMethodText = "User Override - instant lookup of a filter you saved yourself for this exact "
                                "operating point (see Save as Default); no background search.";
            break;
        case ResultSource::SearchCache:
            designMethodText = "Custom - already searched for this exact operating point earlier (this session "
                                "or a previous one); instant recall, no fresh search.";
            break;
        case ResultSource::LiveSearch:
        default:
            designMethodText = "Custom - Minimax, the article's own minimum-peak-sidelobe method, searched "
                                "thoroughly across tap counts and stopband-edge candidates, ranked purely by "
                                "R_peak (lowest-ringing first - see the Top Results table below for the other "
                                "candidates it found).";
            break;
    }

    text << "Design: " << snap.tapCount << " taps, group delay "
         << bbk::detachedpole::latencySamples << " samples fixed (host-reported latency never changes)\n"
         << "Design method: " << designMethodText
         << "\n"
         << "Sidelobe decay: " << juce::String (snap.sidelobeDecayRatio, 3)
         << (snap.sidelobeDecayRatio >= 0.999
              ? " (flat, no decay - unchanged behaviour)"
              : " - taps farther from the main lobe are bounded more tightly, concentrating "
                "ringing near the centre with a shorter, quieter tail")
         << "\n";

    text
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
         << "Temporal concentration (from the article's own metrics):\n"
         << "  R_peak " << juce::String (snap.temporal.rPeakPercent, 2) << "%  |  E_ZC "
         << juce::String (snap.temporal.eZcPercent, 3) << "%  |  T_0.1% "
         << juce::String (snap.temporal.settlingMs, 4) << " ms (" << snap.temporal.settlingSampleSpan << " samples)\n"
         << "  Center-tap gain " << juce::String (snap.temporal.centerTapPercent, 2)
         << "% (share of a non-oversampling DAC's instantaneous impulse kept in the single "
            "centre sample; the rest is time-smeared across the other taps)\n";

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

    text << searchProgressText;

    metricsReadout.setText (text, juce::dontSendNotification);

    // Top-N table: one row per entry in snap.topCandidates (see its own
    // comment in PluginProcessor.h), best R_peak first. Metrics are
    // recomputed here rather than stored anywhere - RankedCandidate
    // deliberately only keeps taps/tapCount/achievedStopbandDb (see its own
    // comment in ParametricFIR.h), everything else is a pure function of
    // the taps, same as the main metrics text above already does via
    // snap.temporal for the active design.
    const int candidateCount = static_cast<int> (snap.topCandidates.size());
    for (int i = 0; i < bbk::parametric::topCandidateCount; ++i)
    {
        auto& rowLabel = candidateRowLabels[static_cast<std::size_t> (i)];
        auto& useButton = useCandidateButtons[static_cast<std::size_t> (i)];
        auto& saveButton = saveCandidateButtons[static_cast<std::size_t> (i)];

        const bool haveRow = i < candidateCount;
        rowLabel.setVisible (haveRow);
        useButton.setVisible (haveRow);
        saveButton.setVisible (haveRow);
        if (! haveRow)
            continue;

        const auto& c = snap.topCandidates[static_cast<std::size_t> (i)];
        const auto temporal = bbk::parametric::computeTemporalMetrics (c.taps, snap.sampleRateHz);
        const bool active = (i == snap.selectedIndex);

        juce::String rowText;
        rowText << "#" << (i + 1) << (active ? " (ACTIVE) " : "  ")
                << c.tapCount << " taps | R_peak " << juce::String (temporal.rPeakPercent, 2)
                << "% | T_0.1% " << juce::String (temporal.settlingMs, 3) << " ms | stopband "
                << juce::String (c.achievedStopbandDb, 1) << " dB";
        rowLabel.setText (rowText, juce::dontSendNotification);
        rowLabel.setColour (juce::Label::textColourId, active ? juce::Colour (0xffd9a34a) : juce::Colours::white);

        // Using the already-active row is a harmless no-op (selectTopCandidate
        // just re-publishes the same taps again), so there's no need to
        // disable it - simpler than special-casing "the one row that
        // matches snap.selectedIndex".
        useButton.setEnabled (true);
        saveButton.setEnabled (true);
    }

    if (coefficientsVisible)
        refreshCoefficientsText (snap);
}
