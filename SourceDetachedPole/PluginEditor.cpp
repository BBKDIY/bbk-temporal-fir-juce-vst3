#include "PluginEditor.h"

// For the live Decay Threshold (%) -> dB conversion in timerCallback()
// below (std::log10/std::max) - not strictly needed given ParametricFIR.h
// already pulls these in transitively, but included directly rather than
// relying on that.
#include <algorithm>
#include <cmath>

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

// Converts a percentage ratio (a Decay Threshold, or an R_peak reading)
// into its dB equivalent - the same -20*log10(pct/100) formula used
// throughout ParametricFIR.h::TemporalMetrics::tdr0dB, just shared here
// for the handful of additional dB labels below (per-candidate/per-preset
// row text, live interim search progress) that don't go through a
// TemporalMetrics struct at all (a threshold setting) or only have the
// raw percent, not a precomputed dB field (SearchProgressSnapshot's
// bestRPeakPercent). Clamped the same way tdr0dB's own computation is, so
// a degenerate 0% input reads as a very large (not infinite/NaN) dB
// figure rather than breaking the display.
double percentRatioToDb (double percent)
{
    return -20.0 * std::log10 (std::max (percent / 100.0, 1.0e-300));
}
}

BBKDetachedPoleAudioProcessorEditor::BBKDetachedPoleAudioProcessorEditor (BBKDetachedPoleAudioProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    // See the member comments on viewport/content in PluginEditor.h: every
    // control below is added to `content`, not to this editor directly, so
    // the actual plugin WINDOW (sized well below content's own height - see
    // setSize() at the end of this constructor) can stay a reasonable,
    // resizable size while content just scrolls under it. false: content is
    // a plain member (not a heap object the viewport should own/delete).
    addAndMakeVisible (viewport);
    viewport.setViewedComponent (&content, false);
    viewport.setScrollBarsShown (true, false); // vertical only - content's width always matches the viewport's own

    prepareLabel (title, 22.0f, true);
    title.setText ("BBK Parametric FIR", juce::dontSendNotification);
    content.addAndMakeVisible (title);

    prepareLabel (subtitle, 12.0f);
    subtitle.setText ("Parametric constrained-least-squares FIR lowpass - auto-detects sample rate "
                       "(44.1/48/88.2/96/176.4/192/384 kHz, or any other rate the host reports)",
                       juce::dontSendNotification);
    content.addAndMakeVisible (subtitle);

    prepareLabel (sampleRate);
    content.addAndMakeVisible (sampleRate);

    // Lit whenever a background design is actually running (see
    // processor.isSearchInProgressForUI()'s own comment) - text/colour set
    // live in timerCallback(), same polling pattern as clipIndicator below.
    searchIndicator.setJustificationType (juce::Justification::centred);
    searchIndicator.setColour (juce::Label::textColourId, juce::Colours::white);
    searchIndicator.setFont (juce::Font (12.0f, juce::Font::bold));
    content.addAndMakeVisible (searchIndicator);

    bypassButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    content.addAndMakeVisible (bypassButton);
    bypassAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "bypass", bypassButton);

    prepareLabel (cutoffLabel, 13.0f, false, juce::Justification::centredLeft);
    cutoffLabel.setText ("Cutoff", juce::dontSendNotification);
    content.addAndMakeVisible (cutoffLabel);
    prepareSlider (cutoffSlider);
    content.addAndMakeVisible (cutoffSlider);
    cutoffAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "cutoff", cutoffSlider);

    prepareLabel (attenuationLabel, 13.0f, false, juce::Justification::centredLeft);
    attenuationLabel.setText ("Attenuation at Cutoff", juce::dontSendNotification);
    content.addAndMakeVisible (attenuationLabel);
    prepareSlider (attenuationSlider);
    // 4 decimal places (not the default 2) and a wider text box: the
    // parameter's own step is now 0.0001 dB (see
    // PluginProcessor.cpp::createParameterLayout()) specifically so
    // research operating points like 0.0027 dB are reachable - a 2-decimal
    // display would visually round that right back to "0.00" even though
    // the stored value is exact, which is confusing to type against.
    attenuationSlider.setNumDecimalPlacesToDisplay (4);
    attenuationSlider.setTextBoxStyle (juce::Slider::TextBoxRight, false, 110, 22);
    content.addAndMakeVisible (attenuationSlider);
    attenuationAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "attenuation", attenuationSlider);

    // On: the slider above is used as typed (spectrally relaxed, Case
    // C-style). Off: the slider is ignored entirely and the exact
    // calibrated near-flat Case B point is used instead - deterministic,
    // not dependent on typing or host rounding at 0.0001 dB precision
    // (see DetachedPoleFilter.h::caseBNearFlatAttenuationDb).
    amplitudeRelaxationButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    content.addAndMakeVisible (amplitudeRelaxationButton);
    amplitudeRelaxationAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "amplitudeRelaxation", amplitudeRelaxationButton);

    prepareLabel (stopbandLabel, 13.0f, false, juce::Justification::centredLeft);
    stopbandLabel.setText ("Min. Stopband Rejection", juce::dontSendNotification);
    content.addAndMakeVisible (stopbandLabel);
    prepareSlider (stopbandSlider);
    content.addAndMakeVisible (stopbandSlider);
    stopbandAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "stopband", stopbandSlider);

    prepareLabel (sidelobeDecayLabel, 13.0f, false, juce::Justification::centredLeft);
    sidelobeDecayLabel.setText ("Sidelobe Decay", juce::dontSendNotification);
    content.addAndMakeVisible (sidelobeDecayLabel);
    prepareSlider (sidelobeDecaySlider);
    sidelobeDecaySlider.setNumDecimalPlacesToDisplay (3);
    content.addAndMakeVisible (sidelobeDecaySlider);
    sidelobeDecayAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "sidelobeDecay", sidelobeDecaySlider);

    // TDR-constrained optimization - see the member comments in
    // PluginEditor.h and ParametricFIR.h::OptimizationMode/FilterSpec's
    // own comments. Off by default: the existing Rpeak-only optimization,
    // completely unchanged.
    tdrConstraintButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    content.addAndMakeVisible (tdrConstraintButton);
    tdrConstraintAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "tdrConstraintOn", tdrConstraintButton);

    prepareLabel (decayThresholdLabel, 13.0f, false, juce::Justification::centredLeft);
    decayThresholdLabel.setText ("Decay Threshold", juce::dontSendNotification);
    content.addAndMakeVisible (decayThresholdLabel);
    prepareSlider (decayThresholdSlider);
    decayThresholdSlider.setNumDecimalPlacesToDisplay (2);
    content.addAndMakeVisible (decayThresholdSlider);
    decayThresholdAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "decayThreshold", decayThresholdSlider);

    // Live dB equivalent right next to the percent slider (TDR_dB =
    // -20*log10(pct/100)) - refreshed every timerCallback() tick, per
    // direct request to display both units together rather than only one.
    prepareLabel (decayThresholdDbLabel, 12.0f, false, juce::Justification::centredLeft);
    content.addAndMakeVisible (decayThresholdDbLabel);

    prepareLabel (maxDecayTimeLabel, 13.0f, false, juce::Justification::centredLeft);
    maxDecayTimeLabel.setText ("Maximum Decay Time (us)", juce::dontSendNotification);
    content.addAndMakeVisible (maxDecayTimeLabel);
    prepareSlider (maxDecayTimeSlider);
    maxDecayTimeSlider.setNumDecimalPlacesToDisplay (0);
    content.addAndMakeVisible (maxDecayTimeSlider);
    maxDecayTimeAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "maxDecayTimeUs", maxDecayTimeSlider);

    // Manual/Auto tap-count selector - greying handled in timerCallback()
    // (needs to track the parameter live, e.g. host automation of
    // "tapCountAuto").
    prepareLabel (manualTapCountLabel, 13.0f, false, juce::Justification::centredLeft);
    manualTapCountLabel.setText ("Manual Tap Count", juce::dontSendNotification);
    content.addAndMakeVisible (manualTapCountLabel);
    prepareSlider (manualTapCountSlider);
    manualTapCountSlider.setNumDecimalPlacesToDisplay (0); // always a whole (odd) tap count - see the parameter's own step
    content.addAndMakeVisible (manualTapCountSlider);
    manualTapCountAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "manualTapCount", manualTapCountSlider);

    tapCountAutoButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    content.addAndMakeVisible (tapCountAutoButton);
    tapCountAutoAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "tapCountAuto", tapCountAutoButton);

    // Custom-mode search safety net - see the member comment in
    // PluginEditor.h. Same typeable IncDecButtons style as headroomSlider
    // below (click the number to edit directly, or use the +/- arrows).
    prepareLabel (maxSearchTimeLabel, 13.0f, false, juce::Justification::centredLeft);
    maxSearchTimeLabel.setText ("Max Search Time", juce::dontSendNotification);
    content.addAndMakeVisible (maxSearchTimeLabel);
    maxSearchTimeSlider.setSliderStyle (juce::Slider::IncDecButtons);
    maxSearchTimeSlider.setTextBoxStyle (juce::Slider::TextBoxLeft, false, 60, 22);
    maxSearchTimeSlider.setIncDecButtonsMode (juce::Slider::incDecButtonsDraggable_Vertical);
    maxSearchTimeSlider.setNumDecimalPlacesToDisplay (1);
    content.addAndMakeVisible (maxSearchTimeSlider);
    maxSearchTimeAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "maxSearchTimeValue", maxSearchTimeSlider);

    maxSearchTimeHoursButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    content.addAndMakeVisible (maxSearchTimeHoursButton);
    maxSearchTimeHoursAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment> (
        processor.getAPVTS(), "maxSearchTimeIsHours", maxSearchTimeHoursButton);

    // Per-candidate search deadline - see the member comment in
    // PluginEditor.h and "perCandidateSearchTimeSeconds" in
    // PluginProcessor.cpp::createParameterLayout(). Same typeable
    // IncDecButtons style as Max Search Time above.
    prepareLabel (perCandidateSearchTimeLabel, 13.0f, false, juce::Justification::centredLeft);
    perCandidateSearchTimeLabel.setText ("Per-Candidate Time (s)", juce::dontSendNotification);
    content.addAndMakeVisible (perCandidateSearchTimeLabel);
    perCandidateSearchTimeSlider.setSliderStyle (juce::Slider::IncDecButtons);
    perCandidateSearchTimeSlider.setTextBoxStyle (juce::Slider::TextBoxLeft, false, 60, 22);
    perCandidateSearchTimeSlider.setIncDecButtonsMode (juce::Slider::incDecButtonsDraggable_Vertical);
    perCandidateSearchTimeSlider.setNumDecimalPlacesToDisplay (0);
    content.addAndMakeVisible (perCandidateSearchTimeSlider);
    perCandidateSearchTimeAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment> (
        processor.getAPVTS(), "perCandidateSearchTimeSeconds", perCandidateSearchTimeSlider);

    // Loads whatever the best result found so far is, right away - see
    // processor.requestStopSearch()'s own comment. Enabled state tracks
    // processor.isSearchInProgressForUI() live in timerCallback(), same
    // pattern as the greyed-out sliders above.
    stopSearchButton.onClick = [this] { processor.requestStopSearch(); };
    content.addAndMakeVisible (stopSearchButton);

    prepareLabel (headroomCaption, 13.0f, false, juce::Justification::centredLeft);
    headroomCaption.setText ("Headroom (dB)", juce::dontSendNotification);
    content.addAndMakeVisible (headroomCaption);

    // IncDecButtons: a typeable numeric box (click the number to edit
    // directly, or use the +/- arrows), not a drag knob.
    headroomSlider.setSliderStyle (juce::Slider::IncDecButtons);
    headroomSlider.setTextBoxStyle (juce::Slider::TextBoxLeft, false, 60, 22);
    headroomSlider.setIncDecButtonsMode (juce::Slider::incDecButtonsDraggable_Vertical);
    content.addAndMakeVisible (headroomSlider);
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
    content.addAndMakeVisible (autoHeadroomButton);
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
    content.addAndMakeVisible (clipIndicator);

    // Percentage display off - a bare "%" would need the search's own
    // finish condition to be a fixed step count, which it isn't (see this
    // member's own comment in PluginEditor.h); setTextToDisplay() in
    // timerCallback() supplies the real "N taps (searching 19-161)" text
    // instead. Starts hidden - only shown while a search is actually
    // running, same as searchIndicator/stopSearchButton (see timerCallback()).
    searchProgressBar.setPercentageDisplay (false);
    searchProgressBar.setVisible (false);
    content.addAndMakeVisible (searchProgressBar);

    // topLeft, not centredLeft: a Label vertically CENTRES its text within
    // its own bounds, so when the (wrapped, multi-line) metrics text is
    // taller than the fixed area below it gives it, centring clips BOTH
    // the top and bottom lines symmetrically - reported directly as "the
    // central part with the text is not big enough to show full text".
    // topLeft means any remaining shortfall only ever clips the bottom,
    // which is far less confusing, and pairs with generously sizing that
    // area in resized() below so clipping shouldn't normally happen at all.
    prepareLabel (metricsReadout, 13.0f, false, juce::Justification::topLeft);
    content.addAndMakeVisible (metricsReadout);

    coefficientsButton.onClick = [this] { toggleCoefficientsPopup(); };
    content.addAndMakeVisible (coefficientsButton);

    // Forces a genuinely fresh search regardless of any existing cache/
    // override entry for the current spec - see requestFreshSearch()'s own
    // comment.
    reSearchButton.onClick = [this] { processor.requestFreshSearch(); };
    content.addAndMakeVisible (reSearchButton);

    prepareLabel (topCandidatesHeader, 13.0f, true, juce::Justification::centredLeft);
    topCandidatesHeader.setText ("Top Results (best R_peak first) - Custom/Auto only", juce::dontSendNotification);
    content.addAndMakeVisible (topCandidatesHeader);

    // One row per possible ranked candidate (see DesignSnapshot::
    // topCandidates) - text and visibility for each are filled in per-tick
    // by timerCallback() based on how many candidates the current snapshot
    // actually has; a row with nothing to show is simply hidden rather than
    // left blank, so Manual-mode/preset-bank results (which have none)
    // don't leave 5 empty rows sitting on screen.
    for (int i = 0; i < bbk::parametric::topCandidateCount; ++i)
    {
        auto& rowLabel = candidateRowLabels[static_cast<std::size_t> (i)];
        prepareLabel (rowLabel, 12.0f, false, juce::Justification::centredLeft);
        content.addAndMakeVisible (rowLabel);

        auto& useButton = useCandidateButtons[static_cast<std::size_t> (i)];
        useButton.setButtonText ("Use");
        useButton.onClick = [this, i] { processor.selectTopCandidate (i); };
        content.addAndMakeVisible (useButton);

        // Persists this row's candidate for instant exact-spec recall
        // later, same as any other override - see
        // saveTopCandidateAsOverride()'s own comment. If you'd rather
        // bookmark it into one of the numbered preset slots below instead
        // (so you can get back to it without needing to recreate this
        // exact spec), use Save on one of the Preset rows once this row is
        // active (Use it first, or it already is the active one).
        auto& saveButton = saveCandidateButtons[static_cast<std::size_t> (i)];
        saveButton.setButtonText ("Save");
        saveButton.onClick = [this, i] { processor.saveTopCandidateAsOverride (i); };
        content.addAndMakeVisible (saveButton);
    }

    // Numbered preset slots - see the member comment in PluginEditor.h and
    // BBKDetachedPoleAudioProcessor::loadPresetSlot()/savePresetSlot()'s own
    // comments. Label text is filled in per-tick by timerCallback() (it
    // needs to reflect whatever's actually saved on disk, which can change
    // from any of these five Save buttons, so it's refreshed the same way
    // the top-N table's own row text is).
    prepareLabel (presetSlotHeader, 13.0f, true, juce::Justification::centredLeft);
    presetSlotHeader.setText ("Presets - bookmark a good find, recall it without redoing the search", juce::dontSendNotification);
    content.addAndMakeVisible (presetSlotHeader);

    for (int i = 0; i < bbk::detachedpole::useroverrides::numPresetSlots; ++i)
    {
        auto& slotLabel = presetSlotLabels[static_cast<std::size_t> (i)];
        prepareLabel (slotLabel, 12.0f, false, juce::Justification::centredLeft);
        content.addAndMakeVisible (slotLabel);

        auto& loadButton = presetLoadButtons[static_cast<std::size_t> (i)];
        loadButton.setButtonText ("Load");
        loadButton.onClick = [this, i] { processor.loadPresetSlot (i); };
        content.addAndMakeVisible (loadButton);

        auto& saveButton = presetSaveButtons[static_cast<std::size_t> (i)];
        saveButton.setButtonText ("Save");
        saveButton.onClick = [this, i] { processor.savePresetSlot (i); };
        content.addAndMakeVisible (saveButton);
    }

    // Whole-bank sharing - see the member comment in PluginEditor.h.
    // launchAsync (never a blocking/modal file dialog) so the native file
    // picker never stalls the message thread; activeFileChooser is kept
    // alive as a member for the async callback's duration (see its own
    // comment) and reset once the callback runs, whether or not the user
    // actually picked a file.
    exportPresetsButton.onClick = [this]
    {
        activeFileChooser = std::make_unique<juce::FileChooser> (
            "Export presets to...", juce::File(), "*.xml");
        activeFileChooser->launchAsync (juce::FileBrowserComponent::saveMode
                                         | juce::FileBrowserComponent::warnAboutOverwriting,
            [this] (const juce::FileChooser& fc)
            {
                auto file = fc.getResult();
                if (file != juce::File())
                    processor.exportPresets (file);
                activeFileChooser.reset();
            });
    };
    content.addAndMakeVisible (exportPresetsButton);

    importPresetsButton.onClick = [this]
    {
        activeFileChooser = std::make_unique<juce::FileChooser> (
            "Import presets from...", juce::File(), "*.xml");
        activeFileChooser->launchAsync (juce::FileBrowserComponent::openMode,
            [this] (const juce::FileChooser& fc)
            {
                auto file = fc.getResult();
                if (file != juce::File())
                    processor.importPresets (file);
                activeFileChooser.reset();
            });
    };
    content.addAndMakeVisible (importPresetsButton);

    coefficientsBox.setMultiLine (true);
    coefficientsBox.setReadOnly (true);
    coefficientsBox.setScrollbarsShown (true);
    coefficientsBox.setCaretVisible (false);
    coefficientsBox.setFont (juce::Font (juce::Font::getDefaultMonospacedFontName(), 12.0f, juce::Font::plain));
    coefficientsBox.setColour (juce::TextEditor::backgroundColourId, juce::Colour (0xff0d0d0d));
    coefficientsBox.setColour (juce::TextEditor::textColourId, juce::Colours::lightgreen);
    coefficientsBox.setVisible (false);
    content.addChildComponent (coefficientsBox);

    // contentHeight is the sum of every fixed row/gap laid out in
    // layOutContent() below (currently ~1468, including the 20px top/bottom
    // margins baked into that method's own area.reduced(20), the 5
    // preset-slot rows - each 40px tall rather than a plain single-line 24px
    // row, since an occupied slot's label now shows a second line of the
    // FIR's own metrics below its spec summary, see timerCallback() - the
    // Export/Import row added below the top-N table, the 30px
    // searchProgressBar row above metricsReadout (see its own comment in
    // PluginEditor.h), the three TDR-constrained-optimization control rows
    // (tdrConstraintButton/decayThresholdSlider/maxDecayTimeSlider, ~94px
    // together - see their own comments in PluginEditor.h), and
    // metricsReadout's own 380->460px bump for the extra TDR reporting
    // lines) plus a fixed allowance for the coefficients box - it has its
    // own internal scrollbar (see setScrollbarsShown() above), so it
    // doesn't need much outer space to still be fully usable. This is
    // content's own, possibly-tall size; it is NOT the window size - see
    // viewport's own comment in PluginEditor.h and setSize() just below for
    // why those are now deliberately different.
    constexpr int contentWidth = 680;
    constexpr int coefficientsBoxHeight = 260;
    constexpr int contentHeight = 1468 + 40 + coefficientsBoxHeight;
    content.setSize (contentWidth, contentHeight);
    layOutContent();

    // The window itself, unlike content above, is kept to a modest,
    // screen-friendly default - reported directly that the previous fixed-
    // height window (grown repeatedly, in step with content, to show
    // everything with no scrolling at all) had become tall enough to open
    // partly off-screen on plenty of real monitors. 700 wide leaves slack
    // beyond content's own 680 for the vertical scrollbar; 820 tall is a
    // conservative default that should comfortably fit even a modest
    // laptop screen alongside the host's own window chrome. Resizable
    // (both directions - a wider window doesn't need scrolling, just shows
    // more blank margin, which is harmless) so anyone with more screen room
    // can drag it taller and see more of content at once, up to content's
    // own full size, beyond which there is nothing more to reveal.
    setResizable (true, true);
    setResizeLimits (420, 400, contentWidth + 40, contentHeight);
    setSize (700, 820);

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
    // content's own size is fixed once, in the constructor (see
    // layOutContent()'s own call site there) - all this needs to do on
    // every actual window resize is let the viewport fill whatever space
    // the window now has; it handles showing/hiding its own scrollbar and
    // scrolling content within that space entirely on its own.
    viewport.setBounds (getLocalBounds());
}

void BBKDetachedPoleAudioProcessorEditor::layOutContent()
{
    auto area = content.getLocalBounds().reduced (20);

    title.setBounds (area.removeFromTop (30));
    subtitle.setBounds (area.removeFromTop (18));
    area.removeFromTop (6);

    {
        auto row = area.removeFromTop (24);
        bypassButton.setBounds (row.removeFromRight (100));
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

    tdrConstraintButton.setBounds (area.removeFromTop (24));
    area.removeFromTop (6);

    {
        // decayThresholdDbLabel sits right of the slider's own text box,
        // in the same row - see its own comment in PluginEditor.h for why
        // this shows both units side by side rather than one or the other.
        auto row = area.removeFromTop (26);
        decayThresholdLabel.setBounds (row.removeFromLeft (170));
        decayThresholdSlider.setBounds (row.removeFromLeft (200));
        row.removeFromLeft (10);
        decayThresholdDbLabel.setBounds (row);
    }
    area.removeFromTop (6);

    {
        auto row = area.removeFromTop (26);
        maxDecayTimeLabel.setBounds (row.removeFromLeft (170));
        maxDecayTimeSlider.setBounds (row);
    }
    area.removeFromTop (6);

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
        perCandidateSearchTimeLabel.setBounds (row.removeFromLeft (170));
        perCandidateSearchTimeSlider.setBounds (row.removeFromLeft (100));
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
    searchProgressBar.setBounds (area.removeFromTop (22));
    area.removeFromTop (8);

    // Sized generously (up from 215, then 380) for the metrics text's
    // actual worst-case line count: the "Design method"/"Stopband mode"/
    // "Center-tap gain" lines alone wrap to 2-3 lines each at this width,
    // plus the live search-progress addendum (see timerCallback()) adds up
    // to 2 more while a search is running, plus the TDR-constrained
    // optimization block (TDR0/threshold/actual Tdecay/TDR30-60 ladder/
    // PASS-FAIL/main-lobe-intrusion warning - see timerCallback()) adds up
    // to 5 more - undersizing this clipped real content, reported directly
    // ("the central part with the text is not big enough to show full
    // text"). Content (not the window itself - see the constructor's own
    // contentHeight comment) is sized to fit this.
    metricsReadout.setBounds (area.removeFromTop (460));

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

    area.removeFromTop (10);
    presetSlotHeader.setBounds (area.removeFromTop (20));
    area.removeFromTop (4);
    for (int i = 0; i < bbk::detachedpole::useroverrides::numPresetSlots; ++i)
    {
        // Taller than a plain single-line row (see the equivalent top-N
        // table loop above) - an occupied slot's label now shows a second
        // line of the FIR's own metrics below its spec summary (see
        // timerCallback()), so it needs the extra height; Load/Save stay
        // their original 24px tall, top-aligned within the row, rather than
        // stretching to fill it.
        auto row = area.removeFromTop (40);
        auto& loadButton = presetLoadButtons[static_cast<std::size_t> (i)];
        auto& saveButton = presetSaveButtons[static_cast<std::size_t> (i)];
        loadButton.setBounds (row.removeFromRight (60).withHeight (24));
        row.removeFromRight (6);
        saveButton.setBounds (row.removeFromRight (60).withHeight (24));
        row.removeFromRight (10);
        presetSlotLabels[static_cast<std::size_t> (i)].setBounds (row);
        area.removeFromTop (4);
    }

    area.removeFromTop (6);
    {
        auto row = area.removeFromTop (26);
        exportPresetsButton.setBounds (row.removeFromLeft (160));
        row.removeFromLeft (10);
        importPresetsButton.setBounds (row.removeFromLeft (160));
    }

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
    // Visual companion to searchProgressText's own numbers below - see
    // searchProgressBar's own comment in PluginEditor.h for why this is a
    // range-position bar (19 up to bbk::detachedpole::maxTapCount, i.e. 161)
    // rather than a plain percentage. Hidden outside a search, same
    // condition as searchIndicator/stopSearchButton just above.
    searchProgressBar.setVisible (searching);
    if (! searching)
        searchProgressFraction = 0.0;

    juce::String searchProgressText;
    if (searching)
    {
        const auto progress = processor.getSearchProgressForUI();

        // TDR-constrained optimization - see the member comments in
        // PluginEditor.h. Once the TDR Constraint toggle is on, TDR0 (dB)
        // is the metric actually being optimised toward, so it leads the
        // interim best-so-far readout below too, not just the finished
        // design's own metrics text - per direct request, so a live
        // search's progress is legible in the same units as the result
        // it's converging toward. Read directly from the parameter (not
        // snap.optimizationMode) so this tracks the toggle live even
        // while the search itself is still running for whatever spec was
        // in flight when it started - same reasoning as
        // decayThresholdDbLabel's own live read further below. Left
        // exactly as before (R_peak% only) while the toggle is off, since
        // TDR0 isn't the quantity being searched for in that mode.
        const bool tdrModeActive = processor.getAPVTS().getRawParameterValue ("tdrConstraintOn")->load() > 0.5f;

        // currentTapCount is 0 before this search's very first onProgress
        // call has landed (see SearchProgressSnapshot's own comment) - clamp
        // rather than let that read as a negative fraction. Manual mode's
        // own climb (designParametricFIRFixedM) can likewise start BELOW
        // autoSearchStartTapCount if fewer taps were requested than Auto's
        // own starting point - clamped to 0 (an empty bar) rather than a
        // negative fraction in that case too; jlimit handles both.
        constexpr int rangeFloor = bbk::parametric::autoSearchStartTapCount;
        constexpr int rangeCeiling = bbk::detachedpole::maxTapCount;
        searchProgressFraction = juce::jlimit (0.0, 1.0,
            static_cast<double> (progress.currentTapCount - rangeFloor) / static_cast<double> (rangeCeiling - rangeFloor));

        searchProgressBar.setTextToDisplay (progress.currentTapCount > 0
            ? ("Trying " + juce::String (progress.currentTapCount) + " taps (search range "
               + juce::String (rangeFloor) + "-" + juce::String (rangeCeiling) + ")")
            : juce::String ("Starting search..."));

        searchProgressText << "\nSearching for a better result in the background ("
                            << progress.attemptsSoFar << " candidate(s) tried so far";
        if (progress.haveBest && progress.bestIsFeasible)
        {
            searchProgressText << ", best so far " << progress.bestTapCount << " taps, ";
            if (tdrModeActive)
                searchProgressText << "TDR0 " << juce::String (percentRatioToDb (progress.bestRPeakPercent), 2)
                                    << " dB (R_peak " << juce::String (progress.bestRPeakPercent, 2) << "%)";
            else
                searchProgressText << "R_peak " << juce::String (progress.bestRPeakPercent, 2) << "%";
            searchProgressText << ", achieved " << juce::String (progress.bestAchievedStopbandDb, 2) << " dB)";
        }
        else if (progress.haveBest)
        {
            // Not yet compliant at any tap count tried so far, but not
            // nothing either - show the closest attempt's own numbers
            // (R_peak/TDR0 here is informational only, not a compliance
            // claim) so a long climb through infeasible tap counts still
            // shows real, moving progress instead of going silent until
            // the first fully compliant candidate finally turns up.
            searchProgressText << ", closest so far (not yet compliant) " << progress.bestTapCount << " taps, ";
            if (tdrModeActive)
                searchProgressText << "TDR0 " << juce::String (percentRatioToDb (progress.bestRPeakPercent), 2)
                                    << " dB (R_peak " << juce::String (progress.bestRPeakPercent, 2) << "%)";
            else
                searchProgressText << "R_peak " << juce::String (progress.bestRPeakPercent, 2) << "%";
            searchProgressText << ", achieved " << juce::String (progress.bestAchievedStopbandDb, 2) << " dB)";
        }
        else
        {
            searchProgressText << ", no candidate yet)";
        }
        searchProgressText << " - click Stop to load it immediately, or let it keep looking.\n";
    }

    // Preset slot labels - refreshed unconditionally (unlike the metrics/
    // top-N table below) since they reflect whatever's saved on disk, not
    // the currently-playing design, and should show correctly even before
    // the very first design has ever completed.
    for (int i = 0; i < bbk::detachedpole::useroverrides::numPresetSlots; ++i)
    {
        const auto info = processor.getPresetSlotInfoForUI (i);
        juce::String labelText;
        labelText << "P" << (i + 1) << ": ";
        if (info.occupied)
        {
            labelText << juce::String (info.spec.cutoffHz, 0) << " Hz / "
                      << juce::String (info.spec.attenuationAtCutoffDb, 4) << " dB / "
                      << juce::String (info.spec.stopbandRejectionDb, 1) << " dB stopband";

            // The actual filter's own metrics, not just the spec it was
            // searched for - recomputed from taps on demand, same as the
            // top-N table's own rows (see the "Top Results" loop below) -
            // this is what the user asked for when they said the presets
            // showed only the parameters, not the FIR's own metrics.
            // Threshold now taken from this preset's own saved spec (not
            // the legacy fixed 0.1%), per direct request, so the settling
            // figure below reflects whatever Decay Threshold this preset
            // was actually saved under, comparable across slots at their
            // own respective thresholds.
            const auto temporal = bbk::parametric::computeTemporalMetrics (
                info.taps, info.spec.sampleRateHz, info.spec.tdrDecayThresholdPercent);

            // Leads with TDR0 (dB) instead of R_peak% only for a preset
            // saved under TDR-constrained optimization - see this same
            // branch in the "Top Results" loop below for the full
            // rationale. A plain Rpeak-only preset keeps R_peak% leading,
            // unchanged.
            const bool tdrMode = (info.spec.optimizationMode == bbk::parametric::OptimizationMode::RpeakWithTdrConstraint);
            labelText << "\n      " << info.tapCount << " taps | ";
            if (tdrMode)
                labelText << "TDR0 " << juce::String (temporal.tdr0dB, 2) << " dB";
            else
                labelText << "R_peak " << juce::String (temporal.rPeakPercent, 2) << "%";
            labelText << " | T(" << juce::String (percentRatioToDb (info.spec.tdrDecayThresholdPercent), 1)
                      << " dB) " << juce::String (temporal.tdecayUs, 2) << " us | stopband "
                      << juce::String (info.achievedStopbandDb, 1) << " dB";
        }
        else if (i == 0)
        {
            labelText << "(empty - Load falls back to the factory bank)";
        }
        else
        {
            labelText << "(empty)";
        }
        presetSlotLabels[static_cast<std::size_t> (i)].setText (labelText, juce::dontSendNotification);
    }

    if (snap.tapCount == 0)
    {
        // Shown at cold start and again on every sample-rate change - the
        // redesign for the new rate runs entirely in the background and
        // never blocks playback; audio passes through unfiltered (delay-
        // matched, no clicks) until it completes and crossfades in. A
        // spec that lands an instant hit (the factory bank, a saved
        // override/preset, or the search cache) resolves near-instantly;
        // otherwise the live search can now run indefinitely (see
        // requestBoundaryRedesign()'s own comment and the Max Search Time
        // safety net) rather than blocking playback.
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

    // Manual Tap Count is a no-op while Auto is on (the engine's own
    // M-search picks the tap count instead - see run() in
    // PluginProcessor.cpp) - grey it out so it never looks live and
    // editable when it wouldn't actually do anything. Loading a preset
    // slot (see loadPresetSlot()) forces Auto back on the same way the old
    // Default toggle used to, so this slider greys out then too, exactly
    // as it always did.
    const bool tapCountAutoOn = processor.getAPVTS().getRawParameterValue ("tapCountAuto")->load() > 0.5f;
    manualTapCountSlider.setEnabled (! tapCountAutoOn);

    // TDR-constrained optimization - see the member comments in
    // PluginEditor.h. maxDecayTimeSlider only matters while the toggle is
    // on, same greying convention as manualTapCountSlider above;
    // decayThresholdSlider stays enabled regardless (it also drives the
    // "Actual Tdecay" display below, in every mode).
    const bool tdrConstraintOn = processor.getAPVTS().getRawParameterValue ("tdrConstraintOn")->load() > 0.5f;
    maxDecayTimeSlider.setEnabled (tdrConstraintOn);

    // Live dB equivalent of the Decay Threshold (%) slider - TDR_dB =
    // -20*log10(pct/100) - per direct request to display both units
    // together. Read directly from the parameter (not snap.
    // tdrDecayThresholdPercent) so it tracks the slider live even while a
    // background search for a different value is still in flight, same
    // reasoning as sampleRate's own live read above.
    {
        const double decayThresholdPercent = static_cast<double> (
            processor.getAPVTS().getRawParameterValue ("decayThreshold")->load());
        const double decayThresholdDb = -20.0 * std::log10 (std::max (decayThresholdPercent / 100.0, 1.0e-300));
        decayThresholdDbLabel.setText ("(~" + juce::String (decayThresholdDb, 2) + " dB)", juce::dontSendNotification);
    }

    juce::String text;
    if (auto* bypassParam = processor.getAPVTS().getRawParameterValue ("bypass"))
        if (bypassParam->load() > 0.5f)
            text << "BYPASSED (dry signal, delay-matched - no filtering audible)\n";

    juce::String designMethodText;
    switch (snap.source)
    {
        using ResultSource = BBKDetachedPoleAudioProcessor::ResultSource;
        case ResultSource::PresetBank:
            designMethodText = "Factory bank - instant lookup in a precomputed filter (18.5 kHz cutoff, 95 dB "
                                "stopband, sidelobe decay 1.0, one of 10 attenuation steps chosen by the slider "
                                "above); no background search. This is what an unassigned Preset 1 falls back to.";
            break;
        case ResultSource::UserOverride:
            designMethodText = "User Override - instant lookup of a filter you saved yourself for this exact "
                                "operating point (see the Presets section, or a top-N row's own Save button); "
                                "no background search.";
            break;
        case ResultSource::SearchCache:
            designMethodText = "Already searched for this exact operating point earlier (this session "
                                "or a previous one); instant recall, no fresh search.";
            break;
        case ResultSource::LiveSearch:
        default:
            designMethodText = "Minimax, the article's own minimum-peak-sidelobe method, searched "
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

    // TDR-constrained optimization reporting (see ParametricFIR.h::
    // OptimizationMode/TemporalMetrics's own comments) - shown for every
    // finished design regardless of mode, per direct request, so the two
    // optimization philosophies can be compared on the same footing.
    // TDR0 is the TRUE unweighted transient dynamic range (from the
    // actual rPeakPercent above, not any internal weighted quantity - see
    // TemporalMetrics::tdr0dB's own comment); "Actual Tdecay" tracks
    // whatever Decay Threshold (%) is currently selected, while the
    // TDR30/40/50/60 ladder are always shown at their own fixed
    // thresholds for comparison.
    text << "Transient dynamic range: Rpeak " << juce::String (snap.temporal.rPeakPercent, 2)
         << "%  |  TDR0 " << juce::String (snap.temporal.tdr0dB, 2) << " dB\n"
         << "  Decay threshold " << juce::String (snap.tdrDecayThresholdPercent, 3) << "% (~"
         << juce::String (-20.0 * std::log10 (std::max (snap.tdrDecayThresholdPercent / 100.0, 1.0e-300)), 2)
         << " dB)  |  actual Tdecay " << snap.temporal.tdecaySamples << " samples ("
         << juce::String (snap.temporal.tdecayUs, 2) << " us)\n"
         << "  TDR30 " << juce::String (snap.temporal.tdr30Us, 2) << " us  |  TDR40 "
         << juce::String (snap.temporal.tdr40Us, 2) << " us  |  TDR50 "
         << juce::String (snap.temporal.tdr50Us, 2) << " us  |  TDR60 "
         << juce::String (snap.temporal.tdr60Us, 2) << " us\n";

    if (snap.optimizationMode == bbk::parametric::OptimizationMode::RpeakWithTdrConstraint)
    {
        // Pass/fail follows the design's own constraintsMet exactly (the
        // TDR constraint is a hard LP row baked into the same bisection
        // that produced this filter - see tryRho's own comment in
        // ParametricFIR.h - so a feasible result already provably
        // satisfies it; nothing separate to compute here).
        const bool tdrPass = snap.constraintsMet;
        text << "  Maximum allowed " << juce::String (snap.tdrMaxDecayTimeUs, 1) << " us - constraint "
             << (tdrPass ? "PASS" : "FAIL (best effort shown - see Status below)") << "\n";

        // Diagnostic only - see AttemptResult::tdrIntrudesMainLobe's own
        // comment. Never means the constraint was weakened, only that it
        // reached inside what would otherwise be the free main lobe.
        if (snap.tdrIntrudesMainLobe)
            text << "  Warning: Maximum Decay Time is short enough that the TDR constraint reaches "
                    "inside the filter's natural main lobe, not just the sidelobe region - decay is "
                    "still being enforced there too.\n";
    }

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

        // Threshold now taken from the active spec's own Decay Threshold
        // (not the legacy fixed 0.1%), per direct request, so all 5 rows
        // are directly comparable to each other and to the main metrics
        // readout above at the same threshold - see the preset-slot
        // loop's identical change above for the full rationale.
        const auto temporal = bbk::parametric::computeTemporalMetrics (
            c.taps, snap.sampleRateHz, snap.tdrDecayThresholdPercent);
        const bool active = (i == snap.selectedIndex);

        // Leads with TDR0 (dB) instead of R_peak% only once TDR-constrained
        // optimization is the active mode - per direct request ("when in
        // TDR search optimization... have this metric as leading, not
        // Rpeak"), since that's the quantity these 5 candidates were
        // actually ranked to maximise in that mode (R_peak ascending and
        // TDR0 descending are the same ordering - see this ranking's own
        // comment above - so nothing about WHICH 5 candidates appear or
        // their order changes here, only which number leads the text).
        // Plain Rpeak-only mode keeps R_peak% leading, unchanged.
        const bool tdrMode = (snap.optimizationMode == bbk::parametric::OptimizationMode::RpeakWithTdrConstraint);

        juce::String rowText;
        rowText << "#" << (i + 1) << (active ? " (ACTIVE) " : "  ") << c.tapCount << " taps | ";
        if (tdrMode)
            rowText << "TDR0 " << juce::String (temporal.tdr0dB, 2) << " dB";
        else
            rowText << "R_peak " << juce::String (temporal.rPeakPercent, 2) << "%";
        rowText << " | T(" << juce::String (percentRatioToDb (snap.tdrDecayThresholdPercent), 1)
                << " dB) " << juce::String (temporal.tdecayUs, 2) << " us | stopband "
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
