#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include "PluginProcessor.h"
#include "UserPresetOverrides.h" // for bbk::detachedpole::useroverrides::numPresetSlots

class BBKDetachedPoleAudioProcessorEditor final : public juce::AudioProcessorEditor,
                                                   private juce::Timer
{
public:
    explicit BBKDetachedPoleAudioProcessorEditor (BBKDetachedPoleAudioProcessor&);
    ~BBKDetachedPoleAudioProcessorEditor() override = default;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    void timerCallback() override;
    void toggleCoefficientsPopup();
    void refreshCoefficientsText (const BBKDetachedPoleAudioProcessor::DesignSnapshot&);

    // Lays out every child within content (see its own comment) exactly
    // once, right after construction - content's own size never changes
    // afterward (only how much of it the viewport shows/scrolls), so unlike
    // a normal resized() override this only ever needs to run a single
    // time, not on every editor resize.
    void layOutContent();

    BBKDetachedPoleAudioProcessor& processor;

    // The window itself (see setSize() in the .cpp) is deliberately kept to
    // a modest, screen-friendly default height - reported directly that the
    // previous fixed-height window (grown repeatedly across several feature
    // additions to fit everything with no scrolling at all) had become tall
    // enough to open partly off-screen on plenty of real monitors. Every
    // control below is instead a child of `content`, a plain Component sized
    // to whatever the FULL layout actually needs (see layOutContent()),
    // wrapped in `viewport` so the window can stay a reasonable, resizable
    // size while the full content just scrolls - and so adding yet more
    // controls in the future grows the scrollable area instead of the
    // window's forced minimum size again.
    juce::Viewport viewport;
    juce::Component content;

    juce::Label title;
    juce::Label subtitle;
    juce::Label sampleRate;

    // Lit whenever processor.isSearchInProgressForUI() is true - see its
    // own comment in PluginProcessor.h. Updated every timerCallback() tick,
    // same as clipIndicator below; blank/dim the rest of the time so it
    // doesn't clutter the top row when nothing is actually being computed.
    juce::Label searchIndicator;

    juce::ToggleButton bypassButton { "Bypass" };
    juce::ToggleButton amplitudeRelaxationButton { "Amplitude Relaxation" };

    juce::Label cutoffLabel;
    juce::Slider cutoffSlider;
    juce::Label attenuationLabel;
    juce::Slider attenuationSlider;
    juce::Label stopbandLabel;
    juce::Slider stopbandSlider;
    juce::Label sidelobeDecayLabel;
    juce::Slider sidelobeDecaySlider;

    // TDR-constrained optimization (see ParametricFIR.h::OptimizationMode/
    // FilterSpec's own comments). tdrConstraintButton: off (default) is
    // the existing Rpeak-only optimization, unchanged; on additionally
    // requires the selected decay threshold to be reached by
    // maxDecayTimeSlider below. decayThresholdSlider stays live (never
    // greyed out) regardless of the toggle - it also controls what
    // "Actual Tdecay" in the metrics readout is measured against for
    // display, per direct request that both modes report the same
    // figures on the same footing; decayThresholdDbLabel shows the live
    // dB equivalent right next to it (TDR_dB = -20*log10(pct/100)),
    // refreshed every timerCallback() tick, per direct request to display
    // both units together. maxDecayTimeSlider only matters while the
    // toggle is on, so it greys out the same way manualTapCountSlider
    // does for Auto mode.
    juce::ToggleButton tdrConstraintButton { "TDR Constraint" };
    juce::Label decayThresholdLabel;
    juce::Slider decayThresholdSlider;
    juce::Label decayThresholdDbLabel;
    juce::Label maxDecayTimeLabel;
    juce::Slider maxDecayTimeSlider;

    // Manual/Auto tap-count selector - see "tapCountAuto"/"manualTapCount"
    // in PluginProcessor.cpp::createParameterLayout(). manualTapCountSlider
    // is greyed out (see timerCallback()) whenever tapCountAutoButton is
    // checked - Auto mode ignores it entirely.
    juce::Label manualTapCountLabel;
    juce::Slider manualTapCountSlider;
    juce::ToggleButton tapCountAutoButton { "Auto" };

    // Custom-mode search safety net (see "maxSearchTimeValue"/
    // "maxSearchTimeIsHours" in PluginProcessor.cpp::createParameterLayout()
    // and run()'s own comment): a typable numeric value (styled like
    // headroomSlider below - click the number to edit directly, or use the
    // +/- arrows) plus an Hours toggle, defaulting to "5" / minutes. This
    // bounds how long an unattended search can run - the search itself no
    // longer has any patience limit of its own and otherwise keeps
    // searching toward the tap-count ceiling until it's stopped, either by
    // this deadline or by stopSearchButton below.
    juce::Label maxSearchTimeLabel;
    juce::Slider maxSearchTimeSlider;
    juce::ToggleButton maxSearchTimeHoursButton { "Hours" };

    // Per-tap-count-candidate search deadline (see
    // "perCandidateSearchTimeSeconds" in
    // PluginProcessor.cpp::createParameterLayout() and run()'s own comment)
    // - distinct from Max Search Time above, which bounds the WHOLE sweep
    // across every tap count tried: this instead bounds how long a SINGLE
    // candidate's own grid-refinement loop may run before being cut off and
    // the search moves on to the next tap count. That loop's own stopping
    // condition is wall-clock time, not a fixed round count, so this is
    // what actually determines how well-converged (and therefore how good)
    // a demanding candidate's result is once the search settles on it -
    // raising it trades covering fewer distinct tap counts (within the same
    // overall Max Search Time budget) for more room to fully converge on
    // each one it does try.
    juce::Label perCandidateSearchTimeLabel;
    juce::Slider perCandidateSearchTimeSlider;

    // Calls processor.requestStopSearch() - immediately loads whatever the
    // best result found so far is, exactly as if the search had hit its own
    // deadline (see PluginProcessor.h). Enabled only while
    // processor.isSearchInProgressForUI() is true - see timerCallback().
    juce::TextButton stopSearchButton { "Stop" };

    juce::Label headroomCaption;
    juce::Slider headroomSlider;
    juce::ToggleButton autoHeadroomButton { "Auto" };
    juce::Label clipIndicator; // lit red for a short hold time after the soft-clip backstop engages

    // Visual companion to searchProgressText's own numbers (see
    // timerCallback()) - a horizontal bar spanning the search's own
    // tap-count range (bbk::parametric::autoSearchStartTapCount up to
    // bbk::detachedpole::maxTapCount, i.e. 19-161 for Auto mode) with the
    // fill showing how far the candidate JUST evaluated
    // (SearchProgressSnapshot::currentTapCount) has climbed through that
    // range so far - not a generic "some % done" bar, since this search has
    // no fixed number of steps to divide by (it can stop the moment a
    // compliant M is found, or a deadline/Stop cuts it short) and a plain
    // percentage would be meaningless. searchProgressFraction is the
    // juce::ProgressBar's own required backing value - must be declared
    // BEFORE searchProgressBar below, since the bar holds a reference to it
    // that must already exist when the bar is constructed. Visible only
    // while a search is actually running (see timerCallback(), same
    // condition as searchIndicator/stopSearchButton).
    double searchProgressFraction = 0.0;
    juce::ProgressBar searchProgressBar { searchProgressFraction };

    juce::Label metricsReadout;
    juce::TextButton coefficientsButton { "Show Coefficients" };
    juce::TextEditor coefficientsBox;
    bool coefficientsVisible = false;

    // Top-N ranked-results table (see BBKDetachedPoleAudioProcessor::
    // DesignSnapshot::topCandidates / bbk::parametric::topCandidateCount):
    // one row per candidate the last completed Auto-mode search found,
    // best R_peak first, each with its own metrics plus a Use (switch to
    // it, no re-search - see selectTopCandidate()) and Save (persist it as
    // a plain, exact-spec-recall override - see saveTopCandidateAsOverride())
    // button. Populated and shown/hidden per row in timerCallback() based
    // on how many candidates the current design snapshot actually has -
    // Manual-mode results and preset/bank entries have none, so every row
    // stays blank/hidden for those, exactly as if this table weren't there
    // at all.
    juce::Label topCandidatesHeader;
    std::array<juce::Label, static_cast<std::size_t> (bbk::parametric::topCandidateCount)> candidateRowLabels;
    std::array<juce::TextButton, static_cast<std::size_t> (bbk::parametric::topCandidateCount)> useCandidateButtons;
    std::array<juce::TextButton, static_cast<std::size_t> (bbk::parametric::topCandidateCount)> saveCandidateButtons;

    // Forces a genuinely fresh live search for the current spec even if a
    // saved override or a search-cache entry already exists for it - see
    // BBKDetachedPoleAudioProcessor::requestFreshSearch()'s own comment.
    // Answers "can I re-run a search for parameters I already have saved
    // results for" directly: yes, on demand, via this button.
    juce::TextButton reSearchButton { "Re-search" };

    // Numbered preset slots - what replaced the plugin's old single
    // "Default" toggle (see BBKDetachedPoleAudioProcessor::loadPresetSlot()/
    // savePresetSlot()'s own comments for the full rationale). Each slot's
    // Label shows a short auto-generated summary of whatever currently
    // occupies it ("P1: 18500 Hz / 0.5000 dB" or similar), refreshed every
    // timerCallback() tick same as everything else here; Load recalls that
    // exact operating point outright, Save bookmarks whatever is CURRENTLY
    // PLAYING into that slot, unconditionally overwriting whatever was
    // there. Directly answers "I don't remember which of the near-infinite
    // parameter combinations gave me a result I liked" - bookmark it the
    // moment you hear it, without needing to reconstruct or even remember
    // the search that produced it.
    juce::Label presetSlotHeader;
    std::array<juce::Label, static_cast<std::size_t> (bbk::detachedpole::useroverrides::numPresetSlots)> presetSlotLabels;
    std::array<juce::TextButton, static_cast<std::size_t> (bbk::detachedpole::useroverrides::numPresetSlots)> presetLoadButtons;
    std::array<juce::TextButton, static_cast<std::size_t> (bbk::detachedpole::useroverrides::numPresetSlots)> presetSaveButtons;

    // Whole-bank sharing: writes every occupied preset slot to a single
    // file (Export), or reads one back and merges it slot-by-slot into
    // this install's own bank (Import) - see
    // BBKDetachedPoleAudioProcessor::exportPresets()/importPresets()'s own
    // comments. That file is just an ordinary XML file - email it, message
    // it, drop it in a shared folder, however you'd share any other file -
    // so a good find can travel to someone else's install of this plugin
    // without them needing to reconstruct the search that produced it
    // either.
    juce::TextButton exportPresetsButton { "Export Presets..." };
    juce::TextButton importPresetsButton { "Import Presets..." };

    // Must outlive the async FileChooser callback (see its own use in the
    // .cpp) - a local variable going out of scope before the user finishes
    // interacting with the native file dialog would leave the callback
    // pointing at a destroyed object.
    std::unique_ptr<juce::FileChooser> activeFileChooser;

    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> cutoffAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> attenuationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> stopbandAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> sidelobeDecayAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> tdrConstraintAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> decayThresholdAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> maxDecayTimeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> manualTapCountAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> tapCountAutoAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> maxSearchTimeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> maxSearchTimeHoursAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> perCandidateSearchTimeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> bypassAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> amplitudeRelaxationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> headroomAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> autoHeadroomAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessorEditor)
};
