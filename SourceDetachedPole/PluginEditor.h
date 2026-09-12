#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include <array>
#include "PluginProcessor.h"

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

    // On: cutoff/stopband/sidelobeDecay are forced to the precomputed
    // preset-bank operating point (18.5 kHz / 95 dB / decay 1.0) and the
    // matching filter for the current sample rate + attenuation step loads
    // instantly, no background search. Those three sliders are greyed out
    // (still showing the forced values, per the chosen UI - see
    // timerCallback()) while this is on; Attenuation stays live since it's
    // what selects which of the 10 precomputed steps is used.
    juce::ToggleButton defaultModeButton { "Default (instant, prebuilt)" };

    juce::Label cutoffLabel;
    juce::Slider cutoffSlider;
    juce::Label attenuationLabel;
    juce::Slider attenuationSlider;
    juce::Label stopbandLabel;
    juce::Slider stopbandSlider;
    juce::Label sidelobeDecayLabel;
    juce::Slider sidelobeDecaySlider;

    // Manual/Auto tap-count selector - see "tapCountAuto"/"manualTapCount"
    // in PluginProcessor.cpp::createParameterLayout(). manualTapCountSlider
    // is greyed out (see timerCallback()) whenever tapCountAutoButton is
    // checked (Auto mode ignores it entirely) or Default mode is on (same
    // reasoning as cutoffSlider/stopbandSlider/sidelobeDecaySlider above).
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

    juce::Label metricsReadout;
    juce::TextButton coefficientsButton { "Show Coefficients" };
    juce::TextEditor coefficientsBox;
    bool coefficientsVisible = false;

    // Persists whatever is CURRENTLY ACTIVE (whichever row of the top-N
    // table below is selected, or the single result itself for a Manual/
    // Default-mode design that has no ranked list at all) as a user
    // override for its own exact spec - see
    // BBKDetachedPoleAudioProcessor::saveTopCandidateAsOverride(). Always
    // enabled; saving an already-instant result is harmless, just pointless.
    // A row's own Save button below (saveCandidateButtons) does the same
    // thing for a SPECIFIC row regardless of which one is currently active -
    // this one is just the "save whatever I'm listening to right now"
    // shortcut.
    juce::TextButton saveAsDefaultButton { "Save as Default" };

    // Top-N ranked-results table (see BBKDetachedPoleAudioProcessor::
    // DesignSnapshot::topCandidates / bbk::parametric::topCandidateCount):
    // one row per candidate the last completed Auto-mode search found,
    // best R_peak first, each with its own metrics plus a Use (switch to
    // it, no re-search - see selectTopCandidate()) and Save (persist it as
    // a user override - see saveTopCandidateAsOverride()) button. Populated
    // and shown/hidden per row in timerCallback() based on how many
    // candidates the current design snapshot actually has - Manual-mode
    // results and Default-mode bank entries have none, so every row stays
    // blank/hidden for those, exactly as if this table weren't there at all.
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

    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> cutoffAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> attenuationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> stopbandAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> sidelobeDecayAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> manualTapCountAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> tapCountAutoAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> maxSearchTimeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> maxSearchTimeHoursAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> perCandidateSearchTimeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> bypassAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> amplitudeRelaxationAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> defaultModeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> headroomAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> autoHeadroomAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BBKDetachedPoleAudioProcessorEditor)
};
