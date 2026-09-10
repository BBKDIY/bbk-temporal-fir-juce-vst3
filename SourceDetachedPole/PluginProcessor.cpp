#include "PluginProcessor.h"
#include "PluginEditor.h"
#include "PresetBankLookup.h"

#include <algorithm>
#include <cmath>

namespace
{
    // This plugin's minimax-designed lowpass has unity DC gain (checked
    // directly: designParametricFIR() always returns taps summing to
    // 1.0), but that does NOT mean unity PEAK gain - the design allows
    // sidelobe ripple (that's the whole point of "Sidelobe Decay"/R_peak),
    // and for any filter with negative taps, sum(|taps|) is strictly
    // greater than sum(taps) = 1. That gap is the exact worst-case peak
    // gain the filter can apply to any bounded input, and it is highly
    // sample-rate/cutoff dependent here: measured directly against the
    // actual designParametricFIR() output for the plugin's own default
    // spec (20 kHz cutoff, 0.5 dB attenuation, 98 dB stopband), it's only
    // about +0.6 dB at 192 kHz (19 taps) but grows to +5.7 dB at 48 kHz
    // and +6.4 dB at 44.1 kHz - because pushing the same 20 kHz cutoff
    // much closer to a lower Nyquist forces a far longer, sharper filter
    // (up to 65 taps) with correspondingly more sidelobe energy. Real
    // program material lands far below that ceiling in practice
    // (empirically ~+0.1 to +0.5 dB over, measured the same way as BBK
    // Phase Corrector/BBK Temporal FIR via Monte Carlo convolution against
    // the actual generated taps) - but since the worst case swings so
    // much with the host's sample rate and the user's own cutoff choice,
    // "headroom" defaults conservatively and Auto is relied on to find
    // the right level for whatever combination is actually in use.
    //
    // Applied only to the WET (filtered) signal, before it's mixed with
    // dry - BYPASS stays byte-for-byte untouched regardless of this
    // setting, same transparency principle as the other two BBK plugins.
    constexpr float softClipKneeStart = 0.891f; // ~ -1 dBFS: identity below this
    constexpr float softClipCeiling   = 0.999f; // ~ -0.01 dBFS: asymptote, never reached exactly

    inline double applySafetySoftClip (double x) noexcept
    {
        const double ax = std::abs (x);
        if (ax <= softClipKneeStart)
            return x;

        const double span = static_cast<double> (softClipCeiling) - softClipKneeStart;
        const double over = (ax - softClipKneeStart) / span;
        const double shaped = softClipKneeStart + span * std::tanh (over);
        return std::copysign (shaped, x);
    }

    // Auto-headroom calibration tuning - one-way ratchet only, same design
    // as the other two BBK plugins. Wider range than BBK Temporal FIR
    // since the measured worst case here goes up to +6.4 dB rather than
    // +4.24 dB, and could plausibly be worse still for a cutoff pushed
    // even closer to Nyquist than what was measured.
    constexpr float autoHeadroomStepDb           = 0.25f;
    constexpr float autoHeadroomMinDb            = -12.0f; // matches the "headroom" parameter's range floor
    constexpr float autoHeadroomTriggerLinear    = 0.01f;  // ~0.1 dB sustained excess over the knee before ratcheting
    constexpr float autoHeadroomReleasePerSecond = 0.5f;
    constexpr double autoHeadroomCooldownSeconds = 3.0;

    // Exact (not tolerance-based) comparison: every field here is either a
    // fixed enum or a double built the same deterministic way each time
    // (an atomic-loaded float widened to double, then at most a jmin/cast
    // against another such value - see specFromParameters()), so two
    // requests built from literally unchanged parameter values reproduce
    // the same bit pattern, not just a "close enough" one. That is exactly
    // what lets this be used as a real memory - see requestBoundaryRedesign().
    inline bool specsEqual (const bbk::parametric::FilterSpec& a, const bbk::parametric::FilterSpec& b) noexcept
    {
        return a.sampleRateHz == b.sampleRateHz
            && a.cutoffHz == b.cutoffHz
            && a.attenuationAtCutoffDb == b.attenuationAtCutoffDb
            && a.stopbandRejectionDb == b.stopbandRejectionDb
            && a.stopbandMode == b.stopbandMode
            && a.sidelobeDecayRatio == b.sidelobeDecayRatio;
    }
}

BBKDetachedPoleAudioProcessor::BBKDetachedPoleAudioProcessor()
: AudioProcessor (BusesProperties()
    .withInput ("Input", juce::AudioChannelSet::stereo(), true)
    .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
  juce::Thread ("BBKDetachedPole Design Thread"),
  parameters (*this, nullptr, "PARAMETERS", createParameterLayout())
{
    parameters.addParameterListener ("cutoff", &paramListener);
    parameters.addParameterListener ("attenuation", &paramListener);
    parameters.addParameterListener ("stopband", &paramListener);
    parameters.addParameterListener ("amplitudeRelaxation", &paramListener);
    parameters.addParameterListener ("sidelobeDecay", &paramListener);
    parameters.addParameterListener ("presetMode", &paramListener);
    parameters.addParameterListener ("tapCountAuto", &paramListener);
    parameters.addParameterListener ("manualTapCount", &paramListener);

    // Loaded once here, not per-lookup - see userOverrides' own comment in
    // PluginProcessor.h.
    userOverrides = bbk::detachedpole::useroverrides::loadAll();
    searchCache = bbk::detachedpole::searchcache::loadAll();

    startThread();
}

BBKDetachedPoleAudioProcessor::~BBKDetachedPoleAudioProcessor()
{
    // Unregister before any members are destroyed - the listener list
    // inside `parameters` holds a raw pointer to paramListener, and
    // members are destroyed in reverse declaration order (paramListener
    // is declared right after parameters), so this must happen here in
    // the destructor body rather than being left implicit.
    parameters.removeParameterListener ("cutoff", &paramListener);
    parameters.removeParameterListener ("attenuation", &paramListener);
    parameters.removeParameterListener ("stopband", &paramListener);
    parameters.removeParameterListener ("amplitudeRelaxation", &paramListener);
    parameters.removeParameterListener ("sidelobeDecay", &paramListener);
    parameters.removeParameterListener ("presetMode", &paramListener);
    parameters.removeParameterListener ("tapCountAuto", &paramListener);
    parameters.removeParameterListener ("manualTapCount", &paramListener);

    signalThreadShouldExit();
    notify();
    stopThread (5000);
}

juce::AudioProcessorValueTreeState::ParameterLayout BBKDetachedPoleAudioProcessor::createParameterLayout()
{
    using namespace bbk::detachedpole;
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "cutoff", 1 }, "Cutoff",
        juce::NormalisableRange<float> (static_cast<float> (minCutoffHz), static_cast<float> (maxCutoffHz), 1.0f),
        static_cast<float> (defaultCutoffHz)));

    // Step was 0.01 dB; too coarse for the sub-0.01 dB "near-flat passband"
    // operating points the paper's Case B needs (e.g. 0.0027 dB) - any
    // value typed with more precision than the step silently snapped to
    // the nearest multiple of it (0.0027 -> 0.00), which is exactly what
    // 0.00 dB can't do: a literal zero-tolerance passband has no feasible
    // 19-tap solution at all (see ParametricFIR.h). 0.0001 dB gives two
    // more decades of precision, enough to hit that point exactly, while
    // still being a defined step rather than a fully continuous range.
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "attenuation", 1 }, "Attenuation at Cutoff",
        juce::NormalisableRange<float> (static_cast<float> (minAttenuationDb), static_cast<float> (maxAttenuationDb), 0.0001f),
        static_cast<float> (defaultAttenuationDb)));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "stopband", 1 }, "Min. Stopband Rejection",
        juce::NormalisableRange<float> (static_cast<float> (minStopbandRejectionDb), static_cast<float> (maxStopbandRejectionDb), 0.1f),
        static_cast<float> (defaultStopbandRejectionDb)));

    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "bypass", 1 }, "Bypass", false));

    // Off by default (so every existing session/preset keeps behaving
    // exactly as before): when on, Cutoff/Min. Stopband Rejection/
    // Sidelobe Decay are forced to the exact operating point a large
    // offline sweep found to work well across every supported sample
    // rate (18.5kHz/95dB/no decay - see SourceDetachedPole/PresetSweep/),
    // and the design comes from an instant table lookup keyed on sample
    // rate + Attenuation instead of a live search - no multi-second (or,
    // for a demanding spec, multi-minute) wait after a slider move. Falls
    // back to a live Custom-mode design if the host's sample rate isn't
    // one of the 7 the bank was swept for (see requestBoundaryRedesign
    // and PresetBankLookup.h::findEntry).
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "presetMode", 1 }, "Default", false));

    // On (default): the attenuation slider above is used as-is (a Case
    // C-style spectrally relaxed design). Off: the attenuation slider is
    // ignored and the exact caseBNearFlatAttenuationDb constant is used
    // instead - a deterministic Case B reproduction that never depends on
    // typed slider precision (see DetachedPoleFilter.h for why that
    // matters at this scale).
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "amplitudeRelaxation", 1 }, "Amplitude Relaxation", true));

    // 1.0 (default, top of the range): the flat sidelobe bound used
    // above - an exact no-op (see ParametricFIR.h::FilterSpec::
    // sidelobeDecayRatio). Lower values progressively tighten the bound
    // on taps farther from the main lobe, concentrating ringing closer
    // to the centre followed by a quieter tail instead of a flat
    // plateau out to the tap boundary. Range extends down to 0.02
    // (verified directly against the engine's own dense-verify-and-
    // refine pipeline across this whole range, including well past it,
    // down to 0.001 - always comes back genuinely spectrally compliant,
    // never a silently-broken filter, though returns diminish sharply
    // below roughly 0.15-0.3 for a typical operating point). A skew
    // below 1 gives finer control in the lower, more perceptually
    // active part of the range without shrinking the reachable span.
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "sidelobeDecay", 1 }, "Sidelobe Decay",
        juce::NormalisableRange<float> (0.02f, 1.0f, 0.001f, 0.5f),
        1.0f));

    // See the anonymous namespace above process() for the full rationale:
    // the measured worst-case peak gain here ranges from about +0.6 dB
    // (192 kHz) to +6.4 dB (44.1 kHz, cutoff near Nyquist) depending on
    // sample rate and cutoff. -1.5 dB comfortably covers the empirically-
    // measured realistic worst case (~+0.5 dB) at any rate; Auto Headroom
    // (below) handles the rarer cases that need more than that, since a
    // single fixed default can't cover every sample-rate/cutoff
    // combination equally well.
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "headroom", 1 },
        "Headroom",
        juce::NormalisableRange<float> (-12.0f, 0.0f, 0.1f),
        -1.5f,
        juce::AudioParameterFloatAttributes().withLabel ("dB")));

    // One-way ratchet, default on - see the auto-headroom constants above
    // process() and PluginEditor.cpp for how manual edits turn this off.
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "autoHeadroom", 1 },
        "Auto Headroom",
        true));

    // Manual/Auto tap-count selector. On (default, unchanged behaviour):
    // designParametricFIR()'s own M-search picks whichever tap count gives
    // the best (lowest-R_peak, shortest-settling tie-break) compliant
    // result - see run() in this file. Off: the M-search is skipped
    // entirely and the design runs fixed at whatever "Manual Tap Count"
    // below is set to (after minimumFeasibleTapCount() - see ParametricFIR.h -
    // silently raises it if it's below the spec's true feasible floor, since
    // a request under that floor can only ever come back non-compliant) -
    // lets you deliberately trade ringing quality for a specific, known tap
    // count/latency, or pin a value for direct A/B comparison.
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "tapCountAuto", 1 },
        "Tap Count Auto",
        true));

    // Odd-only, 1 to maxTapCount (161): every value this engine can
    // actually produce is 2*M+1 for some half-length M (see
    // DetachedPoleFilter.h::maxHalfLength/maxTapCount), so a step of 2
    // keeps the slider itself from ever landing on a value the engine
    // would just silently round down anyway (see designParametricFIRFixedM()
    // in ParametricFIR.h). Default 19 - the paper's own Case B/C baseline
    // tap count (see Tests/DSPTestDetachedPole.cpp, which verifies it's
    // present in the selectable bank).
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "manualTapCount", 1 },
        "Manual Tap Count",
        juce::NormalisableRange<float> (1.0f, static_cast<float> (bbk::detachedpole::maxTapCount), 2.0f),
        19.0f));

    return layout;
}

bbk::parametric::FilterSpec BBKDetachedPoleAudioProcessor::specFromParameters() const
{
    bbk::parametric::FilterSpec spec;

    const double sr = currentSampleRate.load();
    spec.sampleRateHz = sr > 0.0 ? sr : 192000.0;
    const double nyquist = spec.sampleRateHz * 0.5;

    const double cutoffParam = static_cast<double> (parameters.getRawParameterValue ("cutoff")->load());
    // Guard against a cutoff parameter above this sample rate's own
    // Nyquist (e.g. the 20 kHz default at 44.1 kHz, whose Nyquist is only
    // 22.05 kHz) - clamp well inside it so a real transition band always
    // has room to exist.
    spec.cutoffHz = juce::jmin (cutoffParam, nyquist * 0.9);
    const bool relaxationOn = parameters.getRawParameterValue ("amplitudeRelaxation")->load() > 0.5f;
    spec.attenuationAtCutoffDb = relaxationOn
        ? static_cast<double> (parameters.getRawParameterValue ("attenuation")->load())
        : bbk::detachedpole::caseBNearFlatAttenuationDb;
    spec.stopbandRejectionDb = static_cast<double> (parameters.getRawParameterValue ("stopband")->load());

    // Fixed, not user-switchable: the whole cutoff-to-Nyquist span is
    // always treated as one free transition zone (see ParametricFIR.h's
    // StopbandMode::FreeTransition), with -stopbandRejectionDb enforced
    // only in a narrow guard band right at Nyquist. This was previously
    // an opt-in toggle; it is now the plugin's only behaviour. FlatMask
    // (the paper's own flat mirror-band mask) remains in the engine and
    // is still exercised by Tests/DSPTestDetachedPole.cpp as the direct
    // validation against the article's published Case B/C numbers, but
    // is no longer reachable from the plugin itself.
    spec.stopbandMode = bbk::parametric::StopbandMode::FreeTransition;

    spec.sidelobeDecayRatio = static_cast<double> (parameters.getRawParameterValue ("sidelobeDecay")->load());
    return spec;
}

void BBKDetachedPoleAudioProcessor::forcePresetOperatingPoint()
{
    using namespace bbk::detachedpole::presetbank;

    // Must happen before anything below overwrites the very values it's
    // trying to save - see its own comment.
    captureCustomPointBeforeDefault();

    // If the user has saved an override for this sample rate, Default mode
    // should recall THAT entire operating point - cutoff, attenuation,
    // stopband AND decay - rather than just snapping cutoff/stopband/decay
    // to the factory bank's fixed point and leaving attenuation wherever
    // the slider happens to be. Earlier behaviour only recalled the
    // override when the slider was already sitting on the exact
    // attenuation it was saved at, which meant wandering the Attenuation
    // slider away and then re-enabling Default silently fell back to the
    // factory entry for whatever attenuation was currently dialed in -
    // technically consistent with the exact-spec-match design, but not
    // what "save as MY default" means to a user: it should mean "this is
    // now the one thing Default recalls," full stop, the same way
    // reopening a saved preset recalls every field it was saved with.
    //
    // Matched on sample rate alone, most-recently-saved wins (userOverrides
    // is kept in save order - see saveCurrentAsOverride() - so the last
    // entry matching this rate is the one to use). Only sample rate can be
    // part of this lookup key: every other field is exactly what this
    // function is about to decide.
    const auto spec = specFromParameters();

    double targetCutoffHz = presetCutoffHz;
    double targetAttenuationDb = spec.attenuationAtCutoffDb;
    double targetStopbandRejectionDb = presetStopbandRejectionDb;
    double targetSidelobeDecayRatio = presetSidelobeDecayRatio;

    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        for (auto it = userOverrides.rbegin(); it != userOverrides.rend(); ++it)
        {
            if (it->spec.sampleRateHz == spec.sampleRateHz)
            {
                targetCutoffHz = it->spec.cutoffHz;
                targetAttenuationDb = it->spec.attenuationAtCutoffDb;
                targetStopbandRejectionDb = it->spec.stopbandRejectionDb;
                targetSidelobeDecayRatio = it->spec.sidelobeDecayRatio;
                break;
            }
        }
    }

    if (auto* cutoffParam = parameters.getParameter ("cutoff"))
        cutoffParam->setValueNotifyingHost (cutoffParam->convertTo0to1 (static_cast<float> (targetCutoffHz)));
    if (auto* attenuationParam = parameters.getParameter ("attenuation"))
        attenuationParam->setValueNotifyingHost (attenuationParam->convertTo0to1 (static_cast<float> (targetAttenuationDb)));
    if (auto* stopbandParam = parameters.getParameter ("stopband"))
        stopbandParam->setValueNotifyingHost (stopbandParam->convertTo0to1 (static_cast<float> (targetStopbandRejectionDb)));
    if (auto* decayParam = parameters.getParameter ("sidelobeDecay"))
        decayParam->setValueNotifyingHost (decayParam->convertTo0to1 (static_cast<float> (targetSidelobeDecayRatio)));
}

void BBKDetachedPoleAudioProcessor::captureCustomPointBeforeDefault()
{
    const juce::SpinLock::ScopedLockType sl (specLock);

    // currentBoundaryPresetMode still reflects the state BEFORE this
    // transition (requestBoundaryRedesign(), which updates it, hasn't run
    // yet for this event - see ParamListener's ordering) - so "already
    // true" here means this is a resent "on" event for a mode we're
    // already in (some hosts resend automation at the playhead on
    // transport start/stop), not a genuine off->on transition. Skipping
    // the capture in that case is essential: the values visible right now
    // are already the FORCED Default/override ones, not the user's real
    // Custom point, so capturing here would silently overwrite the
    // genuine snapshot taken at the real transition with a copy of
    // Default's own operating point - exactly the bug this exists to fix,
    // just moved one step later.
    if (currentBoundaryPresetMode)
        return;

    preDefaultSnapshot.cutoffHz = static_cast<double> (parameters.getRawParameterValue ("cutoff")->load());
    preDefaultSnapshot.attenuationAtCutoffDb = static_cast<double> (parameters.getRawParameterValue ("attenuation")->load());
    preDefaultSnapshot.stopbandRejectionDb = static_cast<double> (parameters.getRawParameterValue ("stopband")->load());
    preDefaultSnapshot.sidelobeDecayRatio = static_cast<double> (parameters.getRawParameterValue ("sidelobeDecay")->load());
    havePreDefaultSnapshot = true;
}

void BBKDetachedPoleAudioProcessor::restoreCustomPointBeforeDefault()
{
    PreDefaultSnapshot snap;
    bool have;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);

        // Same resend guard as captureCustomPointBeforeDefault(), mirrored:
        // currentBoundaryPresetMode still reflects the state before THIS
        // transition, so "already false" means a resent "off" event while
        // already off - nothing to restore, and restoring again would be
        // harmless but pointless.
        if (! currentBoundaryPresetMode)
            return;

        snap = preDefaultSnapshot;
        have = havePreDefaultSnapshot;
    }

    // Default was never actually engaged this session (e.g. a stray "off"
    // notification with no prior "on") - nothing was ever overwritten, so
    // there is nothing to put back.
    if (! have)
        return;

    if (auto* cutoffParam = parameters.getParameter ("cutoff"))
        cutoffParam->setValueNotifyingHost (cutoffParam->convertTo0to1 (static_cast<float> (snap.cutoffHz)));
    if (auto* attenuationParam = parameters.getParameter ("attenuation"))
        attenuationParam->setValueNotifyingHost (attenuationParam->convertTo0to1 (static_cast<float> (snap.attenuationAtCutoffDb)));
    if (auto* stopbandParam = parameters.getParameter ("stopband"))
        stopbandParam->setValueNotifyingHost (stopbandParam->convertTo0to1 (static_cast<float> (snap.stopbandRejectionDb)));
    if (auto* decayParam = parameters.getParameter ("sidelobeDecay"))
        decayParam->setValueNotifyingHost (decayParam->convertTo0to1 (static_cast<float> (snap.sidelobeDecayRatio)));
}

void BBKDetachedPoleAudioProcessor::saveCurrentAsOverride()
{
    // Whatever is currently published - a live Custom-mode result, a
    // Default-mode bank entry, or even an existing override - becomes the
    // new override for its own exact spec. Reads the UI snapshot rather
    // than latestResult/latestSpec directly since that's already the
    // single point that's guaranteed consistent (spec and result written
    // together under uiSnapshotLock in publishResult()).
    const auto snap = getDesignSnapshotForUI();

    // Nothing designed yet (e.g. called before the very first design
    // completes) - silently do nothing rather than persist a hollow entry.
    if (snap.tapCount <= 0 || snap.taps.empty())
        return;

    bbk::detachedpole::useroverrides::OverrideEntry entry;
    entry.spec.sampleRateHz = snap.sampleRateHz;
    entry.spec.cutoffHz = snap.cutoffHz;
    entry.spec.attenuationAtCutoffDb = snap.attenuationAtCutoffDb;
    entry.spec.stopbandRejectionDb = snap.stopbandRejectionDb;
    entry.spec.stopbandMode = snap.stopbandMode;
    entry.spec.sidelobeDecayRatio = snap.sidelobeDecayRatio;
    entry.tapCount = snap.tapCount;
    entry.achievedStopbandDb = snap.achievedStopbandDb;
    entry.taps = snap.taps;

    {
        const juce::SpinLock::ScopedLockType sl (specLock);

        // Erase-then-push_back (not update-in-place) so the just-saved
        // entry always ends up last - forcePresetOperatingPoint() relies
        // on vector order to find the MOST RECENTLY saved override for a
        // sample rate (scanning from the back), since there's no separate
        // timestamp field.
        userOverrides.erase (std::remove_if (userOverrides.begin(), userOverrides.end(),
                                              [&] (const auto& e) { return specsEqual (e.spec, entry.spec); }),
                              userOverrides.end());
        userOverrides.push_back (entry);
    }

    // Persist to disk. userOverrides is only ever appended/replaced-in-
    // place above, so copying it out here (rather than holding specLock
    // across the file write) is safe - saveAll() takes a snapshot by
    // value anyway.
    std::vector<bbk::detachedpole::useroverrides::OverrideEntry> toSave;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        toSave = userOverrides;
    }
    bbk::detachedpole::useroverrides::saveAll (toSave);

    // Reflect the save immediately in the UI as a UserOverride result -
    // the spec hasn't changed, so requestBoundaryRedesign()'s own dedup
    // guard would otherwise no-op this and leave the "Design method:"
    // label saying Custom/Default even though a saved override now exists
    // for this exact spec.
    bbk::parametric::DesignResult result;
    result.taps = entry.taps;
    result.tapCount = entry.tapCount;
    result.constraintsMet = true;
    result.achievedStopbandDb = entry.achievedStopbandDb;
    result.designAttempts = 0;
    result.temporal = bbk::parametric::computeTemporalMetrics (result.taps, entry.spec.sampleRateHz);

    publishResult (entry.spec, result, ResultSource::UserOverride);
}

void BBKDetachedPoleAudioProcessor::requestBoundaryRedesign()
{
    // May be called from the message thread (typical - a slider moved) or
    // from the audio thread (a host delivered automation for one of these
    // parameters mid-block) - either way this only ever copies a small
    // FilterSpec, queues one lightweight DesignTask under a SpinLock, and
    // signals the worker thread; the actual (slow) design work never runs
    // here. Bumping boundaryEpoch and clearing taskQueue means any design
    // already queued or mid-flight for an older boundary is discarded
    // rather than published once it finishes - only the newest request
    // ever wins.
    if (! hasPrepared.load() || currentSampleRate.load() <= 0.0)
        return;

    const auto spec = specFromParameters();

    // Default mode: an instant lookup instead of a live design, IF the
    // host's current sample rate is one of the 7 the bank was swept for.
    // Looked up here (before the lock below) since it only reads static
    // table data - no need to hold specLock for it. The bank always
    // matches spec.attenuationAtCutoffDb, not the raw slider value:
    // amplitudeRelaxation off substitutes caseBNearFlatAttenuationDb
    // (well outside the swept 0.05-0.50dB grid), so relaxation-off specs
    // correctly find no entry and fall back to a live design below,
    // rather than silently returning some unrelated preset.
    const bool presetModeOn = parameters.getRawParameterValue ("presetMode")->load() > 0.5f;
    const auto* presetEntry = presetModeOn
        ? bbk::detachedpole::presetbank::findEntry (spec.sampleRateHz, spec.attenuationAtCutoffDb)
        : nullptr;

    // Also read early, same reasoning as presetModeOn above: needed both
    // for the "did anything actually change" dedup check below and for
    // the else branch further down (search-cache gating and the queued
    // DesignTask itself) - see requestedTapCount's own comment on why it
    // only matters while manualTapCountOn is true.
    const bool manualTapCountOn = parameters.getRawParameterValue ("tapCountAuto")->load() <= 0.5f;
    const int requestedTapCount = static_cast<int> (parameters.getRawParameterValue ("manualTapCount")->load());

    bool haveInstantResult = false;
    bbk::parametric::DesignResult instantResult;
    ResultSource instantSource = ResultSource::LiveSearch;

    {
        const juce::SpinLock::ScopedLockType sl (specLock);

        // The "memory": a parameter listener fires on every value the host
        // writes, not just ones that actually changed - some hosts resend
        // the automation value at the playhead when transport starts (or
        // resend full state on stop), which used to land here as an
        // apparently-fresh boundary change even though the resulting spec
        // is identical to the one already active (or already mid-design)
        // for currentBoundarySpec. Without this check that still discarded
        // and restarted whatever was queued/running, so hitting stop then
        // play could re-trigger the full multi-second LP search for the
        // exact same filter that was already computed. A real spec change
        // never takes this early-return path (specsEqual is exact, not
        // fuzzy - see its own comment) and proceeds exactly as before.
        // presetModeOn is compared alongside the spec itself (see
        // currentBoundaryPresetMode's own comment) - toggling Default
        // on/off must always force at least one fresh lookup/design even
        // when every FilterSpec field is unchanged, since the two modes
        // pull the result from different places (instant bank lookup vs.
        // live search).
        // manualTapCountOn is compared unconditionally (Auto->Manual or
        // Manual->Auto must always force a fresh redesign), but
        // requestedTapCount only when manualTapCountOn is true - comparing
        // it unconditionally would force a spurious redesign on every mouse
        // nudge of the manual slider even while Auto mode is on and
        // ignoring it entirely.
        if (specsEqual (spec, currentBoundarySpec)
            && presetModeOn == currentBoundaryPresetMode
            && manualTapCountOn == currentBoundaryManualTapCountOn
            && (! manualTapCountOn || requestedTapCount == currentBoundaryRequestedTapCount)
            && boundaryEpoch != 0)
            return;

        currentBoundarySpec = spec;
        currentBoundaryPresetMode = presetModeOn;
        currentBoundaryManualTapCountOn = manualTapCountOn;
        currentBoundaryRequestedTapCount = requestedTapCount;
        ++boundaryEpoch;
        taskQueue.clear(); // also discards any now-stale in-flight live design

        // User overrides win regardless of Default/Custom mode (checked
        // even when presetEntry above is null, i.e. Custom mode or an
        // untabled sample rate) - see saveCurrentAsOverride()'s own
        // comment. userOverrides is guarded by this same specLock (see its
        // declaration in the header) since this function, like the rest of
        // the specLock-guarded block, can run on the audio thread.
        const bbk::detachedpole::useroverrides::OverrideEntry* overrideEntry = nullptr;
        for (auto& e : userOverrides)
        {
            if (specsEqual (e.spec, spec))
            {
                overrideEntry = &e;
                break;
            }
        }

        if (overrideEntry != nullptr)
        {
            haveInstantResult = true;
            instantSource = ResultSource::UserOverride;
            instantResult.taps = overrideEntry->taps;
            instantResult.tapCount = overrideEntry->tapCount;
            instantResult.constraintsMet = true;
            instantResult.achievedStopbandDb = overrideEntry->achievedStopbandDb;
            instantResult.designAttempts = 0;
            instantResult.temporal = bbk::parametric::computeTemporalMetrics (instantResult.taps, spec.sampleRateHz);
        }
        else if (presetEntry != nullptr)
        {
            haveInstantResult = true;
            instantSource = ResultSource::PresetBank;
            instantResult.taps = presetEntry->taps;
            instantResult.tapCount = presetEntry->tapCount;
            instantResult.constraintsMet = true;
            instantResult.achievedStopbandDb = presetEntry->achievedStopbandDb;
            instantResult.designAttempts = 0;
            instantResult.temporal = bbk::parametric::computeTemporalMetrics (instantResult.taps, spec.sampleRateHz);
        }
        else
        {
            // Below both curated sources above: an exact spec we've
            // already paid to search before, this session or an earlier
            // one - see LiveSearchCache.h. Checked in every mode (not just
            // Custom), same reasoning as userOverrides: costs nothing to
            // check, and covers an untabled sample rate in Default mode
            // too.
            //
            // Skipped entirely while Manual tap-count mode is on, in both
            // directions: an existing cache entry may have been produced by
            // an Auto search that landed on a different (better-ringing)
            // tap count than the one explicitly dialed in here, so serving
            // it would silently ignore the user's request - and a Manual-
            // mode result is a deliberate, one-off A/B point rather than
            // "the" answer for this spec, so it must never be written back
            // into the cache either and served later to an Auto-mode
            // lookup for the same spec (see run()'s matching guard on the
            // write side).
            const bbk::detachedpole::searchcache::CacheEntry* cacheEntry = nullptr;
            if (! manualTapCountOn)
            {
                for (auto& e : searchCache)
                {
                    if (specsEqual (e.spec, spec))
                    {
                        cacheEntry = &e;
                        break;
                    }
                }
            }

            if (cacheEntry != nullptr)
            {
                haveInstantResult = true;
                instantSource = ResultSource::SearchCache;
                instantResult.taps = cacheEntry->taps;
                instantResult.tapCount = cacheEntry->tapCount;
                instantResult.constraintsMet = true;
                instantResult.achievedStopbandDb = cacheEntry->achievedStopbandDb;
                instantResult.designAttempts = 0;
                instantResult.temporal = bbk::parametric::computeTemporalMetrics (instantResult.taps, spec.sampleRateHz);
            }
            else
            {
                DesignTask task;
                task.spec = spec;
                task.epoch = boundaryEpoch;
                task.manualTapCount = manualTapCountOn;
                task.requestedTapCount = requestedTapCount;
                taskQueue.push_back (task);

                // See isSearchInProgressForUI()'s own comment: this is the
                // one place real background work gets queued, so it's the
                // one place this needs to turn on.
                searchInProgress.store (true);
            }
        }
    }

    // publishResult() takes specLock itself (briefly, for versionCounter) -
    // must be called after the block above releases it, not from inside.
    if (haveInstantResult)
    {
        publishResult (spec, instantResult, instantSource);

        // An instant result means there is nothing left to wait for, for
        // THIS boundary change - see isSearchInProgressForUI()'s own
        // comment. If an older, now-superseded background task happens to
        // still be running at this exact moment, its own staleness check
        // in run() will simply discard its result without touching this
        // flag once it finishes, so this assignment is never clobbered
        // late by that stale task.
        searchInProgress.store (false);
    }
    else
        notify();
}

void BBKDetachedPoleAudioProcessor::publishResult (const bbk::parametric::FilterSpec& spec,
                                                     const bbk::parametric::DesignResult& result,
                                                     ResultSource source)
{
    int version;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        version = ++versionCounter;
    }

    bool isNewest = false;
    {
        const juce::SpinLock::ScopedLockType sl (resultLock);
        if (version > latestVersion)
        {
            latestResult = result;
            latestSpec = spec;
            latestVersion = version;
            isNewest = true;
        }
    }

    if (! isNewest)
        return;

    {
        const juce::SpinLock::ScopedLockType sl (uiSnapshotLock);
        uiSnapshot.sampleRateHz = spec.sampleRateHz;
        uiSnapshot.cutoffHz = spec.cutoffHz;
        uiSnapshot.attenuationAtCutoffDb = spec.attenuationAtCutoffDb;
        uiSnapshot.stopbandRejectionDb = spec.stopbandRejectionDb;
        uiSnapshot.stopbandMode = spec.stopbandMode;
        uiSnapshot.sidelobeDecayRatio = spec.sidelobeDecayRatio;
        uiSnapshot.amplitudeRelaxationOn = parameters.getRawParameterValue ("amplitudeRelaxation")->load() > 0.5f;
        uiSnapshot.tapCount = result.tapCount;
        uiSnapshot.achievedStopbandDb = result.achievedStopbandDb;
        uiSnapshot.constraintsMet = result.constraintsMet;
        uiSnapshot.designAttempts = result.designAttempts;
        uiSnapshot.source = source;
        uiSnapshot.taps = result.taps;
        uiSnapshot.temporal = result.temporal;
    }
}

void BBKDetachedPoleAudioProcessor::run()
{
    while (! threadShouldExit())
    {
        DesignTask task;
        bool haveTask = false;
        {
            const juce::SpinLock::ScopedLockType sl (specLock);
            if (! taskQueue.empty())
            {
                task = taskQueue.front();
                taskQueue.pop_front();
                haveTask = true;
            }
        }

        if (! haveTask)
        {
            wait (-1);
            continue;
        }

        if (threadShouldExit())
            break;

        // Drop it if a newer boundary change has already superseded it -
        // no point spending time designing a spec nobody wants any more.
        {
            const juce::SpinLock::ScopedLockType sl (specLock);
            if (task.epoch != boundaryEpoch)
                continue;
        }

        // Custom-mode's live search: 900s overall / 60s per candidate,
        // both well above ParametricFIR.h's own real-time-appropriate
        // defaults (180s/15s). Safe to be this patient now that Default
        // mode (see requestBoundaryRedesign()) gives an always-available
        // instant result while this runs in the background - Custom was
        // previously tuned to not make the user wait too long for SOME
        // result, but that's no longer the only result they have.
        //
        // Manual tap-count mode: skip the auto M-search entirely and design
        // fixed at exactly task.requestedTapCount, after silently raising it
        // to the spec's true feasible floor first via minimumFeasibleTapCount()
        // if it's below that (see its own comment in ParametricFIR.h for why
        // a request under that floor is never a useful result to show). Uses
        // the same generous time budget as the Auto path below so the
        // fixed-M solve gets the same convergence opportunity a search
        // candidate at that same M would have gotten.
        bbk::parametric::DesignResult result;
        if (task.manualTapCount)
        {
            const int floor = bbk::parametric::minimumFeasibleTapCount (task.spec, bbk::detachedpole::maxTapCount, 30.0);
            const int target = juce::jmax (task.requestedTapCount, floor);
            result = bbk::parametric::designParametricFIRFixedM (task.spec, target, 900.0, 60.0);
        }
        else
        {
            result = bbk::parametric::designParametricFIR (task.spec, bbk::detachedpole::maxTapCount, 900.0, 60.0);
        }

        // Re-check staleness after the (possibly slow - see
        // ParametricFIR.h for how thorough this search now is)
        // computation - a newer boundary change may have arrived while
        // this task was running. If so, the work is simply discarded, not
        // published.
        bool stillCurrent;
        {
            const juce::SpinLock::ScopedLockType sl (specLock);
            stillCurrent = (task.epoch == boundaryEpoch);
        }
        if (! stillCurrent)
            continue;

        // Remember this result so revisiting the exact same spec later -
        // this session or a future one - is instant (see LiveSearchCache.h
        // and requestBoundaryRedesign()'s own lookup). Skipped if the taps
        // came back empty (a degenerate/failed search, nothing usable to
        // remember). Oldest-first eviction once over the cap: cheap, and a
        // spec searched again naturally re-enters at the back anyway.
        //
        // Also skipped entirely for a Manual tap-count task - see
        // requestBoundaryRedesign()'s matching guard on the lookup side for
        // why a Manual-mode result must never be written into (or served
        // from) the same cache an Auto search uses.
        if (! task.manualTapCount && result.tapCount > 0 && ! result.taps.empty())
        {
            bbk::detachedpole::searchcache::CacheEntry entry;
            entry.spec = task.spec;
            entry.tapCount = result.tapCount;
            entry.achievedStopbandDb = result.achievedStopbandDb;
            entry.taps = result.taps;

            std::vector<bbk::detachedpole::searchcache::CacheEntry> toSave;
            {
                const juce::SpinLock::ScopedLockType sl (specLock);

                searchCache.erase (std::remove_if (searchCache.begin(), searchCache.end(),
                                                    [&] (const auto& e) { return specsEqual (e.spec, entry.spec); }),
                                   searchCache.end());
                searchCache.push_back (entry);

                while (static_cast<int> (searchCache.size()) > bbk::detachedpole::searchcache::maxEntries)
                    searchCache.erase (searchCache.begin());

                toSave = searchCache;
            }
            bbk::detachedpole::searchcache::saveAll (toSave);
        }

        publishResult (task.spec, result);

        // See isSearchInProgressForUI()'s own comment: this is the one
        // place a background search's result actually reaches the UI, so
        // it's the one place in run() this needs to turn back off. Only
        // reached on the branch that just confirmed task.epoch ==
        // boundaryEpoch (stillCurrent) - a stale/discarded task takes the
        // `continue` above instead and never reaches here, exactly as
        // isSearchInProgressForUI()'s comment describes.
        searchInProgress.store (false);
    }
}

BBKDetachedPoleAudioProcessor::DesignSnapshot BBKDetachedPoleAudioProcessor::getDesignSnapshotForUI() const
{
    const juce::SpinLock::ScopedLockType sl (uiSnapshotLock);
    return uiSnapshot;
}

void BBKDetachedPoleAudioProcessor::prepareToPlay (double sampleRate, int)
{
    currentSampleRate.store (sampleRate);

    const int requiredChannels = juce::jmax (getTotalNumInputChannels(), getTotalNumOutputChannels());
    constexpr int channelHeadroom = 16;
    const int channelsToAllocate = juce::jmax (requiredChannels, channelHeadroom);

    const bool formatChanged = ! hasPrepared.load()
                             || std::abs (sampleRate - lastPreparedSampleRate) > 0.5
                             || static_cast<int> (channels.size()) < channelsToAllocate;

    // Set before requestBoundaryRedesign() below, which gates on it -
    // this call is always itself the moment "prepared" becomes true, so
    // there is no reason to make that method wait for a later statement.
    hasPrepared.store (true);

    if (formatChanged)
    {
        channels.resize (static_cast<std::size_t> (channelsToAllocate));
        for (auto& channel : channels)
            channel.clear();

        designCrossfade.reset (sampleRate, 0.015); // 15 ms click-free redesign crossfade
        crossfading = false;

        lastBypassParam = parameters.getRawParameterValue ("bypass")->load() > 0.5f;
        bypassCrossfade.reset (sampleRate, 0.015); // same 15 ms as the redesign crossfade
        bypassCrossfade.setCurrentAndTargetValue (lastBypassParam ? 1.0 : 0.0);

        // A sample-rate (or first-ever) change is a natural discontinuity
        // anyway, but the real design for that rate can legitimately take
        // several seconds for some cutoff/rate combinations pushed close
        // to Nyquist (see ParametricFIR.h) - far too long to block
        // prepareToPlay, which hosts expect back in milliseconds and may
        // treat as a hung plugin otherwise. Install the safe identity
        // pass-through immediately (this is directly on the audio-thread-
        // owned tap arrays, safe here specifically because the host
        // guarantees prepareToPlay is never concurrent with
        // processBlock()), then hand the real design for this rate to the
        // same background worker and crossfade already used for live
        // slider changes - see DetachedPoleFilter.h::identityTaps().
        activeTaps = bbk::detachedpole::identityTaps();
        incomingTaps = activeTaps;

        // Mark the UI snapshot as stale (tapCount==0 already means
        // "no completed design yet" to the editor - see timerCallback(),
        // which shows a "Designing..." message for exactly this case) so
        // it stops showing the *previous* rate's now-irrelevant tap
        // count / achieved-stopband / temporal metrics while the identity
        // pass-through above is what is actually playing.
        {
            const juce::SpinLock::ScopedLockType sl (uiSnapshotLock);
            uiSnapshot.tapCount = 0;
        }

        requestBoundaryRedesign();
    }

    lastPreparedSampleRate = sampleRate;

    clipEnvelope = 0.0f;
    samplesUntilNextAutoAdjust = 0;
    autoAdjustCooldownSamples = static_cast<int> (sampleRate * autoHeadroomCooldownSeconds);

    // maxHalfLength samples, always - see DetachedPoleFilter.h. Every
    // design is zero-padded to this same fixed length, so the
    // host-reported latency never changes at runtime regardless of slider
    // values or sample rate.
    setLatencySamples (bbk::detachedpole::latencySamples);
}

bool BBKDetachedPoleAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto input = layouts.getMainInputChannelSet();
    const auto output = layouts.getMainOutputChannelSet();

    if (input != output)
        return false;

    return output == juce::AudioChannelSet::mono()
        || output == juce::AudioChannelSet::stereo();
}

template <typename SampleType>
void BBKDetachedPoleAudioProcessor::process (juce::AudioBuffer<SampleType>& buffer)
{
    using namespace bbk::detachedpole;

    const int numChannels = buffer.getNumChannels();
    const int numSamples = buffer.getNumSamples();

    if (static_cast<int> (channels.size()) < numChannels)
    {
        const auto oldSize = channels.size();
        channels.resize (static_cast<std::size_t> (numChannels));
        for (auto i = oldSize; i < channels.size(); ++i)
            channels[i].clear();
    }

    // Best-effort, non-blocking pickup of a newer background design. If
    // the worker thread happens to be mid-publish this block simply
    // misses it and picks it up on the next block - never worth blocking
    // the audio thread for.
    {
        const juce::SpinLock::ScopedTryLockType tl (resultLock);
        if (tl.isLocked() && latestVersion != consumedVersion)
        {
            consumedVersion = latestVersion;

            // A design published for a sample rate that is no longer the
            // host's current one can only happen if the host changed
            // rates again before this background design finished (see
            // prepareToPlay(), which is now fully async and can overlap
            // like this in principle, however briefly) - crossfading it
            // in would apply the wrong cutoff, since its taps encode a
            // frequency response normalised to the *old* rate. Silently
            // discard it instead; the redesign already requested for the
            // current rate carries a higher version number and will
            // still be picked up normally once it publishes.
            if (std::abs (latestSpec.sampleRateHz - currentSampleRate.load()) <= 0.5)
            {
                incomingTaps = padTapsToFixedLength (latestResult.taps);
                crossfading = true;
                designCrossfade.setCurrentAndTargetValue (0.0);
                designCrossfade.setTargetValue (1.0);
            }
        }
    }

    // Bypass toggle: retarget (never reset) the independent bypass
    // crossfade so a toggle mid-ramp reverses smoothly from wherever it
    // currently is, rather than jumping.
    const bool bypassNow = parameters.getRawParameterValue ("bypass")->load() > 0.5f;
    if (bypassNow != lastBypassParam)
    {
        lastBypassParam = bypassNow;
        bypassCrossfade.setTargetValue (bypassNow ? 1.0 : 0.0);
    }

    // See the anonymous namespace above for the full rationale. Read once
    // per block, not per sample.
    const float headroomDb = parameters.getRawParameterValue ("headroom")->load();
    const double preAttenuationGain = juce::Decibels::decibelsToGain (headroomDb);
    const bool autoHeadroomEnabled = parameters.getRawParameterValue ("autoHeadroom")->load() > 0.5f;
    double blockPeakOver = 0.0;

    for (int sample = 0; sample < numSamples; ++sample)
    {
        const double crossfadeAmount = crossfading ? designCrossfade.getNextValue() : 1.0;
        const double bypassAmount = bypassCrossfade.getNextValue();

        for (int ch = 0; ch < numChannels; ++ch)
        {
            auto& state = channels[static_cast<std::size_t> (ch)];
            auto* data = buffer.getWritePointer (ch);
            const double x = static_cast<double> (data[sample]);

            state.history[static_cast<std::size_t> (state.writeIndex)] = x;

            double wOld = 0.0;
            for (int k = 0; k < maxTapCount; ++k)
            {
                int index = state.writeIndex - k;
                if (index < 0) index += historyLength;
                wOld += activeTaps[static_cast<std::size_t> (k)] * state.history[static_cast<std::size_t> (index)];
            }

            double wet = wOld;
            if (crossfading)
            {
                double wNew = 0.0;
                for (int k = 0; k < maxTapCount; ++k)
                {
                    int index = state.writeIndex - k;
                    if (index < 0) index += historyLength;
                    wNew += incomingTaps[static_cast<std::size_t> (k)] * state.history[static_cast<std::size_t> (index)];
                }
                wet = wOld + crossfadeAmount * (wNew - wOld);
            }

            // Pad + soft-clip backstop applied to the WET signal only - the
            // headroom gain and clip backstop are both design-side
            // corrections for the filter's own overshoot, so neither
            // belongs on the dry path itself. See the anonymous namespace
            // above for why the backstop is needed at all.
            wet *= preAttenuationGain;

            const double absWet = std::abs (wet);
            if (absWet > softClipKneeStart)
                blockPeakOver = std::max (blockPeakOver, absWet - static_cast<double> (softClipKneeStart));

            wet = applySafetySoftClip (wet);

            // Dry path delayed by exactly latencySamples - the same fixed
            // group delay every design has by construction (centre tap
            // always at maxHalfLength), so bypassing lines up
            // sample-for-sample with the filtered signal it is fading
            // against. Also scaled by the same headroom gain as the wet
            // path (rather than left at unity) so toggling Bypass is a
            // pure A/B of the filter's tonal effect at matched loudness,
            // not a loudness jump - the headroom pad exists purely to
            // avoid clipping the wet signal's overshoot and has nothing to
            // do with the dry signal's own level.
            int dryIndex = state.writeIndex - bbk::detachedpole::latencySamples;
            if (dryIndex < 0) dryIndex += historyLength;
            const double dry = state.history[static_cast<std::size_t> (dryIndex)] * preAttenuationGain;

            const double out = wet + bypassAmount * (dry - wet);
            data[sample] = static_cast<SampleType> (out);

            if (++state.writeIndex == historyLength)
                state.writeIndex = 0;
        }
    }

    if (crossfading && ! designCrossfade.isSmoothing())
    {
        activeTaps = incomingTaps;
        crossfading = false;
    }

    // Clip indicator: stamped independently of Auto Headroom being on -
    // this reflects "the backstop actually engaged this block", the exact
    // same detection that feeds the Auto ratchet below, so the light and
    // Auto never disagree about what counts as a clip.
    if (blockPeakOver > 0.0 && ! bypassNow)
        lastClipTimeMs.store (juce::Time::getMillisecondCounter());

    // Auto-headroom: one-way ratchet only, same design as the other two
    // BBK plugins' Auto Headroom - see the tuning constants above and the
    // member comments in PluginProcessor.h. Only runs while not bypassed;
    // "bypassNow" (not bypassAmount, which is still ramping mid-crossfade)
    // is the right gate here since it reflects the actual target state.
    if (autoHeadroomEnabled && ! bypassNow)
    {
        const auto sampleRate = currentSampleRate.load();
        if (sampleRate > 0.0)
        {
            const float releaseThisBlock = autoHeadroomReleasePerSecond
                                          * static_cast<float> (numSamples)
                                          / static_cast<float> (sampleRate);
            clipEnvelope = std::max (static_cast<float> (blockPeakOver), clipEnvelope - releaseThisBlock);

            samplesUntilNextAutoAdjust -= numSamples;

            if (clipEnvelope > autoHeadroomTriggerLinear && samplesUntilNextAutoAdjust <= 0)
            {
                if (auto* headroomParam = parameters.getParameter ("headroom"))
                {
                    const float newDb = std::max (autoHeadroomMinDb, headroomDb - autoHeadroomStepDb);
                    if (newDb != headroomDb)
                        headroomParam->setValueNotifyingHost (headroomParam->convertTo0to1 (newDb));
                }

                clipEnvelope = 0.0f;
                samplesUntilNextAutoAdjust = autoAdjustCooldownSamples;
            }
        }
    }
}

void BBKDetachedPoleAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    process (buffer);
}

void BBKDetachedPoleAudioProcessor::processBlock (juce::AudioBuffer<double>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    process (buffer);
}

juce::AudioProcessorEditor* BBKDetachedPoleAudioProcessor::createEditor()
{
    return new BBKDetachedPoleAudioProcessorEditor (*this);
}

void BBKDetachedPoleAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = parameters.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void BBKDetachedPoleAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
        if (xml->hasTagName (parameters.state.getType()))
            parameters.replaceState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new BBKDetachedPoleAudioProcessor();
}
