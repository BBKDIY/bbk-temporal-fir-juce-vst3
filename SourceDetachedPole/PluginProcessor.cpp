#include "PluginProcessor.h"
#include "PluginEditor.h"
#include "PresetBankLookup.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

// Windows-only - this plugin only ships for Windows (see README) - and
// kept out of PluginProcessor.h so that header stays a plain,
// always-compilable declaration. Used by pollSystemCpuBusyFraction() (raw
// GetSystemTimes()) and by run()'s onWorkerThreadStart hook (raw
// SetThreadPriority() on each spawned search worker, so the OS scheduler
// always favours the real-time audio thread over these if they ever
// genuinely contend for the same core - see currentAllowedSearchConcurrency()'s
// own comment for why dynamic throttling alone isn't relied on as the only
// safeguard).
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

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

    // Combines a just-finished search's own top-N candidates with whatever
    // was already cached for this exact spec (if anything), re-ranks the
    // union purely by R_peak, and keeps only the best
    // bbk::parametric::topCandidateCount - see run()'s own comment on why
    // this replaces a plain overwrite. R_peak is recomputed here rather
    // than read off either list, since RankedCandidate deliberately doesn't
    // store it (see its own comment in ParametricFIR.h) - computeTemporalMetrics
    // is a pure function of taps + sample rate, so this is cheap and exact,
    // not an approximation. Deduplicated by tap count: if both lists happen
    // to already have an entry at the same M, the existing (already-cached,
    // already-shown-to-the-user) one wins rather than being swapped for a
    // numerically different solution at the same M that offers nothing new.
    inline std::vector<bbk::parametric::RankedCandidate> mergeRankedCandidates (
        const std::vector<bbk::parametric::RankedCandidate>& existing,
        const std::vector<bbk::parametric::RankedCandidate>& fresh,
        double sampleRateHz)
    {
        std::vector<bbk::parametric::RankedCandidate> merged = existing;
        for (auto& c : fresh)
        {
            const bool haveThisTapCountAlready = std::any_of (merged.begin(), merged.end(),
                [&] (const auto& e) { return e.tapCount == c.tapCount; });
            if (! haveThisTapCountAlready)
                merged.push_back (c);
        }

        std::sort (merged.begin(), merged.end(), [sampleRateHz] (const auto& a, const auto& b)
        {
            return bbk::parametric::computeTemporalMetrics (a.taps, sampleRateHz).rPeakPercent
                 < bbk::parametric::computeTemporalMetrics (b.taps, sampleRateHz).rPeakPercent;
        });

        if (merged.size() > static_cast<std::size_t> (bbk::parametric::topCandidateCount))
            merged.resize (static_cast<std::size_t> (bbk::parametric::topCandidateCount));

        return merged;
    }

    // Hard ceiling on how many attemptDesign() workers a single search
    // "round" may ever use, regardless of how idle the machine looks -
    // even a genuinely idle 64-core server has no reason to run more of
    // these at once than this, since each one is already a full LP solve
    // and diminishing returns from further parallel candidates arrive long
    // before this many would ever be useful.
    constexpr int maxSearchWorkerThreads = 20;

    // Cores always left for the OS/host/audio thread, never handed to the
    // search regardless of how many the system reports - see
    // BBKDetachedPoleAudioProcessor::currentAllowedSearchConcurrency().
    // Slightly more generous on very small machines (a dual-core laptop
    // dedicating a whole core to a background search is a much bigger
    // relative hit than an eight-plus-core desktop doing the same).
    inline int reservedCoresFor (unsigned int hardwareConcurrency) noexcept
    {
        return hardwareConcurrency <= 4 ? 2 : 1;
    }

    // Lowers the CALLING thread's own OS scheduling priority - used as the
    // onWorkerThreadStart hook passed into ParametricFIR.h's parallel
    // search (see run()), so every spawned attemptDesign() worker thread
    // starts by deprioritising itself before doing any LP-solving work.
    // This is a backstop underneath the dynamic worker-count throttling in
    // currentAllowedSearchConcurrency() (which is what keeps these threads
    // from being greedy in the first place) - it's what keeps the OS
    // scheduler siding with the real-time audio thread even in the moment
    // they do end up contending for the same core, e.g. between one
    // round's load poll and the next. THREAD_PRIORITY_LOWEST rather than
    // _IDLE: idle-priority threads only run when NOTHING else on the
    // system wants the CPU at all, which risks a search visibly stalling
    // any time the user is doing anything else with the machine at
    // all - lowest still yields to genuinely real-time/normal-priority
    // work but keeps making steady progress otherwise.
    inline void lowerCurrentThreadPriorityForSearchWorker() noexcept
    {
#ifdef _WIN32
        SetThreadPriority (GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif
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
    parameters.addParameterListener ("tapCountAuto", &paramListener);
    parameters.addParameterListener ("manualTapCount", &paramListener);

    // Loaded once here, not per-lookup - see userOverrides' own comment in
    // PluginProcessor.h.
    userOverrides = bbk::detachedpole::useroverrides::loadAll();
    searchCache = bbk::detachedpole::searchcache::loadAll();

    // Priority::low, not the default (normal): this thread only ever runs
    // a background redesign, never anything the user is waiting on
    // synchronously (see the class's own top-of-file comment on why a
    // redesign never blocks the host), so it should always lose CPU
    // arbitration to the real host/audio threads if they ever genuinely
    // contend for the same core - a first line of defence underneath the
    // dynamic worker-count throttling in currentAllowedSearchConcurrency(),
    // not a replacement for it (that throttling is what keeps this thread
    // and its own spawned workers from being greedy in the first place;
    // this is what keeps the OS scheduler on the audio thread's side even
    // if they do end up contending).
    startThread (juce::Thread::Priority::low);
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

    // Custom-mode search safety net (see run()): once designParametricFIR()'s
    // own patience/early-stop logic was removed entirely (it now always
    // extends toward maxTapCount, stopping only on demand via the Stop
    // button or this deadline - see ParametricFIR.h's own comment), some
    // upper bound on how long an unattended search can run is still needed
    // in case the user simply walks away and forgets it's running. This is
    // that bound, in whatever unit maxSearchTimeIsHours below selects - NOT
    // a target (a search that's still improving keeps going the whole time
    // regardless) - and it's user-adjustable rather than a fixed constant
    // precisely because "5 minutes" won't be the right margin for every
    // spec or every machine. 0.1 minute (6s) floor rather than 0 so this
    // can never be accidentally set to "stop immediately"; 999 ceiling is
    // generous headroom (with Hours on, up to 999 hours) without allowing
    // an literally-unbounded value. Default 5.0 (minutes, see
    // maxSearchTimeIsHours's own default) per direct request.
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "maxSearchTimeValue", 1 },
        "Max Search Time",
        juce::NormalisableRange<float> (0.1f, 999.0f, 0.1f),
        5.0f));

    // Off (default): maxSearchTimeValue above is minutes (so the default
    // operating point is a plain "5 minutes"). On: maxSearchTimeValue is
    // hours instead, for the rare demanding spec/slow machine combination
    // where even a generous minutes-scale budget isn't enough and the user
    // wants to deliberately let a search run unattended for a long time.
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "maxSearchTimeIsHours", 1 },
        "Max Search Time (Hours)",
        false));

    // Per-candidate search deadline (see run(): passed straight through as
    // designParametricFIR()/designParametricFIRFixedM()'s perCandidateSeconds).
    // Distinct from Max Search Time above, which bounds the WHOLE sweep
    // across every tap count tried - this instead bounds how long a SINGLE
    // tap count's own grid-refinement loop may run before being cut off,
    // win or lose, and moving on to the next candidate. That loop's own
    // stopping condition is wall-clock time, not a fixed round count (see
    // attemptDesign()'s own comment in ParametricFIR.h), so how well a
    // demanding candidate actually converges - and therefore which tap
    // count the search ultimately settles on as "best" - can vary run to
    // run purely from how much real CPU throughput that candidate's window
    // happened to get (contention from other concurrently-solving
    // candidates, or anything else busy on the machine at that moment).
    // Raising this gives every candidate more real room to fully converge
    // before being cut off, at the cost of covering fewer distinct tap
    // counts within the same overall Max Search Time budget if several
    // candidates in a row are all demanding - reported and requested
    // directly, after exactly this run-to-run variance was traced to this
    // cap. 5s floor keeps a pathologically low value from making every
    // candidate look infeasible; 300s (5 minutes) ceiling is generous for
    // even a very demanding single candidate without letting one candidate
    // alone consume an entire default-length Max Search Time budget by
    // itself. Default 120 (raised from a prior fixed, non-adjustable 60s
    // constant) - still comfortably below the 300s default overall budget.
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "perCandidateSearchTimeSeconds", 1 },
        "Per-Candidate Search Time",
        juce::NormalisableRange<float> (5.0f, 300.0f, 1.0f),
        120.0f,
        juce::AudioParameterFloatAttributes().withLabel ("s")));

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

void BBKDetachedPoleAudioProcessor::selectTopCandidate (int index)
{
    // All of this reads straight out of the already-published uiSnapshot -
    // every entry in topCandidates is already a fully-designed, spec-
    // compliant filter from the same completed search (see ParametricFIR.h),
    // so switching to a different one is just a re-publish of already-known
    // taps, exactly as cheap as an instant cache/override/bank hit; nothing
    // here touches the background search thread at all.
    bbk::parametric::FilterSpec spec;
    bbk::parametric::DesignResult result;
    ResultSource source;
    {
        const juce::SpinLock::ScopedLockType sl (uiSnapshotLock);
        if (index < 0 || index >= static_cast<int> (uiSnapshot.topCandidates.size()))
            return; // out of range, or this design has no ranked list at all (Manual mode/bank entry)

        spec.sampleRateHz = uiSnapshot.sampleRateHz;
        spec.cutoffHz = uiSnapshot.cutoffHz;
        spec.attenuationAtCutoffDb = uiSnapshot.attenuationAtCutoffDb;
        spec.stopbandRejectionDb = uiSnapshot.stopbandRejectionDb;
        spec.stopbandMode = uiSnapshot.stopbandMode;
        spec.sidelobeDecayRatio = uiSnapshot.sidelobeDecayRatio;
        source = uiSnapshot.source;

        const auto& chosen = uiSnapshot.topCandidates[static_cast<std::size_t> (index)];
        result.taps = chosen.taps;
        result.tapCount = chosen.tapCount;
        result.constraintsMet = true;
        result.achievedStopbandDb = chosen.achievedStopbandDb;
        result.designAttempts = uiSnapshot.designAttempts;
        result.temporal = bbk::parametric::computeTemporalMetrics (chosen.taps, spec.sampleRateHz);
        result.topCandidates = uiSnapshot.topCandidates; // keep the full list around for the next switch
    }

    publishResult (spec, result, source, index);
}

namespace
{
    // Shared by saveTopCandidateAsOverride()/savePresetSlot() below: builds
    // the OverrideEntry that whatever is currently published (a live
    // search result, a preset/bank entry, or an existing override) becomes
    // when saved. Reads the UI snapshot rather than latestResult/latestSpec
    // directly since that's already the single point guaranteed consistent
    // (spec and result written together under uiSnapshotLock in
    // publishResult()). Returns an empty candidates list if nothing has
    // been designed yet (snap.tapCount <= 0) - callers check that and no-op.
    bbk::detachedpole::useroverrides::OverrideEntry buildOverrideEntryFromSnapshot (
        const BBKDetachedPoleAudioProcessor::DesignSnapshot& snap, int chosenIndex)
    {
        bbk::detachedpole::useroverrides::OverrideEntry entry;
        if (snap.tapCount <= 0 || snap.taps.empty())
            return entry;

        entry.spec.sampleRateHz = snap.sampleRateHz;
        entry.spec.cutoffHz = snap.cutoffHz;
        entry.spec.attenuationAtCutoffDb = snap.attenuationAtCutoffDb;
        entry.spec.stopbandRejectionDb = snap.stopbandRejectionDb;
        entry.spec.stopbandMode = snap.stopbandMode;
        entry.spec.sidelobeDecayRatio = snap.sidelobeDecayRatio;

        // Saves the WHOLE ranked list (see DesignSnapshot::topCandidates),
        // not just the one candidate the user picked - see OverrideEntry::
        // activeIndex's own comment for why: this is what lets the
        // editor's top-N table still offer every alternative later,
        // exactly as if the search had just finished again, while still
        // instantly recalling the one the user actually chose. Manual-mode
        // results and preset/bank entries have no ranked list at all
        // (topCandidates is empty by design - see ParametricFIR.h/
        // DesignSnapshot's own comments), so those fall back to wrapping
        // the single currently-playing design as a one-entry list, same
        // shape UserPresetOverrides.h's own pre-top-N file upgrade path
        // already produces.
        if (! snap.topCandidates.empty())
        {
            entry.candidates = snap.topCandidates;
            entry.activeIndex = juce::jlimit (0, static_cast<int> (entry.candidates.size()) - 1, chosenIndex);
        }
        else
        {
            bbk::parametric::RankedCandidate c;
            c.taps = snap.taps;
            c.tapCount = snap.tapCount;
            c.achievedStopbandDb = snap.achievedStopbandDb;
            entry.candidates.push_back (std::move (c));
            entry.activeIndex = 0;
        }
        return entry;
    }
}

void BBKDetachedPoleAudioProcessor::saveTopCandidateAsOverride (int index)
{
    const auto snap = getDesignSnapshotForUI();
    auto entry = buildOverrideEntryFromSnapshot (snap, index);
    if (entry.candidates.empty())
        return; // nothing designed yet - see buildOverrideEntryFromSnapshot()'s own comment

    {
        const juce::SpinLock::ScopedLockType sl (specLock);

        // Multiple entries CAN legitimately share the same exact spec now -
        // two different top-N candidates from the very same Auto-mode
        // search (same cutoff/attenuation/stopband/decay, different
        // tap-count/R_peak) saved into two different preset slots is
        // exactly that case (see savePresetSlot()'s own comment on why).
        // This plain top-N-row "Save" button isn't a deliberate decision
        // about preset slots either way, so its own effect on presetSlot
        // stays conservative:
        //   - exactly one existing entry for this spec, and it's already a
        //     numbered preset - the old, unambiguous single-preset-per-spec
        //     case (e.g. re-saving the same find that already occupies a
        //     slot) - preserve that slot assignment, same as before.
        //   - anything else (no entries yet, or this spec already has
        //     MULTIPLE presets saved against it) - this becomes a plain
        //     (-1), non-preset bookmark, and every numbered-slot entry for
        //     this spec is left completely untouched: this button must
        //     never silently destroy someone else's preset just because it
        //     happens to share a spec.
        std::vector<std::size_t> matchIdx;
        for (std::size_t i = 0; i < userOverrides.size(); ++i)
            if (specsEqual (userOverrides[i].spec, entry.spec))
                matchIdx.push_back (i);

        entry.presetSlot = (matchIdx.size() == 1) ? userOverrides[matchIdx[0]].presetSlot : -1;

        // Erase only what this save actually supersedes: the lone prior
        // match in the unambiguous case above, or (when multiple entries
        // share this spec) only a plain entry among them - any numbered
        // preset survives even when it shares this spec.
        userOverrides.erase (std::remove_if (userOverrides.begin(), userOverrides.end(),
                                              [&] (const auto& e)
                                              {
                                                  if (! specsEqual (e.spec, entry.spec))
                                                      return false;
                                                  return matchIdx.size() == 1 || e.presetSlot == -1;
                                              }),
                              userOverrides.end());
        userOverrides.push_back (entry);
    }

    std::vector<bbk::detachedpole::useroverrides::OverrideEntry> toSave;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        toSave = userOverrides;
    }
    bbk::detachedpole::useroverrides::saveAll (toSave);

    // Reflect the save immediately in the UI as a UserOverride result -
    // the spec hasn't changed, so requestBoundaryRedesign()'s own dedup
    // guard would otherwise no-op this and leave the "Design method:"
    // label saying Custom even though a saved override now exists for this
    // exact spec.
    const auto& active = entry.candidates[static_cast<std::size_t> (entry.activeIndex)];
    bbk::parametric::DesignResult result;
    result.taps = active.taps;
    result.tapCount = active.tapCount;
    result.constraintsMet = true;
    result.achievedStopbandDb = active.achievedStopbandDb;
    result.designAttempts = 0;
    result.temporal = bbk::parametric::computeTemporalMetrics (result.taps, entry.spec.sampleRateHz);
    result.topCandidates = entry.candidates;

    publishResult (entry.spec, result, ResultSource::UserOverride, entry.activeIndex);
}

BBKDetachedPoleAudioProcessor::PresetSlotInfo BBKDetachedPoleAudioProcessor::getPresetSlotInfoForUI (int slot) const
{
    PresetSlotInfo info;
    if (slot < 0 || slot >= bbk::detachedpole::useroverrides::numPresetSlots)
        return info;

    const juce::SpinLock::ScopedLockType sl (specLock);
    for (auto& e : userOverrides)
    {
        if (e.presetSlot == slot)
        {
            info.occupied = true;
            info.spec = e.spec;

            // e.activeIndex is guarded on load (see UserPresetOverrides.h::
            // loadAll()), but re-check here too rather than trust that
            // every construction path did - same defensive posture as the
            // range check on slot itself just above.
            if (e.activeIndex >= 0 && e.activeIndex < static_cast<int> (e.candidates.size()))
            {
                const auto& active = e.candidates[static_cast<std::size_t> (e.activeIndex)];
                info.tapCount = active.tapCount;
                info.achievedStopbandDb = active.achievedStopbandDb;
                info.taps = active.taps;
            }
            break;
        }
    }
    return info;
}

void BBKDetachedPoleAudioProcessor::loadPresetSlot (int slot)
{
    using namespace bbk::detachedpole::presetbank;

    if (slot < 0 || slot >= bbk::detachedpole::useroverrides::numPresetSlots)
        return;

    bbk::parametric::FilterSpec targetSpec;
    bool found = false;
    bbk::detachedpole::useroverrides::OverrideEntry targetEntry;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        for (auto& e : userOverrides)
        {
            if (e.presetSlot == slot)
            {
                targetSpec = e.spec;
                targetEntry = e;
                found = true;
                break;
            }
        }
    }

    if (! found)
    {
        // Only slot 0 (Preset 1) has a built-in fallback - see this
        // method's own header comment. Every other empty slot is simply
        // nothing to load yet.
        if (slot != 0)
            return;

        targetSpec = specFromParameters();
        targetSpec.cutoffHz = presetCutoffHz;
        targetSpec.stopbandRejectionDb = presetStopbandRejectionDb;
        targetSpec.sidelobeDecayRatio = presetSidelobeDecayRatio;
        // Attenuation deliberately left as whatever specFromParameters()
        // already read - see this method's own header comment.
    }

    // Force Manual Tap Count off (Auto on) FIRST, before the four
    // operating-point parameters below - so every nested
    // requestBoundaryRedesign() call this triggers (each
    // setValueNotifyingHost re-enters the ParamListener) already sees Auto
    // mode in effect, not just the very last one.
    if (auto* tapCountAutoParam = parameters.getParameter ("tapCountAuto"))
        tapCountAutoParam->setValueNotifyingHost (1.0f);

    if (auto* cutoffParam = parameters.getParameter ("cutoff"))
        cutoffParam->setValueNotifyingHost (cutoffParam->convertTo0to1 (static_cast<float> (targetSpec.cutoffHz)));
    if (auto* attenuationParam = parameters.getParameter ("attenuation"))
        attenuationParam->setValueNotifyingHost (attenuationParam->convertTo0to1 (static_cast<float> (targetSpec.attenuationAtCutoffDb)));
    if (auto* stopbandParam = parameters.getParameter ("stopband"))
        stopbandParam->setValueNotifyingHost (stopbandParam->convertTo0to1 (static_cast<float> (targetSpec.stopbandRejectionDb)));
    if (auto* decayParam = parameters.getParameter ("sidelobeDecay"))
        decayParam->setValueNotifyingHost (decayParam->convertTo0to1 (static_cast<float> (targetSpec.sidelobeDecayRatio)));

    // The four setValueNotifyingHost calls above each re-enter
    // requestBoundaryRedesign() via the ParamListener, which finds an
    // instant result for targetSpec by matching userOverrides on SPEC
    // ALONE (see its own comment there) - ambiguous whenever more than one
    // preset slot shares this exact spec, which is now an expected,
    // supported case (two different top-N candidates from the same Auto
    // search saved into two different slots - see savePresetSlot()'s own
    // comment on why that's allowed). So publish THIS slot's own entry
    // explicitly, as the final authoritative step, rather than trusting
    // whichever entry that generic spec-based lookup last happened to
    // settle on - the same "explicitly publish, don't rely on the generic
    // redesign path" pattern savePresetSlot()/saveTopCandidateAsOverride()
    // already use for their own immediate-UI-feedback publish. A harmless
    // republish of the same taps when this spec is unambiguous (the common
    // case); a no-op for slot 0's factory-bank fallback (found is false
    // there, nothing of our own to republish over the bank's own result).
    if (found && targetEntry.activeIndex >= 0
        && targetEntry.activeIndex < static_cast<int> (targetEntry.candidates.size()))
    {
        const auto& active = targetEntry.candidates[static_cast<std::size_t> (targetEntry.activeIndex)];
        bbk::parametric::DesignResult result;
        result.taps = active.taps;
        result.tapCount = active.tapCount;
        result.constraintsMet = true;
        result.achievedStopbandDb = active.achievedStopbandDb;
        result.designAttempts = 0;
        result.temporal = bbk::parametric::computeTemporalMetrics (result.taps, targetEntry.spec.sampleRateHz);
        result.topCandidates = targetEntry.candidates;

        publishResult (targetEntry.spec, result, ResultSource::UserOverride, targetEntry.activeIndex);
    }
}

void BBKDetachedPoleAudioProcessor::savePresetSlot (int slot)
{
    if (slot < 0 || slot >= bbk::detachedpole::useroverrides::numPresetSlots)
        return;

    const auto snap = getDesignSnapshotForUI();
    auto entry = buildOverrideEntryFromSnapshot (snap, snap.selectedIndex);
    if (entry.candidates.empty())
        return; // nothing designed yet

    entry.presetSlot = slot;

    {
        const juce::SpinLock::ScopedLockType sl (specLock);

        // Two different top-N candidates from the very same Auto-mode
        // search share the identical FilterSpec (only their chosen
        // candidate/activeIndex differs, see RankedCandidate's own comment
        // in ParametricFIR.h) - saving both into two different preset slots
        // is exactly what this method is for, so multiple entries ARE
        // allowed to share one spec now, as long as each belongs to a
        // different slot. Reported directly: saving a second find into
        // Preset 2 right after saving a first into Preset 1 from the same
        // search silently emptied Preset 1 again, because the old dedup
        // rule erased EVERY entry matching this spec regardless of which
        // slot (if any) it belonged to. What must still never happen is
        // two entries claiming the SAME slot, or a stray duplicate plain
        // (non-preset) entry for a spec a preset already covers - so only
        // THOSE are cleared here:
        for (auto& e : userOverrides)
            if (e.presetSlot == slot)
                e.presetSlot = -1; // free whichever entry currently occupies THIS slot (if any) - it stays around as a plain, exact-spec-recall override, it just stops being a preset, same as before

        userOverrides.erase (std::remove_if (userOverrides.begin(), userOverrides.end(),
                                              [&] (const auto& e)
                                              {
                                                  // The just-freed former occupant of this slot (now
                                                  // plain), or any OTHER pre-existing plain entry for
                                                  // this exact spec - never an entry still tagged to a
                                                  // DIFFERENT preset slot, even one sharing this spec.
                                                  return e.presetSlot == -1 && specsEqual (e.spec, entry.spec);
                                              }),
                              userOverrides.end());
        userOverrides.push_back (entry);
    }

    std::vector<bbk::detachedpole::useroverrides::OverrideEntry> toSave;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        toSave = userOverrides;
    }
    bbk::detachedpole::useroverrides::saveAll (toSave);

    // Reflect the save immediately in the UI, same reasoning as
    // saveTopCandidateAsOverride().
    const auto& active = entry.candidates[static_cast<std::size_t> (entry.activeIndex)];
    bbk::parametric::DesignResult result;
    result.taps = active.taps;
    result.tapCount = active.tapCount;
    result.constraintsMet = true;
    result.achievedStopbandDb = active.achievedStopbandDb;
    result.designAttempts = 0;
    result.temporal = bbk::parametric::computeTemporalMetrics (result.taps, entry.spec.sampleRateHz);
    result.topCandidates = entry.candidates;

    publishResult (entry.spec, result, ResultSource::UserOverride, entry.activeIndex);
}

void BBKDetachedPoleAudioProcessor::exportPresets (const juce::File& destFile)
{
    std::vector<bbk::detachedpole::useroverrides::OverrideEntry> toExport;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        for (auto& e : userOverrides)
            if (e.presetSlot >= 0)
                toExport.push_back (e);
    }
    bbk::detachedpole::useroverrides::saveAll (toExport, destFile);
}

bool BBKDetachedPoleAudioProcessor::importPresets (const juce::File& srcFile)
{
    auto imported = bbk::detachedpole::useroverrides::loadAll (srcFile);

    bool mergedAny = false;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        for (auto& incoming : imported)
        {
            if (incoming.presetSlot < 0)
                continue; // a plain override in the imported file, not tagged as a preset - nothing to merge

            for (auto& e : userOverrides)
                if (e.presetSlot == incoming.presetSlot)
                    e.presetSlot = -1; // this slot is about to be replaced - see savePresetSlot()'s own comment

            userOverrides.erase (std::remove_if (userOverrides.begin(), userOverrides.end(),
                                                  [&] (const auto& e) { return specsEqual (e.spec, incoming.spec); }),
                                  userOverrides.end());
            userOverrides.push_back (incoming);
            mergedAny = true;
        }
    }

    if (! mergedAny)
        return false;

    std::vector<bbk::detachedpole::useroverrides::OverrideEntry> toSave;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        toSave = userOverrides;
    }
    return bbk::detachedpole::useroverrides::saveAll (toSave);
}

void BBKDetachedPoleAudioProcessor::requestFreshSearch()
{
    // Forces a genuinely new live search for the current spec, bypassing
    // the bank/override/cache instant lookups that requestBoundaryRedesign()
    // itself would otherwise hit - see this method's own header comment for
    // why that's the whole point of a dedicated Re-search action rather
    // than just calling requestBoundaryRedesign() again (which would find
    // the very cache entry this is trying to replace and instantly no-op).
    if (! hasPrepared.load() || currentSampleRate.load() <= 0.0)
        return;

    const auto spec = specFromParameters();
    const bool manualTapCountOn = parameters.getRawParameterValue ("tapCountAuto")->load() <= 0.5f;
    const int requestedTapCount = static_cast<int> (parameters.getRawParameterValue ("manualTapCount")->load());

    {
        const juce::SpinLock::ScopedLockType sl (specLock);

        currentBoundarySpec = spec;
        currentBoundaryManualTapCountOn = manualTapCountOn;
        currentBoundaryRequestedTapCount = requestedTapCount;
        ++boundaryEpoch; // discards anything already queued/mid-flight, same as requestBoundaryRedesign()
        taskQueue.clear();

        DesignTask task;
        task.spec = spec;
        task.epoch = boundaryEpoch;
        task.manualTapCount = manualTapCountOn;
        task.requestedTapCount = requestedTapCount;
        taskQueue.push_back (task);

        searchInProgress.store (true);
    }

    stopSearchRequested.store (false);
    {
        const juce::SpinLock::ScopedLockType sl (searchProgressLock);
        searchProgress = SearchProgressSnapshot {};
    }

    notify();
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

    // Read early: needed both for the "did anything actually change" dedup
    // check below and for the factory-bank lookup and the else branch
    // further down (search-cache gating and the queued DesignTask itself) -
    // see requestedTapCount's own comment on why it only matters while
    // manualTapCountOn is true.
    const bool manualTapCountOn = parameters.getRawParameterValue ("tapCountAuto")->load() <= 0.5f;
    const int requestedTapCount = static_cast<int> (parameters.getRawParameterValue ("manualTapCount")->load());

    // Factory bank: an always-available instant lookup, exactly like
    // userOverrides/searchCache below - no "Default mode" toggle needed to
    // gate it, since the bank is itself just a compiled cache of designs at
    // ONE fixed (cutoff, stopband, decay) operating point across many
    // (sample rate, attenuation) combinations. It's only a valid hit when
    // cutoff/stopband/decay are already sitting exactly AT that fixed
    // point - findEntry() itself only varies sample rate and attenuation,
    // so checking it without this guard would wrongly serve the bank's
    // 18.5 kHz filter for, say, a 12 kHz Custom-mode request that happens
    // to land on one of the swept attenuation steps. Also skipped entirely
    // while Manual tap-count mode is on - same reasoning as the
    // userOverrides/searchCache guards further down: an instant bank hit
    // carries whatever tap count the bank's own sweep landed on, which
    // must never silently override a tap count the user explicitly dialed
    // in here. Looked up here (before the lock below) since it only reads
    // static table data - no need to hold specLock for it. The bank always
    // matches spec.attenuationAtCutoffDb, not the raw slider value:
    // amplitude relaxation off substitutes caseBNearFlatAttenuationDb (well
    // outside the swept 0.05-0.50 dB grid), so relaxation-off specs
    // correctly find no entry and fall back to a live design below, rather
    // than silently returning some unrelated preset. This is also what
    // backs an unassigned Preset 1's fallback - see loadPresetSlot().
    const bool atPresetOperatingPoint = ! manualTapCountOn
        && spec.cutoffHz == bbk::detachedpole::presetbank::presetCutoffHz
        && spec.stopbandRejectionDb == bbk::detachedpole::presetbank::presetStopbandRejectionDb
        && spec.sidelobeDecayRatio == bbk::detachedpole::presetbank::presetSidelobeDecayRatio;
    const auto* presetEntry = atPresetOperatingPoint
        ? bbk::detachedpole::presetbank::findEntry (spec.sampleRateHz, spec.attenuationAtCutoffDb)
        : nullptr;

    bool haveInstantResult = false;
    bbk::parametric::DesignResult instantResult;
    ResultSource instantSource = ResultSource::LiveSearch;
    int instantSelectedIndex = 0;

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
        // There's no longer a separate "mode" to compare here (the factory
        // bank lookup above is now purely a function of spec itself - see
        // atPresetOperatingPoint's own comment), so specsEqual alone fully
        // determines whether the outcome would differ.
        // manualTapCountOn is compared unconditionally (Auto->Manual or
        // Manual->Auto must always force a fresh redesign), but
        // requestedTapCount only when manualTapCountOn is true - comparing
        // it unconditionally would force a spurious redesign on every mouse
        // nudge of the manual slider even while Auto mode is on and
        // ignoring it entirely.
        if (specsEqual (spec, currentBoundarySpec)
            && manualTapCountOn == currentBoundaryManualTapCountOn
            && (! manualTapCountOn || requestedTapCount == currentBoundaryRequestedTapCount)
            && boundaryEpoch != 0)
            return;

        currentBoundarySpec = spec;
        currentBoundaryManualTapCountOn = manualTapCountOn;
        currentBoundaryRequestedTapCount = requestedTapCount;
        ++boundaryEpoch;
        taskQueue.clear(); // also discards any now-stale in-flight live design

        // User overrides win ahead of the factory bank (checked even when
        // presetEntry above is null, i.e. not at the bank's fixed operating
        // point, or an untabled sample rate) - see saveTopCandidateAsOverride()/
        // savePresetSlot()'s own comments. loadPresetSlot() forces cutoff/
        // attenuation/stopband/decay to match a saved preset's own spec,
        // which re-enters here as an ordinary boundary change - this lookup
        // is what serves an instant hit for it, same as for a manually
        // dialled-in spec that happens to match a saved override.
        //
        // This lookup matches on SPEC ALONE, so it's ambiguous whenever more
        // than one entry shares this exact spec - an expected, supported
        // case now (two different top-N candidates from the same Auto
        // search, saved into two different preset slots - see
        // savePresetSlot()'s own comment on why). The first match found
        // (typically the lowest-numbered preset slot, or a plain override if
        // one exists) is what a manually dialled-in spec recalls; that's an
        // acceptable ambiguity for that path, but loadPresetSlot() itself
        // does NOT rely on this lookup picking the right one - it captures
        // its own slot's entry directly and republishes it explicitly as
        // the authoritative last step (see its own comment), so pressing a
        // specific numbered Load button always recalls that exact slot
        // regardless of what this lookup alone would have found.
        // userOverrides is guarded by this same specLock (see its
        // declaration in the header) since this function, like the rest of
        // the specLock-guarded block, can run on the audio thread.
        //
        // Skipped while Manual tap-count mode is on - same reasoning as the
        // search-cache guard further down (see its own comment): a saved
        // override may hold a different tap count than the one explicitly
        // dialed in here (most commonly whatever Auto's own quality search
        // picked at the time it was saved), so serving it would silently
        // ignore the user's manual request exactly the way an unguarded
        // cache hit would. Bug reported directly: entering a fresh Custom
        // spec with Manual Tap Count set to 19 still instantly loaded an
        // existing 29-tap override for that spec instead of running a
        // fresh fixed-19-tap search - this guard is the fix. Saving a NEW
        // override (via a top-N row's "Save" button, or savePresetSlot())
        // is unaffected - that stays a deliberate, explicit action
        // regardless of Manual/Auto mode; only this automatic lookup is
        // gated.
        const bbk::detachedpole::useroverrides::OverrideEntry* overrideEntry = nullptr;
        if (! manualTapCountOn)
        {
            for (auto& e : userOverrides)
            {
                if (specsEqual (e.spec, spec))
                {
                    overrideEntry = &e;
                    break;
                }
            }
        }

        if (overrideEntry != nullptr)
        {
            haveInstantResult = true;
            instantSource = ResultSource::UserOverride;
            instantSelectedIndex = overrideEntry->activeIndex;
            const auto& active = overrideEntry->candidates[static_cast<std::size_t> (overrideEntry->activeIndex)];
            instantResult.taps = active.taps;
            instantResult.tapCount = active.tapCount;
            instantResult.constraintsMet = true;
            instantResult.achievedStopbandDb = active.achievedStopbandDb;
            instantResult.designAttempts = 0;
            instantResult.temporal = bbk::parametric::computeTemporalMetrics (instantResult.taps, spec.sampleRateHz);
            instantResult.topCandidates = overrideEntry->candidates;
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
            // one - see LiveSearchCache.h. Checked unconditionally, same
            // reasoning as userOverrides: costs nothing to check, and
            // covers a spec at the bank's own fixed operating point but on
            // an untabled sample rate too.
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
                instantSelectedIndex = 0; // a cache hit is never a deliberate user choice - see CacheEntry's own comment
                const auto& top = cacheEntry->candidates[0];
                instantResult.taps = top.taps;
                instantResult.tapCount = top.tapCount;
                instantResult.constraintsMet = true;
                instantResult.achievedStopbandDb = top.achievedStopbandDb;
                instantResult.designAttempts = 0;
                instantResult.temporal = bbk::parametric::computeTemporalMetrics (instantResult.taps, spec.sampleRateHz);
                instantResult.topCandidates = cacheEntry->candidates;
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
        publishResult (spec, instantResult, instantSource, instantSelectedIndex);

        // An instant result means there is nothing left to wait for, for
        // THIS boundary change - see isSearchInProgressForUI()'s own
        // comment. If an older, now-superseded background task happens to
        // still be running at this exact moment, its own staleness check
        // in run() will simply discard its result without touching this
        // flag once it finishes, so this assignment is never clobbered
        // late by that stale task.
        searchInProgress.store (false);
        stopSearchRequested.store (false);
        {
            const juce::SpinLock::ScopedLockType sl (searchProgressLock);
            searchProgress = SearchProgressSnapshot {};
        }
    }
    else
        notify();
}

void BBKDetachedPoleAudioProcessor::publishResult (const bbk::parametric::FilterSpec& spec,
                                                     const bbk::parametric::DesignResult& result,
                                                     ResultSource source,
                                                     int selectedIndex)
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
        uiSnapshot.topCandidates = result.topCandidates;
        uiSnapshot.selectedIndex = selectedIndex;
    }
}

double BBKDetachedPoleAudioProcessor::pollSystemCpuBusyFraction()
{
#ifdef _WIN32
    FILETIME idleFt {}, kernelFt {}, userFt {};
    if (! GetSystemTimes (&idleFt, &kernelFt, &userFt))
        return 0.0; // call failed - treat as "assume idle" rather than throttle on bad data

    const auto toTicks = [] (const FILETIME& ft) -> std::uint64_t
    {
        ULARGE_INTEGER u;
        u.LowPart = ft.dwLowDateTime;
        u.HighPart = ft.dwHighDateTime;
        return u.QuadPart;
    };

    const std::uint64_t idle = toTicks (idleFt);
    const std::uint64_t kernel = toTicks (kernelFt); // includes idle, per GetSystemTimes' own documented contract
    const std::uint64_t user = toTicks (userFt);

    if (! haveSystemCpuSample)
    {
        lastSystemIdleTicks = idle;
        lastSystemKernelTicks = kernel;
        lastSystemUserTicks = user;
        haveSystemCpuSample = true;
        return 0.0; // no previous sample yet to diff against
    }

    // All three counters are monotonically increasing system uptime
    // counters, so a smaller-than-previous reading only happens if the
    // underlying counter wrapped (not realistically reachable at 100ns
    // resolution within a session) - guard it anyway rather than risk a
    // huge bogus fraction from an unsigned underflow.
    const std::uint64_t idleDelta = idle > lastSystemIdleTicks ? idle - lastSystemIdleTicks : 0;
    const std::uint64_t kernelDelta = kernel > lastSystemKernelTicks ? kernel - lastSystemKernelTicks : 0;
    const std::uint64_t userDelta = user > lastSystemUserTicks ? user - lastSystemUserTicks : 0;

    lastSystemIdleTicks = idle;
    lastSystemKernelTicks = kernel;
    lastSystemUserTicks = user;

    const std::uint64_t totalDelta = kernelDelta + userDelta;
    if (totalDelta == 0)
        return 0.0; // no measurable time elapsed between polls

    const std::uint64_t busyDelta = totalDelta > idleDelta ? totalDelta - idleDelta : 0;
    return static_cast<double> (busyDelta) / static_cast<double> (totalDelta);
#else
    return 0.0;
#endif
}

int BBKDetachedPoleAudioProcessor::currentAllowedSearchConcurrency()
{
    const unsigned int hw = std::thread::hardware_concurrency();
    // hardware_concurrency() is documented as "may return 0 if not
    // computable" - fall back to a conservative single-core assumption
    // rather than letting the reserved-cores subtraction below go negative.
    const int hwCores = hw > 0 ? static_cast<int> (hw) : 1;

    const int reserved = reservedCoresFor (hw);
    int allowed = std::max (1, std::min (maxSearchWorkerThreads, hwCores - reserved));

    // System-wide CPU headroom: tiered rather than a smooth scale, since a
    // smooth scale would make this number jitter on every round from
    // ordinary noise in the underlying measurement - a search round only
    // needs a coarse "still plenty of room / getting tight / basically
    // none left" signal, not a precise one.
    const double systemBusy = pollSystemCpuBusyFraction();
    if (systemBusy >= 0.90)
        allowed = 1;
    else if (systemBusy >= 0.75)
        allowed = std::max (1, allowed / 4);
    else if (systemBusy >= 0.50)
        allowed = std::max (1, allowed / 2);

    // Audio callback headroom is the harder, more directly-relevant
    // constraint of the two (see its own comment in PluginProcessor.h) -
    // applied as a second, independent clamp on top of the system-load
    // result above, not blended with it, so a healthy-looking system-wide
    // number can never mask the audio thread itself actually struggling
    // (e.g. because something else is pinned to the same core the host's
    // audio thread happens to be scheduled on).
    const float audioLoad = audioCallbackLoadFraction.load();
    if (audioLoad >= 0.50f)
        allowed = 1;
    else if (audioLoad >= 0.25f)
        allowed = std::max (1, std::min (allowed, maxSearchWorkerThreads / 4));

    return std::max (1, allowed);
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

        // This is a genuinely new (non-stale) task about to start - reset
        // the Stop flag and the live-progress snapshot so neither one leaks
        // forward from whatever the PREVIOUS task left behind (a stale
        // stopSearchRequested left set from an earlier Stop click would
        // otherwise make this brand-new search stop on its very first
        // round; a stale searchProgress would show the editor a leftover
        // trial count/best-so-far from the last search for a moment before
        // this one's own onProgress callback below first fires).
        stopSearchRequested.store (false);
        {
            const juce::SpinLock::ScopedLockType sl (searchProgressLock);
            searchProgress = SearchProgressSnapshot {};
        }

        // Custom-mode's live search now has no patience limit of its own
        // (see ParametricFIR.h's own comment on why that was removed
        // outright): it always keeps extending toward maxTapCount looking
        // for a better result, stopping only when the user clicks Stop
        // (requestStopSearch(), wired to concurrency.shouldStopEarly below),
        // or when it hits this safety-net deadline - a plain backstop for
        // an unattended search, not a target, read live from the
        // "maxSearchTimeValue"/"maxSearchTimeIsHours" parameters (editor-
        // adjustable, 5 minutes by default) rather than a fixed constant,
        // per direct request. perCandidateDeadlineSeconds below (also
        // editor-adjustable now, default 120s, raised from a prior fixed
        // 60s constant) is the other half of that budget - see
        // "perCandidateSearchTimeSeconds"'s own comment in
        // createParameterLayout() for what it actually controls (how many
        // of a demanding candidate's own grid-refinement rounds get to
        // finish before ITS deadline cuts it off, not the overall sweep) -
        // safe to be this patient now that the factory bank and any saved
        // override/preset (see requestBoundaryRedesign()) give an
        // always-available instant result while this runs in the
        // background.
        //
        // Manual tap-count mode: skip the auto M-search entirely and design
        // fixed at exactly task.requestedTapCount, after silently raising it
        // to the spec's true feasible floor first via minimumFeasibleTapCount()
        // if it's below that (see its own comment in ParametricFIR.h for why
        // a request under that floor is never a useful result to show). Uses
        // the same safety-net deadline as the Auto path below, though in
        // practice it stops at the first feasible M and rarely needs it.
        //
        // concurrency: lets both searches try several candidate tap counts
        // at once instead of one at a time - see SearchConcurrencyHooks'
        // own comment in ParametricFIR.h for how this is guaranteed to
        // only ever change wall-clock time, never which M gets chosen (for
        // any design that converges within its own per-candidate time
        // budget - see currentAllowedSearchConcurrency()'s own comment on
        // the one narrow, already-precedented exception: a design that's
        // still not fully converged even at its overall deadline can, like
        // every other timing-sensitive knife-edge already documented in
        // this codebase - see caseBNearFlatAttenuationDb in
        // DetachedPoleFilter.h - land a hair differently depending on
        // exactly how much wall-clock compute each concurrent candidate
        // happened to get). pollConcurrency is re-evaluated once per
        // "round" (a fresh batch of candidates), so it backs off
        // immediately if system load or the audio callback's own headroom
        // gets tight mid-search. onWorkerThreadStart lowers each spawned
        // worker's own OS thread priority the same way the priority
        // passed to startThread() above does for this thread itself.
        // shouldStopEarly/onProgress back requestStopSearch()/
        // getSearchProgressForUI() - see PluginProcessor.h for the full
        // story on both.
        const float maxSearchTimeValue = parameters.getRawParameterValue ("maxSearchTimeValue")->load();
        const bool maxSearchTimeIsHours = parameters.getRawParameterValue ("maxSearchTimeIsHours")->load() > 0.5f;
        const double safetyDeadlineSeconds = static_cast<double> (maxSearchTimeValue) * (maxSearchTimeIsHours ? 3600.0 : 60.0);
        const double perCandidateDeadlineSeconds = static_cast<double> (parameters.getRawParameterValue ("perCandidateSearchTimeSeconds")->load());

        bbk::parametric::SearchConcurrencyHooks concurrency;
        concurrency.pollConcurrency = [this] { return currentAllowedSearchConcurrency(); };
        concurrency.onWorkerThreadStart = [] { lowerCurrentThreadPriorityForSearchWorker(); };

        // Also collapses (same as an explicit Stop click) the instant a
        // newer boundary change supersedes this task's own epoch - e.g.
        // loading a preset while a live search is still running for the
        // previous spec. Before this existed, a superseded task had NO way
        // to notice: it kept running invisibly (this same hook only ever
        // checked stopSearchRequested) all the way to completion or the
        // safety-net deadline, then got silently discarded by the
        // stillCurrent check below - reported directly as a boundary change
        // that lost the best found while an older search kept running
        // forward underneath it. Polled at the same once-per-round cadence as
        // pollConcurrency, so this is a cheap, brief specLock read, not a
        // hot-path cost. Paired with the cache-write-before-staleness-check
        // change just below: collapsing quickly here means there is less
        // work to have wasted, and caching whatever was found so far
        // regardless of staleness means that work is never simply thrown
        // away either way.
        concurrency.shouldStopEarly = [this, taskEpoch = task.epoch]
        {
            if (stopSearchRequested.load())
                return true;
            const juce::SpinLock::ScopedLockType sl (specLock);
            return taskEpoch != boundaryEpoch;
        };
        concurrency.onProgress = [this] (int attemptsSoFar, bool haveBest, int bestTapCount,
                                          double bestRPeakPercent, double bestAchievedStopbandDb,
                                          bool bestIsFeasible)
        {
            const juce::SpinLock::ScopedLockType sl (searchProgressLock);
            searchProgress.attemptsSoFar = attemptsSoFar;
            searchProgress.haveBest = haveBest;
            searchProgress.bestTapCount = bestTapCount;
            searchProgress.bestRPeakPercent = bestRPeakPercent;
            searchProgress.bestAchievedStopbandDb = bestAchievedStopbandDb;
            searchProgress.bestIsFeasible = bestIsFeasible;
        };

        bbk::parametric::DesignResult result;
        if (task.manualTapCount)
        {
            const int floor = bbk::parametric::minimumFeasibleTapCount (task.spec, bbk::detachedpole::maxTapCount, 30.0);
            const int target = juce::jmax (task.requestedTapCount, floor);
            result = bbk::parametric::designParametricFIRFixedM (task.spec, target, bbk::detachedpole::maxTapCount, safetyDeadlineSeconds, perCandidateDeadlineSeconds, concurrency);
        }
        else
        {
            result = bbk::parametric::designParametricFIR (task.spec, bbk::detachedpole::maxTapCount, safetyDeadlineSeconds, perCandidateDeadlineSeconds, concurrency);
        }

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
        //
        // Deliberately done BEFORE the staleness check below, not after -
        // this is the other half of the epoch-supersession fix described on
        // concurrency.shouldStopEarly above: a task that gets superseded
        // mid-flight (by a newer boundary change, e.g. loading a preset and
        // then dialing back to the original spec) used to just fall through
        // the old post-check
        // `continue` with nothing ever cached, forcing a full from-scratch
        // restart the moment the original spec came back. Caching
        // whatever was found - even the best-effort, not-fully-searched
        // result of a search that got cut short by the epoch-aware
        // shouldStopEarly above - means that restart is instead an instant
        // cache hit. This is exactly as safe as caching a "properly
        // finished" result: every cached entry already only ever means
        // "the best this engine found for this spec, within whatever time
        // it was given" (see run()'s own overall-deadline handling), not a
        // promise that the search ran to full completion.
        if (! task.manualTapCount && result.tapCount > 0 && ! result.taps.empty())
        {
            bbk::detachedpole::searchcache::CacheEntry entry;
            entry.spec = task.spec;

            // Auto mode already ranks its own top-N (see ParametricFIR.h's
            // designParametricFIR and DesignResult::topCandidates) - cache
            // exactly that list so a later exact-spec hit can offer the
            // same alternatives the search itself found, not just the
            // single winner. A Manual task never reaches this branch (see
            // the guard above), so the only other way result.topCandidates
            // could be empty here is a degenerate/never-actually-searched
            // result, already excluded by the tapCount/taps check above -
            // but guard it anyway rather than cache a hollow entry.
            std::vector<bbk::parametric::RankedCandidate> freshCandidates;
            if (! result.topCandidates.empty())
            {
                freshCandidates = result.topCandidates;
            }
            else
            {
                bbk::parametric::RankedCandidate c;
                c.taps = result.taps;
                c.tapCount = result.tapCount;
                c.achievedStopbandDb = result.achievedStopbandDb;
                freshCandidates.push_back (std::move (c));
            }

            std::vector<bbk::detachedpole::searchcache::CacheEntry> toSave;
            {
                const juce::SpinLock::ScopedLockType sl (specLock);

                // Merge with whatever is already cached for this exact
                // spec rather than blindly replacing it - see
                // mergeRankedCandidates()'s own comment above process().
                // This is what keeps requestFreshSearch() ("Re-search")
                // from being a strictly risky action: its own fresh sweep
                // can, for a demanding spec, genuinely not reach/converge
                // as far this run as an earlier, luckier one did (system
                // load changes how much of the tap-count range gets
                // covered before the deadline - see designParametricFIR's
                // own comment on concurrency timing sensitivity), and
                // without this merge that worse result would silently
                // overwrite a genuinely better filter already on file with
                // no way back to it - reported directly. A brand-new spec
                // with nothing cached yet is unaffected: merging against an
                // empty existing list is just the fresh list, unchanged.
                const auto existingIt = std::find_if (searchCache.begin(), searchCache.end(),
                                                       [&] (const auto& e) { return specsEqual (e.spec, entry.spec); });
                const std::vector<bbk::parametric::RankedCandidate> existingCandidates =
                    existingIt != searchCache.end() ? existingIt->candidates
                                                     : std::vector<bbk::parametric::RankedCandidate> {};
                entry.candidates = mergeRankedCandidates (existingCandidates, freshCandidates, entry.spec.sampleRateHz);

                searchCache.erase (std::remove_if (searchCache.begin(), searchCache.end(),
                                                    [&] (const auto& e) { return specsEqual (e.spec, entry.spec); }),
                                   searchCache.end());
                searchCache.push_back (entry);

                while (static_cast<int> (searchCache.size()) > bbk::detachedpole::searchcache::maxEntries)
                    searchCache.erase (searchCache.begin());

                toSave = searchCache;
            }
            bbk::detachedpole::searchcache::saveAll (toSave);

            // Publish the MERGED list, not just this run's own - otherwise
            // a Re-search that landed on something worse than what was
            // already cached would still show the worse result on screen
            // right now, even though the better one is (correctly, as of
            // the merge above) still what's cached for next time. If the
            // merge's own best (index 0) differs from what this run itself
            // found, adopt it here too, so what's shown immediately after
            // a search finishes always matches the best known answer for
            // this spec, not just the best THIS run happened to find.
            result.topCandidates = entry.candidates;
            if (! entry.candidates.empty())
            {
                const auto& winner = entry.candidates.front();
                result.taps = winner.taps;
                result.tapCount = winner.tapCount;
                result.achievedStopbandDb = winner.achievedStopbandDb;
                result.temporal = bbk::parametric::computeTemporalMetrics (winner.taps, entry.spec.sampleRateHz);
            }
        }

        // Re-check staleness after the (possibly slow - see
        // ParametricFIR.h for how thorough this search now is)
        // computation - a newer boundary change may have arrived while
        // this task was running. If so, the work is simply discarded, not
        // published - it has already been cached just above, though, so
        // "discarded" here only ever means "not shown right now", never
        // "gone".
        bool stillCurrent;
        {
            const juce::SpinLock::ScopedLockType sl (specLock);
            stillCurrent = (task.epoch == boundaryEpoch);
        }
        if (! stillCurrent)
            continue;

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

BBKDetachedPoleAudioProcessor::SearchProgressSnapshot BBKDetachedPoleAudioProcessor::getSearchProgressForUI() const
{
    const juce::SpinLock::ScopedLockType sl (searchProgressLock);
    return searchProgress;
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

    // Feeds audioCallbackLoadFraction below - see its own comment in
    // PluginProcessor.h for why this exists (throttling the parallel
    // background search) and why it has to be this cheap: one steady_clock
    // read now, one more at the very end of this function, a subtraction
    // and a division against this block's own time budget, and one atomic
    // store - on the order of tens of nanoseconds, immeasurably small next
    // to the convolution work this function is about to do either way.
    const auto blockStartTime = std::chrono::steady_clock::now();

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

    // See blockStartTime's own comment above. budgetSeconds is this
    // block's own real-time deadline - the host expects numSamples worth
    // of audio back within that long, regardless of what else is running
    // on the machine. Blended with a fast-attack/slow-release-ish EMA
    // (weighted toward the newest reading) rather than a plain running
    // average, so a genuine spike is reflected almost immediately - the
    // whole point is to catch the audio thread actually starting to
    // struggle before it audibly does - while a single unusually-fast or
    // -slow block doesn't on its own yank the search's thread count
    // around.
    {
        const auto sampleRate = currentSampleRate.load();
        if (sampleRate > 0.0 && numSamples > 0)
        {
            const auto elapsed = std::chrono::steady_clock::now() - blockStartTime;
            const double elapsedSeconds = std::chrono::duration<double> (elapsed).count();
            const double budgetSeconds = static_cast<double> (numSamples) / sampleRate;
            const float thisBlockLoad = static_cast<float> (std::min (4.0, elapsedSeconds / budgetSeconds));

            const float previous = audioCallbackLoadFraction.load();
            const float blended = thisBlockLoad > previous
                ? thisBlockLoad                              // instant on the way up (a spike matters immediately)
                : previous * 0.9f + thisBlockLoad * 0.1f;    // decay gradually on the way down
            audioCallbackLoadFraction.store (blended);
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
