#include "PluginProcessor.h"
#include "PluginEditor.h"

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

    // Same normalised peak-to-total-impulse-energy concentration Peak-
    // Energy Optimized reports for itself (see PeakEnergyFIR.h: "eta =
    // a[0]^2 / sum(h^2)") - but that's a property of any symmetric FIR's
    // taps, not something specific to how they were designed, so it can be
    // measured identically for Minimax and Prolate/DPSS Basis results too,
    // letting all three modes be compared on the same number.
    struct EtaMetrics { double eta = 0.0; double concentrationDb = -300.0; };

    inline EtaMetrics computeEtaMetrics (const std::vector<double>& taps)
    {
        if (taps.empty())
            return {};

        double sumSquares = 0.0;
        for (double t : taps)
            sumSquares += t * t;

        const auto centreIndex = static_cast<std::size_t> ((taps.size() - 1) / 2);
        const double centreTap = taps[centreIndex];

        EtaMetrics m;
        m.eta = (sumSquares > 0.0) ? (centreTap * centreTap) / sumSquares : 0.0;
        m.concentrationDb = (m.eta > 0.0) ? 10.0 * std::log10 (m.eta) : -300.0;
        return m;
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
    parameters.addParameterListener ("prolateBasis", &paramListener);
    parameters.addParameterListener ("sidelobeDecay", &paramListener);
    parameters.addParameterListener ("peakEnergyOptimized", &paramListener);

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
    parameters.removeParameterListener ("prolateBasis", &paramListener);
    parameters.removeParameterListener ("sidelobeDecay", &paramListener);
    parameters.removeParameterListener ("peakEnergyOptimized", &paramListener);

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

    // Off (default): the article's own minimax method (see
    // ParametricFIR.h). On: the same minimax LP, but restricted to a
    // small span of leading even discrete prolate spheroidal (Slepian)
    // directions instead of the full space of free taps - trading some
    // classical R_peak/E_ZC sidelobe suppression for a continuous-time
    // (sinc-reconstructed) impulse response that is inherently energy-
    // concentrated near t=0 by construction, rather than only bounding
    // discrete-sample sidelobes. Exists so the two can be A/B'd live
    // against real music rather than compared only by numbers.
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "prolateBasis", 1 }, "Prolate/DPSS Basis", false));

    // OFF (default): use the existing Black/minimum-sidelobe filter exactly
    // as the plugin has always designed it (Minimax or Prolate/DPSS Basis
    // above, unchanged - ParametricFIR.h itself is untouched by this mode).
    // ON: ignore both of those and design instead via
    // bbk::peakenergy::designPeakEnergyFIR() (see PeakEnergyFIR.h) - a
    // completely separate, isolated code path targeting direct maximum
    // peak-to-total-impulse-energy concentration (eta = centreTap^2 /
    // sum(taps^2)) rather than minimax's minimum-largest-sidelobe
    // objective, under the exact same passband/stopband spectral
    // boundaries and unity-DC normalisation as the mode it replaces - nothing
    // is relaxed to make the new objective look better. Takes priority over
    // Prolate/DPSS Basis and Sidelobe Decay when on (both become no-ops;
    // see specFromParameters()/run() and the editor's greying-out of those
    // controls).
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "peakEnergyOptimized", 1 }, "Peak-Energy Optimized", false));

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

    const bool prolateBasisOn = parameters.getRawParameterValue ("prolateBasis")->load() > 0.5f;
    spec.designMethod = prolateBasisOn ? bbk::parametric::DesignMethod::ProlateBasis
                                        : bbk::parametric::DesignMethod::Minimax;

    spec.sidelobeDecayRatio = static_cast<double> (parameters.getRawParameterValue ("sidelobeDecay")->load());
    return spec;
}

BBKDetachedPoleAudioProcessor::DesignMode BBKDetachedPoleAudioProcessor::selectedModeFromParameters() const
{
    // Priority order matches specFromParameters()/the editor's greying-out
    // logic exactly: Peak-Energy Optimized, when on, overrides Prolate/DPSS
    // Basis entirely (that toggle becomes a no-op, not a stacked modifier).
    if (parameters.getRawParameterValue ("peakEnergyOptimized")->load() > 0.5f)
        return DesignMode::PeakEnergy;
    if (parameters.getRawParameterValue ("prolateBasis")->load() > 0.5f)
        return DesignMode::ProlateBasis;
    return DesignMode::Minimax;
}

void BBKDetachedPoleAudioProcessor::requestBoundaryRedesign()
{
    // May be called from the message thread (typical - a slider moved) or
    // from the audio thread (a host delivered automation for one of these
    // parameters mid-block) - either way this only ever copies a small
    // FilterSpec/queues three lightweight DesignTasks under a SpinLock and
    // signals the worker thread; the actual (slow) design work never runs
    // here. See the class-level comment in PluginProcessor.h for the full
    // rationale behind bumping boundaryEpoch and invalidating all three
    // per-mode caches rather than just re-running the currently selected
    // one.
    if (! hasPrepared.load() || currentSampleRate.load() <= 0.0)
        return;

    const auto spec = specFromParameters();
    const auto selectedMode = selectedModeFromParameters();

    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        currentBoundarySpec = spec;
        ++boundaryEpoch;
        taskQueue.clear();

        auto pushTask = [&] (DesignMode mode)
        {
            DesignTask task;
            task.spec = spec;
            // Only Minimax/ProlateBasis tasks actually consult
            // spec.designMethod (designPeakEnergyFIR() ignores it - see
            // PeakEnergyFIR.h) but setting it consistently per task keeps
            // the queued spec self-describing.
            task.spec.designMethod = (mode == DesignMode::ProlateBasis) ? bbk::parametric::DesignMethod::ProlateBasis
                                                                          : bbk::parametric::DesignMethod::Minimax;
            task.mode = mode;
            task.epoch = boundaryEpoch;
            taskQueue.push_back (task);
        };

        // Selected mode first, so the still-audible/about-to-be-audible
        // result arrives as soon as possible; the other two fill the cache
        // in behind it, invisibly to playback, ready for an instant
        // switch later.
        pushTask (selectedMode);
        for (auto mode : { DesignMode::Minimax, DesignMode::ProlateBasis, DesignMode::PeakEnergy })
            if (mode != selectedMode)
                pushTask (mode);
    }

    {
        const juce::SpinLock::ScopedLockType sl (resultLock);
        cacheMinimax.valid = false;
        cacheProlateBasis.valid = false;
        cachePeakEnergy.valid = false;
    }

    notify();
}

void BBKDetachedPoleAudioProcessor::enforceModeExclusivity (const juce::String& changedParamID, float newValue)
{
    // Only turning a mode ON is ever a reason to turn the other OFF -
    // unchecking one should never reach back and touch the other.
    if (newValue <= 0.5f || enforcingExclusivity)
        return;

    const char* otherID = (changedParamID == "prolateBasis") ? "peakEnergyOptimized" : "prolateBasis";
    if (auto* other = parameters.getParameter (juce::String (otherID)))
    {
        if (other->getValue() > 0.5f)
        {
            enforcingExclusivity = true;
            other->setValueNotifyingHost (0.0f);
            enforcingExclusivity = false;
        }
    }
}

void BBKDetachedPoleAudioProcessor::requestModeSwitch()
{
    // Deliberately does NOT touch currentBoundarySpec/boundaryEpoch/the
    // caches for the other two modes - a mode toggle is a pure "look at
    // what's already known" operation, never a reason to redesign
    // anything. See the class-level comment in PluginProcessor.h.
    if (! hasPrepared.load() || currentSampleRate.load() <= 0.0)
        return;

    const auto selectedMode = selectedModeFromParameters();

    int epoch;
    bbk::parametric::FilterSpec spec;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        epoch = boundaryEpoch;
        spec = currentBoundarySpec;
        spec.designMethod = (selectedMode == DesignMode::ProlateBasis) ? bbk::parametric::DesignMethod::ProlateBasis
                                                                         : bbk::parametric::DesignMethod::Minimax;

        // If the newly selected mode's task is still queued behind
        // others, bump it to the front - the user is waiting on it now,
        // whatever else was mid-fill can wait its turn.
        for (auto it = taskQueue.begin(); it != taskQueue.end(); ++it)
        {
            if (it->mode == selectedMode && it->epoch == epoch)
            {
                auto task = *it;
                taskQueue.erase (it);
                taskQueue.push_front (task);
                break;
            }
        }
    }

    CachedDesign cached;
    bool haveCached = false;
    {
        const juce::SpinLock::ScopedLockType sl (resultLock);
        const CachedDesign& slot = (selectedMode == DesignMode::PeakEnergy) ? cachePeakEnergy
                                  : (selectedMode == DesignMode::ProlateBasis) ? cacheProlateBasis
                                                                                : cacheMinimax;
        if (slot.valid && slot.epoch == epoch)
        {
            cached = slot;
            haveCached = true;
        }
    }

    if (haveCached)
    {
        // Instant path: already known for the current boundary - install
        // and crossfade straight away, no wait, no worker thread
        // involvement at all.
        publishResult (spec, selectedMode, cached.result, cached.etaAchieved, cached.concentrationDb);
        return;
    }

    notify(); // wake the worker so it picks up the just-reprioritised task promptly
}

void BBKDetachedPoleAudioProcessor::publishResult (const bbk::parametric::FilterSpec& spec, DesignMode mode,
                                                     const bbk::parametric::DesignResult& result,
                                                     double etaAchieved, double concentrationDb)
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
            latestPeakEnergyOn = (mode == DesignMode::PeakEnergy);
            latestEtaAchieved = etaAchieved;
            latestConcentrationDb = concentrationDb;
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
        uiSnapshot.designMethod = (mode == DesignMode::ProlateBasis) ? bbk::parametric::DesignMethod::ProlateBasis
                                                                       : bbk::parametric::DesignMethod::Minimax;
        uiSnapshot.sidelobeDecayRatio = spec.sidelobeDecayRatio;
        uiSnapshot.amplitudeRelaxationOn = parameters.getRawParameterValue ("amplitudeRelaxation")->load() > 0.5f;
        uiSnapshot.tapCount = result.tapCount;
        uiSnapshot.achievedStopbandDb = result.achievedStopbandDb;
        uiSnapshot.constraintsMet = result.constraintsMet;
        uiSnapshot.designAttempts = result.designAttempts;
        uiSnapshot.taps = result.taps;
        uiSnapshot.temporal = result.temporal;
        uiSnapshot.peakEnergyOptimizedOn = (mode == DesignMode::PeakEnergy);
        uiSnapshot.etaAchieved = etaAchieved;
        uiSnapshot.concentrationDb = concentrationDb;
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
        // no point spending time (possibly several seconds, for
        // Peak-Energy Optimized) designing a spec nobody wants any more.
        {
            const juce::SpinLock::ScopedLockType sl (specLock);
            if (task.epoch != boundaryEpoch)
                continue;
        }

        // Peak-Energy Optimized is a completely separate, isolated engine
        // (PeakEnergyFIR.h) - its result is copied into the same
        // bbk::parametric::DesignResult shape (both structs share these
        // field names/meanings by design) so everything downstream - the
        // cache slots, the crossfade hand-off, the UI snapshot's core
        // fields - works identically regardless of which mode ran. Only
        // the two peak-energy-specific fields (eta/concentration) are
        // carried separately, since ordinary DesignResult has no room for
        // them.
        bbk::parametric::DesignResult result;
        double etaAchieved = 0.0;
        double concentrationDb = 0.0;
        if (task.mode == DesignMode::PeakEnergy)
        {
            const auto peResult = bbk::peakenergy::designPeakEnergyFIR (task.spec, bbk::detachedpole::maxTapCount);
            result.taps = peResult.taps;
            result.tapCount = peResult.tapCount;
            result.constraintsMet = peResult.constraintsMet;
            result.achievedStopbandDb = peResult.achievedStopbandDb;
            result.designAttempts = peResult.designAttempts;
            result.temporal = peResult.temporal;
            etaAchieved = peResult.etaAchieved;
            concentrationDb = peResult.concentrationDb;
        }
        else
        {
            result = bbk::parametric::designParametricFIR (task.spec, bbk::detachedpole::maxTapCount);

            // Minimax/Prolate don't optimise for this the way Peak-Energy
            // does, but the same eta/concentration numbers can still be
            // measured after the fact from their taps, so all three modes
            // report a directly comparable figure (see computeEtaMetrics
            // above).
            const auto etaMetrics = computeEtaMetrics (result.taps);
            etaAchieved = etaMetrics.eta;
            concentrationDb = etaMetrics.concentrationDb;
        }

        // Re-check staleness after the (possibly slow) computation - a
        // newer boundary change may have arrived while this task was
        // running. If so, the work is simply discarded: not cached (it
        // would be wrong for the new boundary) and not published.
        bool stillCurrent;
        {
            const juce::SpinLock::ScopedLockType sl (specLock);
            stillCurrent = (task.epoch == boundaryEpoch);
        }
        if (! stillCurrent)
            continue;

        {
            const juce::SpinLock::ScopedLockType sl (resultLock);
            CachedDesign& slot = (task.mode == DesignMode::PeakEnergy) ? cachePeakEnergy
                                : (task.mode == DesignMode::ProlateBasis) ? cacheProlateBasis
                                                                           : cacheMinimax;
            slot.valid = true;
            slot.epoch = task.epoch;
            slot.result = result;
            slot.etaAchieved = etaAchieved;
            slot.concentrationDb = concentrationDb;
        }

        // Only crossfade this into playback if it's both still the
        // current boundary AND the mode the user currently has selected -
        // a cache-fill for a mode nobody's listening to right now just
        // sits in the cache quietly, ready for an instant switch later.
        // (selectedModeFromParameters() is re-read fresh here rather than
        // captured with the task, since the user may have toggled modes
        // again while this design was running.)
        if (task.mode == selectedModeFromParameters())
            publishResult (task.spec, task.mode, result, etaAchieved, concentrationDb);
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
