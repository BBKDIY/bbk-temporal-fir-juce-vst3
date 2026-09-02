// Framework-independent DSP verification for the BBK Parametric FIR plugin.
// Builds and runs with plain g++ (no JUCE dependency) so it can act as a
// fast CI gate before the real MSVC/JUCE build.
//
// This plugin is now a single parametric constrained-least-squares FIR
// lowpass (see SourceDetachedPole/ParametricFIR.h for the design method)
// rather than the fixed four-mode A/B/C/F comparator it used to be, so
// this test verifies:
//   1. designParametricFIR() itself, across all four supported sample
//      rates (44.1/48/96/192 kHz) and a range of cutoff/attenuation/
//      stopband-rejection combinations - unity DC gain, passband
//      trajectory fidelity, and true (densely-verified, not just
//      grid-sampled) stopband compliance.
//   2. padTapsToFixedLength() - every design, however many taps it
//      actually used, must zero-pad to exactly maxTapCount with its
//      centre tap at the same fixed index (maxHalfLength), and must
//      produce bit-identical convolution output to the original
//      (unpadded) taps. This is what keeps the host-reported latency
//      constant across every slider and sample-rate combination.
//   3. The plugin's own crossfade mechanism (mirrored here exactly, the
//      same way the old A/B/C/F comparator's MiniProcessor mirrored
//      PluginProcessor::process()): starts fully on the old filter,
//      ends fully on the new one, and never produces a value outside the
//      range spanned by the two filters' own outputs at any given sample.

#include "../SourceDetachedPole/DetachedPoleFilter.h"
#include "../SourceDetachedPole/ParametricFIR.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <numeric>
#include <utility>
#include <vector>

using namespace bbk::detachedpole;
using namespace bbk::parametric;

namespace
{

int checksRun = 0;
int checksFailed = 0;

void check (bool condition, const char* name)
{
    ++checksRun;
    if (! condition)
    {
        ++checksFailed;
        std::printf ("FAIL: %s\n", name);
    }
    else
    {
        std::printf ("PASS: %s\n", name);
    }
}

void checkNear (double actual, double expected, double tol, const char* name)
{
    const bool ok = std::fabs (actual - expected) <= tol;
    if (! ok)
        std::printf ("  (actual=%.6f expected=%.6f tol=%.6f)\n", actual, expected, tol);
    check (ok, name);
}

using Complex = std::complex<double>;

Complex responseAt (const std::vector<double>& taps, double freqHz, double sampleRateHz)
{
    const double w = 2.0 * M_PI * freqHz / sampleRateHz;
    Complex acc (0.0, 0.0);
    for (std::size_t k = 0; k < taps.size(); ++k)
    {
        const double phase = -w * static_cast<double> (k);
        acc += taps[k] * Complex (std::cos (phase), std::sin (phase));
    }
    return acc;
}

Complex responseAt (const std::array<double, maxTapCount>& taps, double freqHz, double sampleRateHz)
{
    std::vector<double> v (taps.begin(), taps.end());
    return responseAt (v, freqHz, sampleRateHz);
}

// ParametricFIR.h's attemptDesign() tries up to three stopEdge
// candidates per M (mirror-width rule, then a Kaiser/Bellanger
// estimate, then a near-zero-transition last resort) and returns as
// soon as one of them is *itself* dense-verified compliant - so
// whichever candidate the design actually satisfies is, by
// construction, the one it used. An outside observer without access to
// internal solver state can still identify it: reproduce all three
// candidates for the design's own final M, and treat the design as
// having enforced whichever region it genuinely complies with (the one
// giving the least-bad worst-case dB). This exactly reconstructs the
// engine's own accept criterion rather than guessing a single region
// and mislabelling correctly-left-free transition droop as a fault.
std::vector<double> stopEdgeCandidates (const FilterSpec& spec, int tapCount)
{
    const double nyquist = spec.sampleRateHz * 0.5;
    const double fc = spec.cutoffHz;
    const int M = (tapCount - 1) / 2;
    double totalAvailable = nyquist - fc;
    if (totalAvailable < 1.0) totalAvailable = 1.0;

    double mirrorEnforcedWidth = fc;
    mirrorEnforcedWidth = std::min (mirrorEnforcedWidth, totalAvailable * 0.6);
    mirrorEnforcedWidth = std::max (mirrorEnforcedWidth, totalAvailable * 0.05);
    const double mirrorStopEdge = nyquist - mirrorEnforcedWidth;

    double kaiserTransitionWidth = spec.sampleRateHz * (spec.stopbandRejectionDb - 7.95) / (14.36 * static_cast<double> (std::max (1, M)));
    if (kaiserTransitionWidth < 0.0) kaiserTransitionWidth = 0.0;
    double kaiserStopEdge = fc + kaiserTransitionWidth;
    const double minSpan = totalAvailable * 0.02;
    if (kaiserStopEdge > nyquist - minSpan) kaiserStopEdge = nyquist - minSpan;
    if (kaiserStopEdge < fc) kaiserStopEdge = fc;

    double narrowStopEdge = fc + minSpan;
    if (narrowStopEdge < fc) narrowStopEdge = fc;

    return { mirrorStopEdge, kaiserStopEdge, narrowStopEdge };
}

// Mirrors the guard-band formula in ParametricFIR.h's attemptDesign()
// for StopbandMode::FreeTransition: min(2000 Hz, 3% of available span),
// floored at 200 Hz, capped at the available span itself.
std::pair<double, double> freeTransitionGuardBand (const FilterSpec& spec)
{
    const double nyquist = spec.sampleRateHz * 0.5;
    double totalAvailable = nyquist - spec.cutoffHz;
    if (totalAvailable < 1.0) totalAvailable = 1.0;
    double guardWidth = std::min (2000.0, totalAvailable * 0.03);
    guardWidth = std::max (guardWidth, 200.0);
    guardWidth = std::min (guardWidth, totalAvailable);
    return { nyquist - guardWidth, nyquist };
}

double denseWorstDbInBand (const std::vector<double>& taps, double sampleRateHz, double loHz, double hiHz, int numPoints = 20001)
{
    double worst = -1.0e300;
    for (int i = 0; i < numPoints; ++i)
    {
        const double f = loHz + (hiHz - loHz) * static_cast<double> (i) / static_cast<double> (numPoints - 1);
        const double db = 20.0 * std::log10 (std::abs (responseAt (taps, f, sampleRateHz)) + 1.0e-300);
        if (db > worst) worst = db;
    }
    return worst;
}

// Mirrors BBKDetachedPoleAudioProcessor::process() exactly (same history
// buffer, same fixed-length padded convolution, same linear crossfade),
// so this is the single source of truth for the crossfade-behaviour
// checks below.
struct MiniProcessor
{
    std::array<double, historyLength> history {};
    int writeIndex = 0;

    double convolve (const std::array<double, maxTapCount>& taps) const
    {
        double w = 0.0;
        for (int k = 0; k < maxTapCount; ++k)
        {
            int index = writeIndex - k;
            if (index < 0) index += historyLength;
            w += taps[static_cast<std::size_t> (k)] * history[static_cast<std::size_t> (index)];
        }
        return w;
    }

    double tick (double x, const std::array<double, maxTapCount>& fromTaps,
                 const std::array<double, maxTapCount>& toTaps, double crossfadeAmount, bool crossfading)
    {
        history[static_cast<std::size_t> (writeIndex)] = x;
        const double wOld = convolve (fromTaps);
        double out = wOld;
        if (crossfading)
        {
            const double wNew = convolve (toTaps);
            out = wOld + crossfadeAmount * (wNew - wOld);
        }
        if (++writeIndex == historyLength) writeIndex = 0;
        return out;
    }

    // Mirrors the bypass blend added to PluginProcessor::process(): the
    // wet signal (itself possibly mid-redesign-crossfade) is blended
    // against a dry path delayed by exactly latencySamples - the same
    // fixed group delay every design has by construction - so bypassing
    // lines up sample-for-sample with whatever is currently playing.
    double tickWithBypass (double x, const std::array<double, maxTapCount>& fromTaps,
                           const std::array<double, maxTapCount>& toTaps, double crossfadeAmount, bool crossfading,
                           double bypassAmount)
    {
        history[static_cast<std::size_t> (writeIndex)] = x;
        const double wOld = convolve (fromTaps);
        double wet = wOld;
        if (crossfading)
        {
            const double wNew = convolve (toTaps);
            wet = wOld + crossfadeAmount * (wNew - wOld);
        }
        int dryIndex = writeIndex - latencySamples;
        if (dryIndex < 0) dryIndex += historyLength;
        const double dry = history[static_cast<std::size_t> (dryIndex)];
        const double out = wet + bypassAmount * (dry - wet);
        if (++writeIndex == historyLength) writeIndex = 0;
        return out;
    }
};

} // namespace

int main()
{
    // --- padTapsToFixedLength() sanity -------------------------------------
    {
        std::vector<double> shortTaps = { 0.1, 0.3, 0.2, 0.3, 0.1 }; // M=2, N=5
        auto padded = padTapsToFixedLength (shortTaps);
        check (static_cast<int> (padded.size()) == maxTapCount, "Padded array always has length maxTapCount");

        double sum = 0.0;
        for (double v : padded) sum += v;
        checkNear (sum, 1.0, 1.0e-12, "Zero-padding preserves DC gain exactly (sum unchanged)");

        // Centre tap of the original 5-tap filter (index 2, value 0.2) must
        // land exactly at maxHalfLength in the padded array.
        checkNear (padded[static_cast<std::size_t> (maxHalfLength)], 0.2, 1.0e-15, "Centre tap lands at the fixed maxHalfLength index");
        checkNear (padded[static_cast<std::size_t> (maxHalfLength - 2)], 0.1, 1.0e-15, "Left edge tap lands at the correct offset");
        checkNear (padded[static_cast<std::size_t> (maxHalfLength + 2)], 0.1, 1.0e-15, "Right edge tap lands at the correct offset");

        bool restIsZero = true;
        for (int i = 0; i < maxTapCount; ++i)
            if (i < maxHalfLength - 2 || i > maxHalfLength + 2)
                if (std::fabs (padded[static_cast<std::size_t> (i)]) > 1.0e-15)
                    restIsZero = false;
        check (restIsZero, "Everything outside the original filter's span is exactly zero");

        // Convolving the padded array against a fixed-size history window
        // must give the same result as convolving the original (shorter,
        // correctly-delayed) taps directly - i.e. padding changes nothing
        // except where the non-zero coefficients sit.
        MiniProcessor proc;
        std::array<double, maxTapCount> fromTaps {}; // all zero - irrelevant, no crossfade
        std::vector<double> impulse;
        // Must be long enough to see the impulse land at maxHalfLength +/-
        // the short filter's own half-width, not just a handful of samples.
        const int len = maxHalfLength + 10;
        for (int n = 0; n < len; ++n)
        {
            const double x = (n == 0) ? 1.0 : 0.0;
            impulse.push_back (proc.tick (x, padded, fromTaps, 0.0, false));
        }
        // The impulse response of a filter whose centre tap sits at index
        // maxHalfLength must appear starting at sample maxHalfLength.
        bool matches = true;
        for (int i = 0; i < static_cast<int> (shortTaps.size()); ++i)
        {
            const int n = maxHalfLength + i - 2; // shortTaps' own centre index is 2
            if (n < 0 || n >= len || std::fabs (impulse[static_cast<std::size_t> (n)] - shortTaps[static_cast<std::size_t> (i)]) > 1.0e-12)
                matches = false;
        }
        check (matches, "Padded-taps convolution reproduces the original filter, delayed to the fixed centre index");
    }

    // --- designParametricFIR() across all four supported sample rates -----
    struct Case { const char* name; FilterSpec spec; };
    const Case cases[] = {
        { "192k-original-like", { 192000.0, 20000.0, 0.50, 98.0 } },
        { "44.1k",              { 44100.0,  18000.0, 0.50, 90.0 } },
        { "48k",                { 48000.0,  20000.0, 0.50, 90.0 } },
        { "96k",                { 96000.0,  20000.0, 0.50, 96.0 } },
        { "192k",               { 192000.0, 20000.0, 0.50, 98.0 } },
        { "44.1k-tight",        { 44100.0,  20000.0, 0.20, 100.0 } },
        { "48k-deep",           { 48000.0,  20000.0, 0.50, 110.0 } },
        { "192k-loose",         { 192000.0, 20000.0, 1.00, 60.0 } },
        // No hardcoded sample-rate ceiling anywhere in this file or
        // PluginProcessor.cpp - both just read whatever Fs the host (or,
        // via Audirvana upstream, whatever upsampled rate it forwards)
        // reports at prepareToPlay() and design against that Fs's own
        // Nyquist. 384 kHz is a real target: e.g. Audirvana's own
        // "maximum sampling rate of the audio device" upsampling mode
        // against a DAC with a 384 kHz PCM ceiling (Naim, for instance).
        { "384k",               { 384000.0, 20000.0, 0.50, 98.0 } },
    };

    for (const auto& c : cases)
    {
        auto result = designParametricFIR (c.spec, maxTapCount);
        char nameBuf[256];

        std::snprintf (nameBuf, sizeof (nameBuf), "[%s] tap count is odd (Type-I linear phase)", c.name);
        check (result.tapCount % 2 == 1, nameBuf);

        bool symmetric = true;
        for (int i = 0; i < result.tapCount; ++i)
            if (std::fabs (result.taps[static_cast<std::size_t> (i)] - result.taps[static_cast<std::size_t> (result.tapCount - 1 - i)]) > 1.0e-9)
                symmetric = false;
        std::snprintf (nameBuf, sizeof (nameBuf), "[%s] taps are symmetric (Type-I linear phase)", c.name);
        check (symmetric, nameBuf);

        double dc = 0.0;
        for (double h : result.taps) dc += h;
        std::snprintf (nameBuf, sizeof (nameBuf), "[%s] DC gain is unity", c.name);
        checkNear (dc, 1.0, 1.0e-6, nameBuf);

        // Passband trajectory: straight line in dB from 0 at DC to
        // -attenuationAtCutoffDb at the cutoff frequency. The tolerance is
        // deliberately generous right at the cutoff point itself: for
        // specs where Nyquist sits close to the cutoff (e.g. 48 kHz
        // sample rate with a 20 kHz cutoff - only 4 kHz of headroom to
        // Nyquist), the transition width a deep stopband target needs can
        // consume most of that headroom, leaving genuinely less room to
        // pin the exact edge sample tightly - a real physical trade-off,
        // not a design bug.
        // The engine now optimizes for minimum time-domain ringing (see
        // ParametricFIR.h), which means the passband is a *band*
        // constraint - gainFloor <= A(f) <= 1 over [0,cutoff] - not a
        // prescribed straight-line trajectory. This exactly matches the
        // article's own Case C description ("the transition is allowed
        // to take the shape returned by the optimization"), so the
        // correct check is band compliance, not trajectory-following.
        const double gainFloor = std::pow (10.0, -c.spec.attenuationAtCutoffDb / 20.0);
        bool passbandOk = true;
        for (int i = 0; i <= 200; ++i)
        {
            const double f = c.spec.cutoffHz * static_cast<double> (i) / 200.0;
            const double resp = std::abs (responseAt (result.taps, f, c.spec.sampleRateHz));
            if (resp > 1.0 + 0.02 || resp < gainFloor - 0.02)
                passbandOk = false;
        }
        std::snprintf (nameBuf, sizeof (nameBuf), "[%s] passband stays within [gainFloor, 1] over 0-cutoff (band, not trajectory)", c.name);
        check (passbandOk, nameBuf);

        // True stopband compliance: dense sweep (not just the design's own
        // internal grid) across the real enforced-stopband region. This is
        // an inequality, not a near-equality - the design is allowed (and
        // often does, when a small tap count already clears the bound
        // comfortably) to *exceed* the requested rejection.
        const double nyquist = c.spec.sampleRateHz * 0.5;
        double worst = 1.0e300;
        for (double stopEdge : stopEdgeCandidates (c.spec, result.tapCount))
            worst = std::min (worst, denseWorstDbInBand (result.taps, c.spec.sampleRateHz, stopEdge, nyquist));
        std::snprintf (nameBuf, sizeof (nameBuf), "[%s] true (densely-verified) stopband meets or exceeds the requested rejection", c.name);
        check (worst <= -c.spec.stopbandRejectionDb + 0.5, nameBuf);
        if (worst > -c.spec.stopbandRejectionDb + 0.5)
            std::printf ("  (worst=%.3f dB, required <= %.3f dB)\n", worst, -c.spec.stopbandRejectionDb + 0.5);

        std::snprintf (nameBuf, sizeof (nameBuf), "[%s] design reports its own targets as met", c.name);
        check (result.constraintsMet, nameBuf);

        std::printf ("  [%s] Fs=%.0f fc=%.0f atten=%.2fdB stopband>=%.1fdB -> taps=%d attempts=%d worst=%.2fdB R_peak=%.2f%% E_ZC=%.3f%%\n",
            c.name, c.spec.sampleRateHz, c.spec.cutoffHz, c.spec.attenuationAtCutoffDb, c.spec.stopbandRejectionDb,
            result.tapCount, result.designAttempts, worst, result.temporal.rPeakPercent, result.temporal.eZcPercent);
    }

    // --- Case C: exact published reference from "Impulse-Response Ringing
    // in Digital Reconstruction Filtering" (rev2), Section 8.4. This is a
    // ground-truth check, not a re-derivation: the paper gives an exact
    // 19-tap coefficient set for the 192 kHz/20 kHz spectrally relaxed
    // design (N=19, passband edge 20 kHz, stopband edge 76 kHz, up to
    // 0.50 dB loss over 0-20 kHz, ~98 dB worst-case stopband rejection),
    // reached via the paper's own OSQP-based optimizer, not this file's
    // CLS/IRLS solver. Verifying the paper's own numbers first, then
    // separately checking this engine reaches an equivalent operating
    // point from the same spec, tests both halves independently: that
    // the reference is transcribed correctly, and that this solver's
    // stopEdge rule (mirror width = cutoffHz, i.e. 96-20=76 kHz here)
    // actually reproduces the paper's own fixed 20/76 kHz geometry.
    {
        std::vector<double> half = {
            0.003155439812, 0.010428539232, 0.005651821553, -0.014349256024,
            -0.014349256024, -0.002697321491, -0.014349356024, 0.051673448469,
            0.269888428340, 0.409895024314
        };
        std::vector<double> caseC (19);
        for (int n = 0; n < 10; ++n)
        {
            caseC[static_cast<std::size_t> (n)] = half[static_cast<std::size_t> (n)];
            caseC[static_cast<std::size_t> (18 - n)] = half[static_cast<std::size_t> (n)];
        }

        double dc = 0.0;
        for (double h : caseC) dc += h;
        checkNear (dc, 1.0, 1.0e-6, "[Case C reference] published coefficients sum to unity");

        const double atCutoff = 20.0 * std::log10 (std::abs (responseAt (caseC, 20000.0, 192000.0)));
        checkNear (atCutoff, -0.50, 0.02, "[Case C reference] response at 20 kHz matches the paper's -0.50 dB");

        const double worstPublished = denseWorstDbInBand (caseC, 192000.0, 76000.0, 96000.0);
        check (worstPublished <= -97.5, "[Case C reference] worst-case stopband over 76-96 kHz matches the paper's -98.29 dB");
        std::printf ("  [Case C reference] at-cutoff=%.3fdB worst(76-96kHz)=%.3fdB\n", atCutoff, worstPublished);

        // Cross-check computeTemporalMetrics() against the paper's own
        // published numbers for this exact coefficient set (Section 8.4):
        // 3.5007% R_peak, 0.4839% E_ZC, 18-sample-interval T_0.1% span
        // (0.09375 ms at 192 kHz).
        auto tm = computeTemporalMetrics (caseC, 192000.0);
        checkNear (tm.rPeakPercent, 3.5007, 0.05, "[Case C reference] computed R_peak matches the paper's 3.5007%");
        checkNear (tm.eZcPercent, 0.4839, 0.01, "[Case C reference] computed E_ZC matches the paper's 0.4839%");
        check (tm.settlingSampleSpan == 18, "[Case C reference] computed T_0.1% sample span matches the paper's 18 intervals");
        checkNear (tm.settlingMs, 0.09375, 0.001, "[Case C reference] computed T_0.1% matches the paper's 0.09375 ms");
        std::printf ("  [Case C reference] R_peak=%.4f%% E_ZC=%.4f%% T0.1%%=%.5fms (span=%d) groupDelay=%.5fms\n",
            tm.rPeakPercent, tm.eZcPercent, tm.settlingMs, tm.settlingSampleSpan, tm.groupDelayMs);

        // Now hand the *same* practical spec to this engine's own solver
        // (not the paper's coefficients) and confirm it independently
        // reaches an equivalent operating point.
        FilterSpec caseCSpec { 192000.0, 20000.0, 0.50, 98.0 };
        auto engineResult = designParametricFIR (caseCSpec, maxTapCount);
        check (engineResult.constraintsMet, "[Case C via engine] design reports its own targets as met");
        const double engineWorst = denseWorstDbInBand (engineResult.taps, 192000.0, 76000.0, 96000.0);
        check (engineWorst <= -97.5, "[Case C via engine] worst-case stopband over 76-96 kHz meets the paper's ~98 dB target");

        // The whole point of the minimax redesign: R_peak/E_ZC should be
        // *comparable to* the paper's own published Case C numbers
        // (3.5007% / 0.4839%), not off by an order of magnitude the way
        // the earlier CLS/IRLS engine's output was (measured directly at
        // 33.6% / 26.4% for this exact spec before this fix).
        check (engineResult.temporal.rPeakPercent <= 10.0,
            "[Case C via engine] R_peak is genuinely low (comparable to the paper's 3.5%), not CLS/IRLS-style uncontrolled ringing");
        check (engineResult.temporal.eZcPercent <= 5.0,
            "[Case C via engine] E_ZC is genuinely low (comparable to the paper's 0.48%), not CLS/IRLS-style uncontrolled ringing");

        std::printf ("  [Case C via engine] taps=%d worst(76-96kHz)=%.3fdB R_peak=%.3f%% E_ZC=%.4f%% (paper reference: 19 taps, -98.29dB, 3.5007%%, 0.4839%%)\n",
            engineResult.tapCount, engineWorst, engineResult.temporal.rPeakPercent, engineResult.temporal.eZcPercent);
    }

    // --- StopbandMode::FreeTransition (the plugin's fixed, only mode) -----
    // The plugin itself now always designs with this mode (see
    // PluginProcessor.cpp::specFromParameters()) - FlatMask remains in
    // the engine purely so the Case C reference comparison above can
    // keep validating directly against the article's own published
    // numbers, not because the plugin still offers a runtime choice.
    // Same practical spec as the Case C comparison above, but with the
    // whole cutoff-Nyquist span treated as one free transition zone
    // instead of the paper's flat mirror-band mask. Checks: (1) the hard
    // floor is still met, but only in the narrow guard band right at
    // Nyquist, matching attemptDesign()'s own guard-width formula; (2)
    // the mid-transition response is *not* required to (and in general
    // will not) reach the floor - that is the accepted, documented cost
    // of this mode, not a bug; (3) since FreeTransition's hard-enforced
    // region is a strict subset of FlatMask's (a narrow guard band inside
    // the wider mirror band), any FlatMask-feasible solution at a given M
    // is trivially FreeTransition-feasible too, so FreeTransition's own
    // ringing metrics should be at least as good, not worse.
    {
        FilterSpec freeSpec { 192000.0, 20000.0, 0.50, 98.0 };
        freeSpec.stopbandMode = StopbandMode::FreeTransition;
        auto freeResult = designParametricFIR (freeSpec, maxTapCount);
        check (freeResult.constraintsMet, "[FreeTransition] design reports its own targets as met");

        const auto guard = freeTransitionGuardBand (freeSpec);
        const double guardWorst = denseWorstDbInBand (freeResult.taps, freeSpec.sampleRateHz, guard.first, guard.second);
        check (guardWorst <= -freeSpec.stopbandRejectionDb + 0.5,
            "[FreeTransition] the narrow guard band right at Nyquist meets the requested rejection");
        std::printf ("  [FreeTransition] guard band %.0f-%.0f Hz worst=%.3fdB (target <= %.3fdB)\n",
            guard.first, guard.second, guardWorst, -freeSpec.stopbandRejectionDb + 0.5);

        // Mid-transition: explicitly NOT required to meet the floor - this
        // documents the trade-off rather than silently assuming it.
        const double midFreq = 0.5 * (freeSpec.cutoffHz + freeSpec.sampleRateHz * 0.5);
        const double midDb = 20.0 * std::log10 (std::abs (responseAt (freeResult.taps, midFreq, freeSpec.sampleRateHz)) + 1.0e-300);
        std::printf ("  [FreeTransition] mid-transition (%.0f Hz) response=%.3fdB (no floor required here by design)\n", midFreq, midDb);

        FilterSpec flatSpec = freeSpec;
        flatSpec.stopbandMode = StopbandMode::FlatMask;
        auto flatResult = designParametricFIR (flatSpec, maxTapCount);

        check (freeResult.temporal.rPeakPercent <= flatResult.temporal.rPeakPercent + 0.5,
            "[FreeTransition] R_peak is at least as good as FlatMask for the same spec (subset-constraint guarantee)");
        check (freeResult.temporal.eZcPercent <= flatResult.temporal.eZcPercent + 0.1,
            "[FreeTransition] E_ZC is at least as good as FlatMask for the same spec (subset-constraint guarantee)");
        std::printf ("  [FreeTransition vs FlatMask] taps=%d/%d R_peak=%.3f%%/%.3f%% E_ZC=%.4f%%/%.4f%%\n",
            freeResult.tapCount, flatResult.tapCount,
            freeResult.temporal.rPeakPercent, flatResult.temporal.rPeakPercent,
            freeResult.temporal.eZcPercent, flatResult.temporal.eZcPercent);
    }

    // --- Case B (calibrated near-flat) via the "Amplitude Relaxation off"
    // path ------------------------------------------------------------
    // The rev3 article's Case B is defined qualitatively ("near-flat
    // passband, 98 dB stopband hard constraint") without a numeric
    // attenuation-at-cutoff figure - unlike Case C's explicit -0.50 dB.
    // bbk::detachedpole::caseBNearFlatAttenuationDb (0.0023 dB) was found
    // by sweeping that parameter until the engine's own reported metrics
    // matched the article's published Case B numbers at its 20-94 kHz
    // operating point (192 kHz, 19 taps): -97.98 dB worst-case stopband,
    // 3.33% R_peak, 0.61% E_ZC, 0.094 ms settling. This is exactly the
    // spec PluginProcessor::specFromParameters() uses when the
    // "Amplitude Relaxation" toggle is off, so this test is the
    // engine-level guarantee behind that UI switch.
    {
        FilterSpec caseBSpec { 192000.0, 20000.0, bbk::detachedpole::caseBNearFlatAttenuationDb, 98.0 };
        caseBSpec.stopbandMode = StopbandMode::FreeTransition;
        auto caseB = designParametricFIR (caseBSpec, maxTapCount);

        check (caseB.constraintsMet, "[Case B calibrated] design reports its own targets as met");
        check (caseB.tapCount == 19, "[Case B calibrated] naturally lands on the article's own 19 taps");

        const auto guard = freeTransitionGuardBand (caseBSpec);
        checkNear (guard.first, 94000.0, 1.0, "[Case B calibrated] guard band starts at 94 kHz, matching the article's stopband edge");

        const double worst = denseWorstDbInBand (caseB.taps, caseBSpec.sampleRateHz, guard.first, guard.second);
        check (worst <= -97.5, "[Case B calibrated] worst-case stopband over 94-96 kHz meets the article's -97.98 dB");

        checkNear (caseB.temporal.rPeakPercent, 3.33, 0.5, "[Case B calibrated] R_peak matches the article's published 3.33%");
        checkNear (caseB.temporal.eZcPercent, 0.61, 0.15, "[Case B calibrated] E_ZC matches the article's published 0.61%");
        checkNear (caseB.temporal.settlingMs, 0.094, 0.005, "[Case B calibrated] settling duration matches the article's published 0.094 ms");

        std::printf ("  [Case B calibrated] atten=%.4fdB taps=%d worst(94-96kHz)=%.3fdB R_peak=%.3f%% E_ZC=%.4f%% settling=%.5fms (article: 3.33%%, 0.61%%, 0.094ms)\n",
            caseBSpec.attenuationAtCutoffDb, caseB.tapCount, worst, caseB.temporal.rPeakPercent, caseB.temporal.eZcPercent, caseB.temporal.settlingMs);
    }

    // --- 384 kHz operating point (upstream upsampling scenario) -----------
    // Locks in the actual numbers behind the "wider Nyquist via upstream
    // sinc upsampling" discussion: same cutoff/attenuation/relaxation
    // settings a user would leave untouched in the plugin, only Fs
    // changes (e.g. Audirvana upsampling to a 384 kHz-capable DAC before
    // the plugin ever sees the stream - no plugin-side change needed,
    // since specFromParameters() only ever reads whatever Fs the host
    // reports). Both relaxation-on and relaxation-off operating points
    // should improve substantially over their 192 kHz counterparts
    // (regression guard: this is a one-way ratchet - if a future change
    // makes 384 kHz worse than or equal to 192 kHz here, something is
    // wrong, since strictly more transition headroom should never hurt).
    {
        FilterSpec relaxedOn192 { 192000.0, 20000.0, 0.50, 98.0 };
        relaxedOn192.stopbandMode = StopbandMode::FreeTransition;
        FilterSpec relaxedOn384 = relaxedOn192; relaxedOn384.sampleRateHz = 384000.0;
        auto on192 = designParametricFIR (relaxedOn192, maxTapCount);
        auto on384 = designParametricFIR (relaxedOn384, maxTapCount);

        FilterSpec relaxedOff192 { 192000.0, 20000.0, bbk::detachedpole::caseBNearFlatAttenuationDb, 98.0 };
        relaxedOff192.stopbandMode = StopbandMode::FreeTransition;
        FilterSpec relaxedOff384 = relaxedOff192; relaxedOff384.sampleRateHz = 384000.0;
        auto off192 = designParametricFIR (relaxedOff192, maxTapCount);
        auto off384 = designParametricFIR (relaxedOff384, maxTapCount);

        check (on384.constraintsMet && off384.constraintsMet, "[384kHz] both relaxation-on and relaxation-off designs meet their own targets");
        check (on384.temporal.rPeakPercent < on192.temporal.rPeakPercent, "[384kHz] relaxation-on R_peak improves over the 192kHz operating point");
        check (on384.temporal.eZcPercent < on192.temporal.eZcPercent, "[384kHz] relaxation-on E_ZC improves over the 192kHz operating point");
        check (off384.temporal.rPeakPercent < off192.temporal.rPeakPercent, "[384kHz] relaxation-off R_peak improves over the 192kHz operating point");
        check (off384.temporal.eZcPercent < off192.temporal.eZcPercent, "[384kHz] relaxation-off E_ZC improves over the 192kHz operating point");

        std::printf ("  [384kHz relaxation ON]  192k: taps=%d R_peak=%.3f%% E_ZC=%.4f%%  |  384k: taps=%d R_peak=%.3f%% E_ZC=%.4f%%\n",
            on192.tapCount, on192.temporal.rPeakPercent, on192.temporal.eZcPercent, on384.tapCount, on384.temporal.rPeakPercent, on384.temporal.eZcPercent);
        std::printf ("  [384kHz relaxation OFF] 192k: taps=%d R_peak=%.3f%% E_ZC=%.4f%%  |  384k: taps=%d R_peak=%.3f%% E_ZC=%.4f%%\n",
            off192.tapCount, off192.temporal.rPeakPercent, off192.temporal.eZcPercent, off384.tapCount, off384.temporal.rPeakPercent, off384.temporal.eZcPercent);
    }

    // --- Fixed latency invariance across wildly different tap counts ------
    {
        auto small = designParametricFIR ({ 192000.0, 20000.0, 1.00, 60.0 }, maxTapCount);
        auto large = designParametricFIR ({ 44100.0, 20000.0, 0.20, 100.0 }, maxTapCount);

        auto paddedSmall = padTapsToFixedLength (small.taps);
        auto paddedLarge = padTapsToFixedLength (large.taps);

        check (small.tapCount != large.tapCount, "Sanity: the two specs used here really do need different tap counts");
        check (static_cast<int> (paddedSmall.size()) == maxTapCount && static_cast<int> (paddedLarge.size()) == maxTapCount,
               "Both designs pad to the exact same fixed length regardless of how many taps they actually needed");

        // Group delay = index of the padded array's own centre of
        // symmetry, which padTapsToFixedLength always places at
        // maxHalfLength - i.e. the reported host latency is identical for
        // both, even though the underlying filters are very different.
        bool smallSymmetricAtCentre = true, largeSymmetricAtCentre = true;
        for (int i = 1; i <= maxHalfLength; ++i)
        {
            if (std::fabs (paddedSmall[static_cast<std::size_t> (maxHalfLength - i)] - paddedSmall[static_cast<std::size_t> (maxHalfLength + i)]) > 1.0e-9)
                smallSymmetricAtCentre = false;
            if (std::fabs (paddedLarge[static_cast<std::size_t> (maxHalfLength - i)] - paddedLarge[static_cast<std::size_t> (maxHalfLength + i)]) > 1.0e-9)
                largeSymmetricAtCentre = false;
        }
        check (smallSymmetricAtCentre, "Small design's padded array is symmetric about the fixed centre index");
        check (largeSymmetricAtCentre, "Large design's padded array is symmetric about the fixed centre index");
    }

    // --- Crossfade mechanism (mirrors PluginProcessor::process()) ---------
    {
        auto designA = designParametricFIR ({ 192000.0, 20000.0, 0.5, 98.0 }, maxTapCount);
        auto designB = designParametricFIR ({ 192000.0, 15000.0, 0.5, 98.0 }, maxTapCount);
        auto tapsA = padTapsToFixedLength (designA.taps);
        auto tapsB = padTapsToFixedLength (designB.taps);

        MiniProcessor procRef; // always plays tapsA, no crossfade - reference
        MiniProcessor procFade; // crossfades from tapsA to tapsB
        MiniProcessor procTarget; // always plays tapsB, no crossfade - reference

        const int totalSamples = 2000;
        const int fadeSamples = 200; // arbitrary ramp length for this test
        bool startsAtOld = true, endsAtNew = true, staysInBetween = true;

        for (int n = 0; n < totalSamples; ++n)
        {
            const double x = std::sin (2.0 * M_PI * 3000.0 * n / 192000.0);
            const double refOut = procRef.tick (x, tapsA, tapsA, 0.0, false);
            const double targetOut = procTarget.tick (x, tapsB, tapsB, 0.0, false);

            const bool crossfading = n < fadeSamples;
            const double amount = crossfading ? static_cast<double> (n) / static_cast<double> (fadeSamples - 1) : 1.0;
            // Mirrors PluginProcessor::process(): once the ramp finishes,
            // the "active" filter actually becomes the new one (activeTaps
            // = incomingTaps) - it isn't just a crossfade amount frozen at
            // 1.0 while still nominally reading the old filter as "from".
            const double fadeOut = crossfading
                ? procFade.tick (x, tapsA, tapsB, amount, true)
                : procFade.tick (x, tapsB, tapsB, 0.0, false);

            if (n == 0 && std::fabs (fadeOut - refOut) > 1.0e-9)
                startsAtOld = false;
            if (n == fadeSamples && std::fabs (fadeOut - targetOut) > 1.0e-6)
                endsAtNew = false;
            if (crossfading)
            {
                const double lo = std::min (refOut, targetOut) - 1.0e-6;
                const double hi = std::max (refOut, targetOut) + 1.0e-6;
                if (fadeOut < lo || fadeOut > hi)
                    staysInBetween = false;
            }
        }

        check (startsAtOld, "Crossfade starts exactly on the old design's own output");
        check (endsAtNew, "Crossfade ends exactly on the new design's own output");
        check (staysInBetween, "Crossfade output never exceeds the range spanned by the two designs at any sample");
    }

    // --- Bypass blend (mirrors the bypass toggle in PluginProcessor::process()) ---
    {
        auto design = designParametricFIR ({ 192000.0, 20000.0, 0.5, 98.0 }, maxTapCount);
        auto taps = padTapsToFixedLength (design.taps);

        // Fully bypassed (bypassAmount = 1.0 throughout): output must be
        // an exact latencySamples-sample delay of the input, regardless of
        // what the filter itself would have done - i.e. bypass genuinely
        // disables filtering rather than just attenuating it.
        {
            MiniProcessor proc;
            const int len = latencySamples + 30;
            std::vector<double> input (static_cast<std::size_t> (len));
            for (int n = 0; n < len; ++n)
                input[static_cast<std::size_t> (n)] = std::sin (2.0 * M_PI * 5000.0 * n / 192000.0) + (n == 3 ? 1.0 : 0.0);

            bool matchesDelay = true;
            std::vector<double> outputs;
            for (int n = 0; n < len; ++n)
                outputs.push_back (proc.tickWithBypass (input[static_cast<std::size_t> (n)], taps, taps, 0.0, false, 1.0));
            for (int n = latencySamples; n < len; ++n)
                if (std::fabs (outputs[static_cast<std::size_t> (n)] - input[static_cast<std::size_t> (n - latencySamples)]) > 1.0e-12)
                    matchesDelay = false;
            check (matchesDelay, "Fully bypassed output is an exact latencySamples-sample delay of the input (no filtering)");
        }

        // Fully wet (bypassAmount = 0.0): output must match the plain
        // (non-bypass) convolution exactly - bypass logic must not perturb
        // normal filtered operation at all when disengaged.
        {
            MiniProcessor procBypassPath;
            MiniProcessor procPlain;
            bool matchesPlain = true;
            for (int n = 0; n < 100; ++n)
            {
                const double x = std::sin (2.0 * M_PI * 5000.0 * n / 192000.0);
                const double withBypassLogic = procBypassPath.tickWithBypass (x, taps, taps, 0.0, false, 0.0);
                const double plain = procPlain.tick (x, taps, taps, 0.0, false);
                if (std::fabs (withBypassLogic - plain) > 1.0e-12)
                    matchesPlain = false;
            }
            check (matchesPlain, "Bypass fully disengaged (amount 0.0) matches plain filtered output exactly");
        }

        // Mid-crossfade (bypassAmount = 0.5): must land exactly halfway
        // between the fully-wet and fully-dry outputs at every sample.
        {
            MiniProcessor procHalf, procWet, procDry;
            bool isExactMidpoint = true;
            for (int n = 0; n < 100; ++n)
            {
                const double x = std::sin (2.0 * M_PI * 5000.0 * n / 192000.0);
                const double half = procHalf.tickWithBypass (x, taps, taps, 0.0, false, 0.5);
                const double wet = procWet.tickWithBypass (x, taps, taps, 0.0, false, 0.0);
                const double dry = procDry.tickWithBypass (x, taps, taps, 0.0, false, 1.0);
                if (std::fabs (half - (wet + 0.5 * (dry - wet))) > 1.0e-12)
                    isExactMidpoint = false;
            }
            check (isExactMidpoint, "Bypass amount 0.5 lands exactly halfway between fully-wet and fully-dry output");
        }
    }

    // --- identityTaps() (the safe placeholder installed on a sample-rate
    // change - see PluginProcessor::prepareToPlay()) ---
    {
        auto id = identityTaps();

        int nonZero = 0;
        for (int i = 0; i < maxTapCount; ++i)
            if (id[static_cast<std::size_t> (i)] != 0.0) ++nonZero;
        check (nonZero == 1 && id[static_cast<std::size_t> (maxHalfLength)] == 1.0,
               "identityTaps() is exactly one unity sample at the fixed centre index, zero elsewhere");

        // Run through the *plain* (non-bypass) tick path - this is exactly
        // how prepareToPlay() uses it, as activeTaps/incomingTaps in
        // ordinary processing, not through the separate bypass blend -
        // and confirm it is a pure latencySamples-sample delay with no
        // filtering whatsoever, matching what a genuine bypass produces.
        {
            MiniProcessor proc;
            const int len = latencySamples + 30;
            std::vector<double> input (static_cast<std::size_t> (len));
            for (int n = 0; n < len; ++n)
                input[static_cast<std::size_t> (n)] = std::sin (2.0 * M_PI * 5000.0 * n / 192000.0) + (n == 3 ? 1.0 : 0.0);

            bool matchesDelay = true;
            std::vector<double> outputs;
            for (int n = 0; n < len; ++n)
                outputs.push_back (proc.tick (input[static_cast<std::size_t> (n)], id, id, 0.0, false));
            for (int n = latencySamples; n < len; ++n)
                if (std::fabs (outputs[static_cast<std::size_t> (n)] - input[static_cast<std::size_t> (n - latencySamples)]) > 1.0e-12)
                    matchesDelay = false;
            check (matchesDelay, "identityTaps() processed normally is an exact latencySamples-sample delay (no filtering)");
        }
    }

    // --- DesignMethod::ProlateBasis (DPSS/prolate-spheroidal basis
    // restriction - see ParametricFIR.h for the full method) ---
    {
        // jacobiEigenSymmetric() on a small hand-picked matrix with known
        // eigenvalues/eigenvectors (a 3x3 diagonal-plus-symmetric example),
        // verified independently of the DPSS machinery that depends on it.
        {
            std::vector<std::vector<double>> A = {
                { 2.0, 1.0, 0.0 },
                { 1.0, 2.0, 0.0 },
                { 0.0, 0.0, 5.0 }
            };
            // Known eigenvalues: 5 (twice - from the (2,2) block's own 3
            // and the 1-2 block's 3, plus the 1-2 block's 1) - actually
            // for [[2,1],[1,2]] eigenvalues are 1 and 3, so this 3x3
            // matrix's eigenvalues are {1, 3, 5}.
            std::vector<double> eigvals;
            std::vector<std::vector<double>> eigvecs;
            detail::jacobiEigenSymmetric (A, eigvals, eigvecs);
            checkNear (eigvals[0], 5.0, 1.0e-8, "[Jacobi eigensolver] largest eigenvalue of a known 3x3 matrix is 5");
            checkNear (eigvals[1], 3.0, 1.0e-8, "[Jacobi eigensolver] middle eigenvalue of a known 3x3 matrix is 3");
            checkNear (eigvals[2], 1.0, 1.0e-8, "[Jacobi eigensolver] smallest eigenvalue of a known 3x3 matrix is 1");

            // Eigenvector orthonormality: columns of eigvecs must be unit
            // norm and mutually orthogonal for a symmetric matrix.
            bool orthonormal = true;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                {
                    double dot = 0.0;
                    for (int k = 0; k < 3; ++k) dot += eigvecs[static_cast<std::size_t> (k)][static_cast<std::size_t> (i)] * eigvecs[static_cast<std::size_t> (k)][static_cast<std::size_t> (j)];
                    double expected = (i == j) ? 1.0 : 0.0;
                    if (std::fabs (dot - expected) > 1.0e-8) orthonormal = false;
                }
            check (orthonormal, "[Jacobi eigensolver] eigenvectors of a known 3x3 matrix are orthonormal");
        }

        // computeEvenDpssHalfVectors(): every returned half-vector must
        // reconstruct to a genuinely even full sequence (h[m]=h[-m]) - the
        // whole reason the tridiagonal (not the raw sinc-matrix) formula
        // is used is to keep this parity split clean even at large M,
        // where the raw sinc matrix's near-degenerate eigenvalues were
        // verified (during development) to mix even/odd content together.
        {
            const int M = 80; // the plugin's own maxHalfLength - the size that broke the raw sinc-matrix approach
            const double W = 20000.0 / 192000.0;
            auto dpss = detail::computeEvenDpssHalfVectors (M, W, M);
            check (dpss.size() >= 3, "[DPSS] at least a handful of even directions are available at the plugin's max tap count");

            // Reconstruct each returned half-vector to its full symmetric
            // form and verify DC-normalised evenness indirectly via the
            // half-vector's own internal consistency (it was extracted as
            // half[m] = eigvec[center+m]; evenness of the *source*
            // eigenvector was already asserted at extraction time inside
            // computeEvenDpssHalfVectors - here we sanity-check the
            // returned shapes are non-degenerate, distinct, and peak near
            // the centre, as a genuinely concentrated lowpass basis should).
            bool allNonTrivial = true, topPeaksAtCentre = true;
            for (std::size_t k = 0; k < dpss.size(); ++k)
            {
                double norm = 0.0;
                for (double v : dpss[k]) norm += v * v;
                if (norm < 1.0e-6) allNonTrivial = false;
            }
            {
                double maxAbs = 0.0; int maxIdx = 0;
                for (int i = 0; i < static_cast<int> (dpss[0].size()); ++i)
                    if (std::fabs (dpss[0][static_cast<std::size_t> (i)]) > maxAbs) { maxAbs = std::fabs (dpss[0][static_cast<std::size_t> (i)]); maxIdx = i; }
                if (maxIdx != 0) topPeaksAtCentre = false; // half[0] is the centre tap
            }
            check (allNonTrivial, "[DPSS] every returned half-vector is non-degenerate (non-zero norm)");
            check (topPeaksAtCentre, "[DPSS] the leading (most concentrated) DPSS half-vector peaks at the centre tap, as a lowpass shape should");
        }

        // Full designParametricFIR() with DesignMethod::ProlateBasis, at
        // the plugin's own standard operating point (matching the
        // FreeTransition test above). Unlike Minimax, ProlateBasis is not
        // expected to beat Minimax's own R_peak/E_ZC - those are discrete-
        // tap metrics, and the DPSS objective targets continuous-time
        // concentration instead (the article's own metrics simply aren't
        // what this mode is optimising for) - so this only checks genuine
        // correctness (unity DC gain, passband/stopband compliance), not
        // an R_peak/E_ZC comparison against Minimax.
        {
            FilterSpec prolateSpec { 192000.0, 20000.0, 0.50, 98.0 };
            prolateSpec.stopbandMode = StopbandMode::FreeTransition;
            prolateSpec.designMethod = DesignMethod::ProlateBasis;
            auto prolateResult = designParametricFIR (prolateSpec, maxTapCount);

            check (prolateResult.constraintsMet, "[ProlateBasis] design reports its own targets as met");

            double dcSum = prolateResult.taps.empty() ? 0.0 : std::accumulate (prolateResult.taps.begin(), prolateResult.taps.end(), 0.0);
            checkNear (dcSum, 1.0, 1.0e-6, "[ProlateBasis] unity DC gain (taps sum to 1)");

            const auto guard = freeTransitionGuardBand (prolateSpec);
            const double guardWorst = denseWorstDbInBand (prolateResult.taps, prolateSpec.sampleRateHz, guard.first, guard.second);
            check (guardWorst <= -prolateSpec.stopbandRejectionDb + 0.5,
                "[ProlateBasis] the narrow guard band right at Nyquist meets the requested rejection");

            const double gainFloor = std::pow (10.0, -prolateSpec.attenuationAtCutoffDb / 20.0);
            bool passbandOk = true;
            for (int i = 0; i <= 200; ++i)
            {
                double f = prolateSpec.cutoffHz * static_cast<double> (i) / 200.0;
                double resp = std::abs (responseAt (prolateResult.taps, f, prolateSpec.sampleRateHz));
                if (resp > 1.0 + 0.01 || resp < gainFloor - 0.02) passbandOk = false;
            }
            check (passbandOk, "[ProlateBasis] passband stays within [gainFloor, 1] across [0, cutoff]");

            std::printf ("  [ProlateBasis] taps=%d R_peak=%.3f%% E_ZC=%.3f%% T_0.1%%=%.4fms (Minimax at the same spec: see [FreeTransition] block above)\n",
                prolateResult.tapCount, prolateResult.temporal.rPeakPercent, prolateResult.temporal.eZcPercent, prolateResult.temporal.settlingMs);
        }

        // Default (Minimax) must be completely unaffected by the presence
        // of DesignMethod::ProlateBasis - re-running the exact same
        // FreeTransition spec used earlier in this file with an explicit
        // designMethod = Minimax must reproduce identical taps.
        {
            FilterSpec explicitMinimax { 192000.0, 20000.0, 0.50, 98.0 };
            explicitMinimax.stopbandMode = StopbandMode::FreeTransition;
            explicitMinimax.designMethod = DesignMethod::Minimax;
            auto a = designParametricFIR (explicitMinimax, maxTapCount);

            FilterSpec defaultMethod { 192000.0, 20000.0, 0.50, 98.0 };
            defaultMethod.stopbandMode = StopbandMode::FreeTransition;
            auto b = designParametricFIR (defaultMethod, maxTapCount);

            bool identical = a.taps.size() == b.taps.size();
            if (identical)
                for (std::size_t i = 0; i < a.taps.size(); ++i)
                    if (a.taps[i] != b.taps[i]) identical = false;
            check (identical, "[ProlateBasis] FilterSpec's default designMethod is Minimax - explicit and implicit Minimax specs produce identical taps");
        }
    }

    // --- FilterSpec::sidelobeDecayRatio (distance-dependent sidelobe
    // penalty - see ParametricFIR.h for the full rationale) ---
    {
        // Default (1.0) must be an exact no-op: identical taps to a spec
        // that never sets the field at all.
        {
            FilterSpec explicitFlat { 192000.0, 20000.0, 0.50, 98.0 };
            explicitFlat.stopbandMode = StopbandMode::FreeTransition;
            explicitFlat.sidelobeDecayRatio = 1.0;
            auto a = designParametricFIR (explicitFlat, maxTapCount);

            FilterSpec defaultDecay { 192000.0, 20000.0, 0.50, 98.0 };
            defaultDecay.stopbandMode = StopbandMode::FreeTransition;
            auto b = designParametricFIR (defaultDecay, maxTapCount);

            bool identical = a.taps.size() == b.taps.size();
            if (identical)
                for (std::size_t i = 0; i < a.taps.size(); ++i)
                    if (a.taps[i] != b.taps[i]) identical = false;
            check (identical, "[sidelobeDecayRatio] 1.0 is an exact no-op - identical taps to the default (unset) spec");
        }

        // A decayed design must still be a fully valid, spectrally
        // compliant filter (unity DC gain, passband/stopband met) - the
        // whole point of routing this through the same
        // dense-verify-and-refine pipeline as every other spec.
        {
            FilterSpec decayed { 192000.0, 20000.0, 0.50, 98.0 };
            decayed.stopbandMode = StopbandMode::FreeTransition;
            decayed.sidelobeDecayRatio = 0.3;
            auto r = designParametricFIR (decayed, maxTapCount);

            check (r.constraintsMet, "[sidelobeDecayRatio] a decayed design (0.3) still reports its own targets as met");

            double dcSum = r.taps.empty() ? 0.0 : std::accumulate (r.taps.begin(), r.taps.end(), 0.0);
            checkNear (dcSum, 1.0, 1.0e-6, "[sidelobeDecayRatio] a decayed design (0.3) keeps unity DC gain");

            const auto guard = freeTransitionGuardBand (decayed);
            const double guardWorst = denseWorstDbInBand (r.taps, decayed.sampleRateHz, guard.first, guard.second);
            check (guardWorst <= -decayed.stopbandRejectionDb + 0.5,
                "[sidelobeDecayRatio] a decayed design (0.3) still meets the guard-band rejection target");

            const double gainFloor = std::pow (10.0, -decayed.attenuationAtCutoffDb / 20.0);
            bool passbandOk = true;
            for (int i = 0; i <= 200; ++i)
            {
                double f = decayed.cutoffHz * static_cast<double> (i) / 200.0;
                double resp = std::abs (responseAt (r.taps, f, decayed.sampleRateHz));
                if (resp > 1.0 + 0.01 || resp < gainFloor - 0.02) passbandOk = false;
            }
            check (passbandOk, "[sidelobeDecayRatio] a decayed design (0.3) keeps the passband within [gainFloor, 1]");

            // The whole point: settling span should genuinely shorten
            // relative to the undecayed baseline at the same spec, not
            // just move R_peak around.
            FilterSpec undecayed = decayed;
            undecayed.sidelobeDecayRatio = 1.0;
            auto flat = designParametricFIR (undecayed, maxTapCount);
            check (r.temporal.settlingSampleSpan < flat.temporal.settlingSampleSpan,
                "[sidelobeDecayRatio] a decayed design (0.3) has a strictly shorter settling span than the undecayed baseline");
            std::printf ("  [sidelobeDecayRatio] decay=1.0: T_0.1%%=%d samples, R_peak=%.3f%%  |  decay=0.3: T_0.1%%=%d samples, R_peak=%.3f%%\n",
                flat.temporal.settlingSampleSpan, flat.temporal.rPeakPercent,
                r.temporal.settlingSampleSpan, r.temporal.rPeakPercent);
        }

        // A very aggressive decay ratio must never silently ship a
        // non-compliant filter - this is exactly the risk identified
        // during development (a quick refine-loop-free probe let an
        // aggressive ratio "cheat" the sparse constraint grid while
        // genuinely violating the passband/stopband spec between grid
        // points); the real engine's existing dense-verify-and-refine
        // loop in solveForStopEdge must catch this for ANY
        // sidelobeDecayRatio, the same way it already does for the
        // undecayed case, so a dense re-check here must come back clean
        // regardless of how small the ratio is.
        {
            FilterSpec aggressive { 192000.0, 20000.0, 0.50, 98.0 };
            aggressive.stopbandMode = StopbandMode::FreeTransition;
            aggressive.sidelobeDecayRatio = 0.001;
            auto r = designParametricFIR (aggressive, maxTapCount);
            check (r.constraintsMet, "[sidelobeDecayRatio] an aggressive decay ratio (0.001) still reports its own targets as met");

            const auto guard = freeTransitionGuardBand (aggressive);
            const double guardWorst = denseWorstDbInBand (r.taps, aggressive.sampleRateHz, guard.first, guard.second);
            check (guardWorst <= -aggressive.stopbandRejectionDb + 0.5,
                "[sidelobeDecayRatio] an aggressive decay ratio (0.001) is genuinely (densely) stopband-compliant, not just grid-compliant");

            const double gainFloor = std::pow (10.0, -aggressive.attenuationAtCutoffDb / 20.0);
            bool passbandOk = true;
            for (int i = 0; i <= 2000; ++i)
            {
                double f = aggressive.cutoffHz * static_cast<double> (i) / 2000.0;
                double resp = std::abs (responseAt (r.taps, f, aggressive.sampleRateHz));
                if (resp > 1.0 + 0.01 || resp < gainFloor - 0.02) passbandOk = false;
            }
            check (passbandOk, "[sidelobeDecayRatio] an aggressive decay ratio (0.001) is genuinely (densely) passband-compliant, not just grid-compliant");
        }
    }

    std::printf ("\n%d/%d checks passed\n", checksRun - checksFailed, checksRun);
    return checksFailed == 0 ? 0 : 1;
}
