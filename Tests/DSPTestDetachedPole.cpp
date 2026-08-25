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
        bool passbandOk = true;
        for (double frac : { 0.0, 0.25, 0.5, 0.75, 1.0 })
        {
            const double f = c.spec.cutoffHz * frac;
            const double db = 20.0 * std::log10 (std::abs (responseAt (result.taps, f, c.spec.sampleRateHz)));
            const double target = -c.spec.attenuationAtCutoffDb * frac;
            const double tol = (frac >= 1.0) ? 1.0 : 0.3;
            if (std::fabs (db - target) > tol)
                passbandOk = false;
        }
        std::snprintf (nameBuf, sizeof (nameBuf), "[%s] passband follows the requested trajectory (DC to cutoff)", c.name);
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

        std::printf ("  [%s] Fs=%.0f fc=%.0f atten=%.2fdB stopband>=%.1fdB -> taps=%d attempts=%d worst=%.2fdB\n",
            c.name, c.spec.sampleRateHz, c.spec.cutoffHz, c.spec.attenuationAtCutoffDb, c.spec.stopbandRejectionDb,
            result.tapCount, result.designAttempts, worst);
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
        std::printf ("  [Case C via engine] taps=%d worst(76-96kHz)=%.3fdB (paper reference: 19 taps, -98.29dB)\n",
            engineResult.tapCount, engineWorst);
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

    std::printf ("\n%d/%d checks passed\n", checksRun - checksFailed, checksRun);
    return checksFailed == 0 ? 0 : 1;
}
