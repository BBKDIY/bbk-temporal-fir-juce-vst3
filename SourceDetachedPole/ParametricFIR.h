#pragma once

// Minimum-peak-sidelobe (minimax) FIR design engine, framework-independent
// (no JUCE dependency) so it can be unit-tested with plain g++ before use
// inside the plugin.
//
// This is a direct implementation of the article's own method for its
// Case B/Case C designs (Sections 3 and 8.3 of "Impulse-Response Ringing
// in Digital Reconstruction Filtering"), not a frequency-domain-only
// fit. An earlier version of this file used constrained least squares
// (CLS) with IRLS to enforce the stopband: that reliably met the
// requested passband/stopband *frequency-domain* numbers, but had no
// time-domain objective at all, and in practice produced badly ringing
// filters (measured directly: R_peak/E_ZC several times worse than even
// the article's plain equiripple baseline, on the article's own 192 kHz/
// 20 kHz/98 dB operating point) - meeting the spectral spec is
// necessary but nowhere near sufficient for the temporal-concentration
// benefit this whole plugin exists to demonstrate. That is a real,
// measured implementation gap, not a preference, so the design method
// itself changed:
//
//   - Exact Type-I symmetry makes the zero-phase amplitude response
//     A(f) = a[0] + 2*sum_{m=1}^{M} a[m]*cos(2*pi*f*m/Fs) linear in the
//     half-coefficients a[0..M] - and a[0..M] *are* the FIR's own
//     impulse-response samples (a[0] the center/peak tap, a[m] the
//     tap m steps either side), so a genuinely time-domain objective on
//     the taps is just as linear as a frequency-domain one.
//   - Passband [0, cutoffHz]: a band constraint, gainFloor <= A(f) <= 1,
//     where gainFloor = 10^(-attenuationAtCutoffDb/20) - not a
//     prescribed trajectory. This matches the article's own Case C
//     description exactly ("stays between 0 and -0.50 dB... the
//     transition is allowed to take the shape returned by the
//     optimization"), and Case A/B's near-flat passband is just the
//     same constraint with a very small attenuationAtCutoffDb.
//   - True stopband [stopEdge, Nyquist]: -eps <= A(f) <= eps, eps
//     = 10^(-stopbandRejectionDb/20) - a hard linear inequality, exactly
//     as before. stopEdge itself is unchanged: the fixed geometric rule
//     described further down (enforced width = passband width, with a
//     Kaiser-based and a near-zero-transition fallback), which already
//     reproduces the article's own 20/76 kHz edges at 192 kHz/20 kHz.
//   - Objective: minimize rho, the ratio (largest |tap| outside a
//     zero-crossing-bounded main lobe) / (center tap), subject to the
//     two constraints above plus |a[i]| <= rho*a[0] for every tap i
//     outside the main lobe. For a *fixed* trial rho this whole system
//     - passband band, stopband band, sidelobe bound, all linear in
//     a[0..M] and therefore in the reduced (null-space) coordinates
//     used for the DC=1 constraint - is a linear feasibility problem,
//     solved by a from-scratch two-phase simplex (detail::
//     solveLPFeasibility). Bisecting on rho finds the smallest value
//     that is still feasible, i.e. the true minimum achievable ringing
//     at this M/spec - not an approximation of it. The main-lobe
//     boundary is made self-consistent with the article's own method:
//     after each bisection converges, the boundary is recomputed from
//     the solution's own first sign change and the whole bisection is
//     redone if that moved, so the optimizer cannot "hide" energy just
//     outside a stale, guessed boundary.
//   - As with the stopband before: the bisection only ever samples a
//     finite grid, so after it converges the continuous response is
//     re-verified on a dense sweep and any missed violations are folded
//     back in as new grid points, repeating until the dense sweep is
//     clean (or a small round cap is hit, in which case the best
//     dense-verified result found is kept).
//
// The DC=1 equality constraint is handled the same way as before: the
// null-space method (build an orthonormal basis Z for the hyperplane
// e^T a = 1, e being the "sum to unity" row; a0 = e/(e^T e) is one
// particular solution; any a = a0 + Z*y satisfies the constraint for
// every y), reducing every constraint above to one on the free
// coordinates y, with one fewer variable than a[0..M] itself.
//
// Tap count (the FIR half-length M) is searched from small to large -
// one full minimax solve per M - until the result actually meets the
// requested stopband floor and passband band, or the tap-count cap is
// reached (in which case the best-effort result found so far is
// returned with constraintsMet = false).
//
// This is genuinely more expensive than the old CLS/IRLS approach - a
// true minimax result costs real bisection + simplex work, not one
// least-squares solve - but it only ever runs on a background thread
// when a slider or the sample rate changes, never on the audio thread.
// Measured cost: well under a second for easy specs (a small M works on
// the first try, e.g. the article's own 192 kHz/20 kHz/98 dB point),
// typically a few seconds when several M values must be tried, and up
// to tens of seconds in the most demanding cases (chiefly cutoff pushed
// close to Nyquist, which forces both a large M and many M attempts).
// Three wall-clock budgets bound the worst case rather than letting it run
// unbounded: 6 seconds (advisory, loop-control only) per stopEdge candidate
// inside attemptDesign, 180 seconds for the whole M-search in
// designParametricFIR (the caller may pass a shorter one - see its own
// parameter comment), and, as a hard backstop against a single simplex
// solve running away uninterrupted on slower hardware (solveLPFeasibility
// has no *implicit* limit of its own beyond a fixed iteration cap), an
// explicit deadline check inside that solve's own pivot loop, bound to the
// overall M-search budget rather than the tighter 6-second one - tying it
// to the tighter cap was tried and measured to abort perfectly good,
// still-converging solves that only needed a few seconds more, actively
// regressing a spec that the looser (no internal check at all) original
// code handled fine. Past any of these, the best (least-far-from-compliant)
// result found so far is returned with constraintsMet = false, the same
// signal used when the tap-count cap is hit. During a slider drag this
// means the audible filter can noticeably lag the slider and only catch up a few seconds
// after it stops moving, rather than following it in real time - a
// direct, accepted cost of computing the article's actual minimum-
// ringing result instead of an approximation of it.
// Validated directly against the article's own published Case B and
// Case C numbers at 192 kHz/19 taps/20-76 kHz (Section 8.4): this
// engine reaches 12.9% R_peak against the article's reported 14.33% for
// Case B, and 3.3% against 3.50% for Case C, while meeting the same
// ~98 dB stopband target - matching, and in these cases slightly
// beating, the article's own results from an independently-written
// solver. See Tests/DSPTestDetachedPole.cpp for the exact comparison.
//
// StopbandMode::FreeTransition is a second mode - now the plugin's fixed,
// only behaviour (see PluginProcessor.cpp::specFromParameters()), though
// FlatMask remains here in the engine since it is what the Case B/C
// validation above still checks directly against the article's own
// published numbers. Where FlatMask enforces -stopbandRejectionDb flat
// across the whole mirror band [Nyquist-cutoff, Nyquist] (the article's
// own Case A/B/C geometry), FreeTransition treats the *entire* [cutoff,
// Nyquist] span as one free transition zone, with -stopbandRejectionDb
// only enforced in a narrow guard band immediately below Nyquist (a few
// hundred Hz to a couple of kHz, not a single point - a literal one-point
// constraint is not numerically meaningful and doesn't bound the response
// just below it). This trades away the flat band's margin for better
// temporal concentration at the same tap count, at a real, accepted cost:
// most of the transition can sit far above -stopbandRejectionDb (often
// only 20-40 dB down) until very close to Nyquist. That is only safe when
// nothing between this plugin and final reconstruction can fold that
// near-Nyquist energy back into the audible band - any downstream
// nonlinearity (saturation, compression, dither, a further sample-rate
// conversion) can alias it straight back down. This is a deliberate,
// informed trade, made explicitly (see the discussion that led to it),
// not an oversight.

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace bbk::parametric
{

// See the top-of-file comment for the trade-off between these two modes.
enum class StopbandMode
{
    FlatMask,       // default: paper-faithful, -stopbandRejectionDb held flat across [Nyquist-cutoff, Nyquist] (or a Kaiser/narrow fallback)
    FreeTransition  // opt-in: [cutoff, Nyquist] is one free transition zone, -stopbandRejectionDb only enforced in a narrow guard band right at Nyquist
};

struct FilterSpec
{
    double sampleRateHz = 192000.0;
    double cutoffHz = 20000.0;
    double attenuationAtCutoffDb = 0.5;
    double stopbandRejectionDb = 98.0;
    StopbandMode stopbandMode = StopbandMode::FlatMask;

    // Distance-dependent penalty on the sidelobe/rho bound: outer tap i
    // (i steps from the main lobe boundary) is bounded by
    // rho*a[0]*sidelobeDecayRatio^i instead of the flat rho*a[0] used
    // when this is 1.0 (the default - byte-identical to the original
    // flat-envelope minimax with no special-casing needed, since
    // pow(1.0, anything) == 1.0). Values < 1.0 progressively tighten the
    // bound the farther a tap sits from the main lobe, concentrating
    // ringing closer to the centre followed by a quieter tail instead of
    // a flat sidelobe plateau extending to the tap boundary - directly
    // addressing the "ripples closer to the main lobe, then quiet"
    // shape requested during development. An orthogonal modifier to the
    // sidelobe constraint shared by the tryRho/solveForStopEdge
    // machinery below, not a separate design method itself.
    //
    // Measured directly (192 kHz/20 kHz/0.5 dB/98 dB, 19 taps) across
    // the plugin's actual dense-verify-and-refine pipeline (which a
    // quicker, refine-loop-free probe during development did NOT
    // include, and which is essential - without it, aggressive decay
    // ratios were found to let the LP satisfy only the sparse
    // constraint grid while genuinely violating the passband/stopband
    // spec between grid points, e.g. a 2.4% passband violation at
    // decayRatio=0.001 - the dense-verify-and-refine loop in
    // solveForStopEdge closes exactly that gap for any decayRatio, the
    // same way it already does for the undecayed case):
    //   1.0 (no decay): R_peak 0.6%,  T_0.1% 16 samples (unchanged baseline)
    //   0.5:             R_peak 4.0%,  T_0.1% 14 samples
    //   0.3:             R_peak 3.5%,  T_0.1% 10 samples (best measured trade-off)
    //   0.15:            R_peak 4.6%,  T_0.1% 10 samples (plateaus, no further gain)
    // Not monotonic and not a single "best" value for every spec/tap
    // count, which is why this is a live control rather than a fixed
    // constant - see PluginEditor.
    double sidelobeDecayRatio = 1.0;
};

// The paper's own time-domain concentration metrics (Section 8.1),
// computed directly from a Type-I FIR's tap array - for a finite
// impulse response the taps *are* the impulse response, so no
// simulation is needed. Definitions, verbatim from the paper:
//   - R_peak: locate the global peak, walk outward to the nearest sign
//     change on each side (the "zero-crossing-bounded main lobe"); the
//     largest absolute sample outside that lobe, divided by the peak
//     amplitude, expressed as a percentage.
//   - E_ZC: sum of squared samples outside the same boundary, divided
//     by total impulse-response energy, as a percentage.
//   - T_0.1%: the interval (converted to time via the sample rate) over
//     which the absolute impulse response remains above 0.1% of its
//     peak - i.e. first-to-last sample crossing that threshold.
//   - tau_g: group delay, (N-1)/(2*Fs) for an exact linear-phase FIR.
// Verified against the paper's own published Case C reference (Section
// 8.4): the exact published 19-tap coefficients reproduce the paper's
// stated 3.5007% R_peak, 0.4839% E_ZC, and an 18-sample-interval
// T_0.1% span (0.09375 ms at 192 kHz) - see Tests/DSPTestDetachedPole.cpp.
//
// centerTapPercent is not from the paper: it is the fraction of the
// filter's total (unity, by construction - see designParametricFIR) DC
// gain that arrives in the single centre/peak sample itself, as a
// percentage. A true non-oversampling DAC with no reconstruction filter
// at all has an impulse response that IS a single sample, so 100% of its
// output for a given input sample arrives at that one instant -
// centerTapPercent is how close any given design of ours gets to that
// same instantaneous delivery, versus spreading it across the rest of
// the impulse response to achieve the requested anti-aliasing. It is a
// direct, cheap-to-compute proxy for the "transients feel averaged/less
// punchy" perceptual trade-off inherent to any reconstruction filtering:
// lower values mean more of the original instant is smeared into
// neighbouring samples, higher values (closer to 100%) mean less of it
// is. This is a genuinely different question from R_peak/E_ZC above,
// which describe energy OUTSIDE the main lobe - centerTapPercent
// describes how much sits, specifically, in the one centre sample.
struct TemporalMetrics
{
    double rPeakPercent = 0.0;
    double eZcPercent = 0.0;
    double settlingMs = 0.0;
    int settlingSampleSpan = 0;
    double groupDelayMs = 0.0;
    double centerTapPercent = 0.0;
};

inline TemporalMetrics computeTemporalMetrics (const std::vector<double>& taps, double sampleRateHz)
{
    TemporalMetrics m;
    const int n = static_cast<int> (taps.size());
    if (n == 0) return m;

    int peakIdx = 0;
    double peakAbs = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double v = std::fabs (taps[static_cast<std::size_t> (i)]);
        if (v > peakAbs) { peakAbs = v; peakIdx = i; }
    }
    if (peakAbs <= 0.0) return m;

    // See TemporalMetrics::centerTapPercent's own comment above. Uses the
    // signed tap value (not peakAbs) deliberately - a well-formed unity-DC
    // lowpass always has a positive centre tap in practice, but this
    // should report the true signed reality rather than assume it.
    m.centerTapPercent = taps[static_cast<std::size_t> (peakIdx)] * 100.0;

    const double peakSign = (taps[static_cast<std::size_t> (peakIdx)] >= 0.0) ? 1.0 : -1.0;

    int lobeStart = 0;
    for (int i = peakIdx - 1; i >= 0; --i)
    {
        double s = taps[static_cast<std::size_t> (i)];
        if (s * peakSign < 0.0) { lobeStart = i + 1; break; }
        if (i == 0) lobeStart = 0;
    }
    int lobeEnd = n - 1;
    for (int i = peakIdx + 1; i < n; ++i)
    {
        double s = taps[static_cast<std::size_t> (i)];
        if (s * peakSign < 0.0) { lobeEnd = i - 1; break; }
        if (i == n - 1) lobeEnd = n - 1;
    }

    double totalEnergy = 0.0, outsideEnergy = 0.0, worstOutside = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double v = taps[static_cast<std::size_t> (i)];
        totalEnergy += v * v;
        if (i < lobeStart || i > lobeEnd)
        {
            outsideEnergy += v * v;
            worstOutside = std::max (worstOutside, std::fabs (v));
        }
    }

    m.rPeakPercent = (worstOutside / peakAbs) * 100.0;
    m.eZcPercent = (totalEnergy > 0.0) ? (outsideEnergy / totalEnergy) * 100.0 : 0.0;

    const double threshold = 0.001 * peakAbs; // 0.1% of peak
    int firstAbove = -1, lastAbove = -1;
    for (int i = 0; i < n; ++i)
    {
        if (std::fabs (taps[static_cast<std::size_t> (i)]) > threshold)
        {
            if (firstAbove < 0) firstAbove = i;
            lastAbove = i;
        }
    }
    m.settlingSampleSpan = (firstAbove >= 0) ? (lastAbove - firstAbove) : 0;
    m.settlingMs = (sampleRateHz > 0.0) ? (static_cast<double> (m.settlingSampleSpan) / sampleRateHz) * 1000.0 : 0.0;

    const int M = (n - 1) / 2;
    m.groupDelayMs = (sampleRateHz > 0.0) ? (static_cast<double> (M) / sampleRateHz) * 1000.0 : 0.0;

    return m;
}

struct DesignResult
{
    std::vector<double> taps;
    int tapCount = 0;
    bool constraintsMet = false;
    double achievedStopbandDb = 0.0;
    int designAttempts = 0;
    TemporalMetrics temporal;
};

namespace detail
{

// Two-phase simplex for linear feasibility: does there exist a *free*
// vector y (size n) such that, for every row i, dot(Arows[i], y) <= b[i]?
// Free variables are handled by the standard split y_j = yp_j - yn_j
// (yp, yn >= 0); each row gets a slack (already basic, if b >= 0 after
// sign-normalisation) or a surplus plus an artificial variable (if the
// row needed flipping, i.e. was effectively a >= constraint). Phase 1
// minimizes the sum of artificials; the system is feasible iff that
// minimum is ~0. Bland's rule (lowest-index entering column, lowest-
// index leaving basic variable on ties) is used throughout to guarantee
// termination - this is a dense tableau implementation, chosen for
// simplicity and ease of independent verification over raw speed, which
// is why the caller keeps each individual LP small (see attemptDesign
// below: a sparse grid refined only where a dense sweep finds a genuine
// violation, not a single very dense grid from the start).
struct LPFeasibilityResult
{
    bool feasible = false;
    std::vector<double> y;
};

inline LPFeasibilityResult solveLPFeasibility (const std::vector<std::vector<double>>& Arows, const std::vector<double>& b, int n,
                                                std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max())
{
    const int numRows = static_cast<int> (Arows.size());

    std::vector<std::vector<double>> rows (static_cast<std::size_t> (numRows));
    std::vector<double> rhs (static_cast<std::size_t> (numRows));
    std::vector<int> sign (static_cast<std::size_t> (numRows));

    for (int i = 0; i < numRows; ++i)
    {
        double bi = b[static_cast<std::size_t> (i)];
        std::vector<double> r = Arows[static_cast<std::size_t> (i)];
        if (bi < 0.0)
        {
            for (auto& v : r) v = -v;
            bi = -bi;
            sign[static_cast<std::size_t> (i)] = -1;
        }
        else
        {
            sign[static_cast<std::size_t> (i)] = 1;
        }
        rows[static_cast<std::size_t> (i)] = r;
        rhs[static_cast<std::size_t> (i)] = bi;
    }

    int numArtificial = 0;
    for (int i = 0; i < numRows; ++i)
        if (sign[static_cast<std::size_t> (i)] == -1) ++numArtificial;

    const int numYCols = 2 * n;
    const int numSlackCols = numRows;
    const int numCols = numYCols + numSlackCols + numArtificial;

    std::vector<std::vector<double>> T (static_cast<std::size_t> (numRows + 1), std::vector<double> (static_cast<std::size_t> (numCols + 1), 0.0));
    std::vector<int> basis (static_cast<std::size_t> (numRows));

    int artIdx = 0;
    for (int i = 0; i < numRows; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            T[static_cast<std::size_t> (i)][static_cast<std::size_t> (j)] = rows[static_cast<std::size_t> (i)][static_cast<std::size_t> (j)];
            T[static_cast<std::size_t> (i)][static_cast<std::size_t> (n + j)] = -rows[static_cast<std::size_t> (i)][static_cast<std::size_t> (j)];
        }
        int slackCol = numYCols + i;
        T[static_cast<std::size_t> (i)][static_cast<std::size_t> (slackCol)] = (sign[static_cast<std::size_t> (i)] == 1) ? 1.0 : -1.0;
        if (sign[static_cast<std::size_t> (i)] == -1)
        {
            int artCol = numYCols + numSlackCols + artIdx;
            T[static_cast<std::size_t> (i)][static_cast<std::size_t> (artCol)] = 1.0;
            basis[static_cast<std::size_t> (i)] = artCol;
            ++artIdx;
        }
        else
        {
            basis[static_cast<std::size_t> (i)] = slackCol;
        }
        T[static_cast<std::size_t> (i)][static_cast<std::size_t> (numCols)] = rhs[static_cast<std::size_t> (i)];
    }

    for (int j = numYCols + numSlackCols; j < numCols; ++j)
        T[static_cast<std::size_t> (numRows)][static_cast<std::size_t> (j)] = 1.0;

    for (int i = 0; i < numRows; ++i)
    {
        if (basis[static_cast<std::size_t> (i)] >= numYCols + numSlackCols)
        {
            for (int j = 0; j <= numCols; ++j)
                T[static_cast<std::size_t> (numRows)][static_cast<std::size_t> (j)] -= T[static_cast<std::size_t> (i)][static_cast<std::size_t> (j)];
        }
    }

    const double eps = 1.0e-9;
    const int maxIters = 20000;
    for (int iter = 0; iter < maxIters; ++iter)
    {
        // Checked every 64 pivots (not every one - steady_clock::now() has
        // real overhead and a single pivot is cheap) so that a single LP
        // solve can never run past the caller's deadline uninterrupted,
        // however slow the machine or however large this particular
        // tableau is. Before this check existed, the deadlines threaded
        // through attemptDesign/solveForStopEdge were only ever tested
        // BETWEEN whole simplex solves, not inside one - so one slow solve
        // at a large M could silently blow through the entire budget on
        // slower hardware (measured directly as the cause of CI-only
        // failures that did not reproduce locally). Bailing out here
        // returns the same "infeasible" sentinel as genuine infeasibility
        // (pivotRow < 0, below) - the caller already treats that as a
        // legitimate, expected outcome to degrade gracefully from, not a
        // crash or an ambiguous error.
        if ((iter & 63) == 0 && std::chrono::steady_clock::now() > deadline)
            return { false, {} };

        int pivotCol = -1;
        for (int j = 0; j < numCols; ++j)
        {
            if (T[static_cast<std::size_t> (numRows)][static_cast<std::size_t> (j)] < -eps) { pivotCol = j; break; }
        }
        if (pivotCol < 0) break;

        int pivotRow = -1;
        double bestRatio = 1.0e300;
        for (int i = 0; i < numRows; ++i)
        {
            double a = T[static_cast<std::size_t> (i)][static_cast<std::size_t> (pivotCol)];
            if (a > eps)
            {
                double ratio = T[static_cast<std::size_t> (i)][static_cast<std::size_t> (numCols)] / a;
                if (pivotRow < 0 || ratio < bestRatio - 1.0e-12
                    || (std::fabs (ratio - bestRatio) < 1.0e-12 && basis[static_cast<std::size_t> (i)] < basis[static_cast<std::size_t> (pivotRow)]))
                {
                    bestRatio = ratio;
                    pivotRow = i;
                }
            }
        }
        if (pivotRow < 0)
            return { false, {} };

        double pv = T[static_cast<std::size_t> (pivotRow)][static_cast<std::size_t> (pivotCol)];
        for (int j = 0; j <= numCols; ++j)
            T[static_cast<std::size_t> (pivotRow)][static_cast<std::size_t> (j)] /= pv;
        for (int i = 0; i <= numRows; ++i)
        {
            if (i == pivotRow) continue;
            double factor = T[static_cast<std::size_t> (i)][static_cast<std::size_t> (pivotCol)];
            if (std::fabs (factor) < 1.0e-15) continue;
            for (int j = 0; j <= numCols; ++j)
                T[static_cast<std::size_t> (i)][static_cast<std::size_t> (j)] -= factor * T[static_cast<std::size_t> (pivotRow)][static_cast<std::size_t> (j)];
        }
        basis[static_cast<std::size_t> (pivotRow)] = pivotCol;
    }

    double phase1Obj = -T[static_cast<std::size_t> (numRows)][static_cast<std::size_t> (numCols)];
    if (phase1Obj > 1.0e-6)
        return { false, {} };

    std::vector<double> x (static_cast<std::size_t> (numCols), 0.0);
    for (int i = 0; i < numRows; ++i)
        x[static_cast<std::size_t> (basis[static_cast<std::size_t> (i)])] = T[static_cast<std::size_t> (i)][static_cast<std::size_t> (numCols)];

    std::vector<double> y (static_cast<std::size_t> (n));
    for (int j = 0; j < n; ++j)
        y[static_cast<std::size_t> (j)] = x[static_cast<std::size_t> (j)] - x[static_cast<std::size_t> (n + j)];

    return { true, y };
}

// Amplitude response of a Type-I linear-phase FIR expressed via its
// "half coefficients" a[0..M]: A(f) = a[0] + 2 * sum_{m=1}^{M} a[m]*cos(2*pi*f*m/Fs).
// NOTE: this takes the *half*-coefficient form (length M+1), not the full
// symmetric tap array (length 2M+1) that DesignResult::taps returns - the
// two are not interchangeable.
inline double amplitudeResponse (const std::vector<double>& a, double freqHz, double sampleRateHz)
{
    const double w = 2.0 * M_PI * freqHz / sampleRateHz;
    double sum = a[0];
    for (std::size_t m = 1; m < a.size(); ++m)
        sum += 2.0 * a[m] * std::cos (w * static_cast<double> (m));
    return sum;
}

struct AttemptResult
{
    std::vector<double> a;
    bool feasible = false;
    double worstStopbandDb = 0.0;
    // Temporal-concentration quality scores for this candidate (see
    // computeTemporalMetrics below) - only meaningful when feasible is
    // true; used to pick the best among several compliant stopEdge/M
    // candidates instead of just the first one found (see attemptDesign's
    // candidate sweep and designParametricFIR's M-search).
    //
    // settlingSampleSpan (T_0.1%, in samples) - not rPeakPercent - is what
    // designParametricFIR's M-search actually compares candidate M's on:
    // a lower R_peak does not always mean faster settling. Measured
    // directly at the Case B operating point: the 25-tap candidate has a
    // BETTER R_peak than the paper's own 19-tap design (1.758% vs 3.33%)
    // but a WORSE (longer) settling time (0.125ms vs 0.094ms) - more taps
    // bought a smaller relative sidelobe ratio while still adding enough
    // extra above-threshold samples at the tail to lengthen the actual
    // settling window. Settling time is the metric this plugin exists to
    // minimize (see the top-of-file article reference), so the search
    // should optimize for it directly rather than for a proxy that can
    // move the wrong way. rPeakPercent is kept alongside it for display/
    // diagnostics (see PluginEditor.cpp) but no longer drives the
    // cross-M comparison itself.
    double rPeakPercent = 1.0e300;
    int settlingSampleSpan = INT_MAX;
};

// attemptDesign now sweeps several stopband-edge candidates per M (see
// the tail of this function) and, per the "search more thoroughly"
// decision, keeps searching past the first compliant one - overallDeadline
// (shared across every M and every candidate within it, set by the
// caller in designParametricFIR) bounds the total worst-case cost so a
// single M can never run away with the whole search budget.
//
// perCandidateSeconds is the separate, per-stopEdge-candidate cap used
// inside solveForStopEdge below (see its own comment - default 15s,
// unchanged for the live plugin and for every existing caller that
// doesn't pass this explicitly, e.g. designParametricFIR and the DSP
// test suite). It exists as a parameter (not just the hardcoded
// constant it used to be) because it turned out to matter for more than
// just latency: measured directly that on a slower/more contended CPU
// (this repo's own offline sweep tool, run in a shared sandbox), 15
// seconds is sometimes not enough for the grid-refinement loop to reach
// its actual converged optimum, and the run silently keeps whatever
// partially-refined candidate it had at the cutoff instead - genuinely
// worse (higher R_peak, sometimes even failing the stopband target)
// than the same spec/M solved with more wall-clock, not a different
// design. The live plugin doesn't need to change (it already budgets
// for real-time responsiveness, and the outer M-search's own patience/
// deadline logic already treats a capped-out candidate as "not this M,
// try another" correctly) - only offline, non-interactive regeneration
// of the prebuilt filter bank benefits from raising this past 15s.
inline AttemptResult attemptDesign (const FilterSpec& spec, int M, std::chrono::steady_clock::time_point overallDeadline,
                                     double perCandidateSeconds = 15.0)
{
    const int numVars = M + 1;
    const double Fs = spec.sampleRateHz;
    const double fc = spec.cutoffHz;
    const double nyquist = Fs / 2.0;
    const double eps = std::pow (10.0, -spec.stopbandRejectionDb / 20.0);
    const double gainFloor = std::pow (10.0, -spec.attenuationAtCutoffDb / 20.0);

    // Sparse starting grids, refined by dense-verify-and-inject inside
    // solveForStopEdge below (each LP solve costs real time, unlike the
    // old single QR solve, so keeping the *starting* grid small matters -
    // see the top-of-file comment for measured timings).
    const int pbPoints = std::max (10, 2 * M);
    std::vector<double> pbFreqsInit (static_cast<std::size_t> (pbPoints));
    for (int i = 0; i < pbPoints; ++i)
        pbFreqsInit[static_cast<std::size_t> (i)] = fc * static_cast<double> (i) / static_cast<double> (pbPoints - 1);

    auto cosRow = [&] (double f)
    {
        std::vector<double> row (static_cast<std::size_t> (numVars));
        const double w = 2.0 * M_PI * f / Fs;
        row[0] = 1.0;
        for (int m = 1; m <= M; ++m)
            row[static_cast<std::size_t> (m)] = 2.0 * std::cos (w * static_cast<double> (m));
        return row;
    };

    // Null-space method for the DC=1 equality constraint: e^T a = 1, where
    // e = [1,2,2,...,2]. Build a Householder reflector H with H*e parallel
    // to the first axis; since H is symmetric and its own inverse, columns
    // 2..numVars of H then form an orthonormal basis Z for the hyperplane
    // orthogonal to e (i.e. exactly the null space of the constraint), and
    // a0 = e/(e^T e) is one particular point satisfying e^T a0 = 1. Any
    // a = a0 + Z*y automatically satisfies the constraint for every y, so
    // substituting that in turns the constrained problem into an ordinary
    // unconstrained least squares of one fewer variable.
    std::vector<double> e (static_cast<std::size_t> (numVars));
    e[0] = 1.0;
    for (int m = 1; m <= M; ++m) e[static_cast<std::size_t> (m)] = 2.0;

    double eSq = 0.0;
    for (double v : e) eSq += v * v;
    std::vector<double> a0 (static_cast<std::size_t> (numVars));
    for (int i = 0; i < numVars; ++i) a0[static_cast<std::size_t> (i)] = e[static_cast<std::size_t> (i)] / eSq;

    double normE = std::sqrt (eSq);
    std::vector<double> u = e;
    double alpha = (e[0] >= 0.0) ? -normE : normE;
    u[0] -= alpha;
    double unorm2 = 0.0;
    for (double v : u) unorm2 += v * v;

    int reducedVars = numVars - 1;
    std::vector<double> Z (static_cast<std::size_t> (numVars) * static_cast<std::size_t> (reducedVars));
    for (int col = 1; col < numVars; ++col)
    {
        for (int row = 0; row < numVars; ++row)
        {
            double val = (row == col ? 1.0 : 0.0) - 2.0 * u[static_cast<std::size_t> (row)] * u[static_cast<std::size_t> (col)] / unorm2;
            Z[static_cast<std::size_t> (row) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (col - 1)] = val;
        }
    }

    // A "coefficient row": a[i] as a linear function of y (constant term
    // a0[i], since a = a0 + Z*y). Used for the sidelobe bound, which
    // constrains taps directly rather than a frequency-domain sample.
    auto indexToLinear = [&] (int i, std::vector<double>& outZ, double& outConstant)
    {
        outConstant = a0[static_cast<std::size_t> (i)];
        outZ.assign (static_cast<std::size_t> (reducedVars), 0.0);
        for (int k = 0; k < reducedVars; ++k)
            outZ[static_cast<std::size_t> (k)] = Z[static_cast<std::size_t> (i) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (k)];
    };

    // A frequency row A(f) as a linear function of y: constant term
    // dot(row, a0), coefficients row.Z.
    auto freqToLinear = [&] (double f, std::vector<double>& outZ, double& outConstant)
    {
        auto row = cosRow (f);
        outConstant = 0.0;
        for (int j = 0; j < numVars; ++j) outConstant += row[static_cast<std::size_t> (j)] * a0[static_cast<std::size_t> (j)];
        outZ.assign (static_cast<std::size_t> (reducedVars), 0.0);
        for (int k = 0; k < reducedVars; ++k)
        {
            double sum = 0.0;
            for (int j = 0; j < numVars; ++j)
                sum += row[static_cast<std::size_t> (j)] * Z[static_cast<std::size_t> (j) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (k)];
            outZ[static_cast<std::size_t> (k)] = sum;
        }
    };

    auto reconstruct = [&] (const std::vector<double>& y)
    {
        std::vector<double> a (static_cast<std::size_t> (numVars));
        for (int i = 0; i < numVars; ++i)
        {
            double sum = a0[static_cast<std::size_t> (i)];
            for (int k = 0; k < reducedVars; ++k)
                sum += Z[static_cast<std::size_t> (i) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (k)] * y[static_cast<std::size_t> (k)];
            a[static_cast<std::size_t> (i)] = sum;
        }
        return a;
    };

    // Builds the full LP (passband band + stopband band, optionally the
    // sidelobe bound too) for one trial rho and one main-lobe boundary,
    // and solves it. applySidelobe=false is used once per main-lobe
    // iteration to confirm the spectral requirements alone are feasible
    // at this M/stopEdge, independent of any ringing target.
    auto tryRho = [&] (const std::vector<double>& pbF, const std::vector<double>& sbF,
                        int mainLobeStart, double rho, bool applySidelobe) -> LPFeasibilityResult
    {
        std::vector<std::vector<double>> A;
        std::vector<double> b;
        A.reserve (static_cast<std::size_t> (2 * (static_cast<int> (pbF.size()) + static_cast<int> (sbF.size()) + (M - mainLobeStart + 1))));
        b.reserve (A.capacity());

        std::vector<double> z; double c;
        for (double f : pbF)
        {
            freqToLinear (f, z, c);
            A.push_back (z); b.push_back (1.0 - c);                 // A(f) <= 1
            std::vector<double> neg (z.size()); for (std::size_t k = 0; k < z.size(); ++k) neg[k] = -z[k];
            A.push_back (neg); b.push_back (c - gainFloor);         // A(f) >= gainFloor
        }
        for (double f : sbF)
        {
            freqToLinear (f, z, c);
            A.push_back (z); b.push_back (eps - c);                 // A(f) <= eps
            std::vector<double> neg (z.size()); for (std::size_t k = 0; k < z.size(); ++k) neg[k] = -z[k];
            A.push_back (neg); b.push_back (c + eps);               // A(f) >= -eps
        }
        if (applySidelobe)
        {
            std::vector<double> z0; double c0;
            indexToLinear (0, z0, c0);
            for (int i = mainLobeStart; i <= M; ++i)
            {
                // Distance-dependent tightening (see FilterSpec::
                // sidelobeDecayRatio) - a plain multiplier on the same
                // bound used before, 1.0 when the feature is unused so
                // this is an exact no-op by construction, not a special
                // case.
                const double w = std::pow (spec.sidelobeDecayRatio, static_cast<double> (i - mainLobeStart));
                std::vector<double> zi; double ci;
                indexToLinear (i, zi, ci);
                std::vector<double> up (zi.size()), lo (zi.size());
                for (std::size_t k = 0; k < zi.size(); ++k)
                {
                    up[k] = zi[k] - rho * w * z0[k];
                    lo[k] = -zi[k] - rho * w * z0[k];
                }
                A.push_back (up); b.push_back (rho * w * c0 - ci);  // a[i] - rho*w*a[0] <= 0
                A.push_back (lo); b.push_back (rho * w * c0 + ci);  // -a[i] - rho*w*a[0] <= 0
            }

            // Peak-dominance: a[0] must actually BE the peak, not just
            // have small sidelobes *outside* the self-consistent main
            // lobe. Nothing above constrains the taps *inside* that
            // boundary (indices 1..mainLobeStart-1) relative to a[0], so
            // for some M the LP could (and, verified directly, did) find
            // a solution where an inner same-sign tap grows larger than
            // a[0] while staying spectrally compliant - since that tap
            // sits inside the notional main lobe, the rho objective above
            // never sees it, so minimizing rho no longer means minimizing
            // the article's actual R_peak (which is defined from the
            // array's real global peak, not from a[0] by assumption).
            // Hard-bind every inner tap to a[0] (rho fixed at 1, not the
            // bisected value) to make that assumption an enforced fact
            // rather than an unchecked one.
            for (int i = 1; i < mainLobeStart; ++i)
            {
                std::vector<double> zi; double ci;
                indexToLinear (i, zi, ci);
                std::vector<double> up (zi.size()), lo (zi.size());
                for (std::size_t k = 0; k < zi.size(); ++k)
                {
                    up[k] = zi[k] - z0[k];
                    lo[k] = -zi[k] - z0[k];
                }
                A.push_back (up); b.push_back (c0 - ci);             // a[i] - a[0] <= 0
                A.push_back (lo); b.push_back (c0 + ci);             // -a[i] - a[0] <= 0
            }
        }
        // The safety-net deadline passed here is the FULL overallDeadline
        // (the whole M-search's budget, e.g. 90s in the test file or 180s
        // in the plugin) - deliberately NOT solveForStopEdge's own tighter,
        // local ~6-second cap. That local cap is advisory/loop-control only
        // (it decides when to stop bisecting or refining the grid, exactly
        // as validated before this safety net existed); using it as the
        // hard per-solve cutoff too was tried and measured to actively
        // regress a previously-succeeding hard case (a single solve that
        // genuinely needs ~8-10s to converge was aborted at 6s and came
        // back "infeasible" even though the overall search still had ample
        // time left). The real problem this net exists for is a solve that
        // is degenerate/pathological and would otherwise run for minutes -
        // bounding it against the whole search's own budget (rather than
        // one stopEdge candidate's slice of it) stops that runaway cost
        // without punishing ordinary, if slow-on-this-hardware, solves.
        return solveLPFeasibility (A, b, reducedVars, overallDeadline);
    };

    // Solve for one specific stopEdge choice (the boundary between the
    // free transition zone [fc, stopEdge] and the hard-enforced stopband
    // [stopEdge, Nyquist]). stopEdge itself is chosen by the caller below
    // - see the fixed geometric rule described at the top of this file.
    //
    // For this stopEdge: bisect on rho (largest sidelobe / center tap)
    // to find the smallest value that is still spectrally feasible,
    // re-deriving the main-lobe boundary from each solution and re-
    // bisecting if it moved (self-consistency, per the article's own
    // method), then dense-verify the continuous response and inject any
    // missed violations as new grid points, repeating a bounded number
    // of times. A wall-clock budget bounds worst-case latency for
    // pathological specs (e.g. cutoff pushed hard against Nyquist).
    auto solveForStopEdge = [&] (double stopEdge) -> AttemptResult
    {
        const int sbPoints = std::max (25, 4 * M);
        std::vector<double> sbFreqsInit (static_cast<std::size_t> (sbPoints));
        for (int i = 0; i < sbPoints; ++i)
            sbFreqsInit[static_cast<std::size_t> (i)] = stopEdge + (nyquist - stopEdge) * (static_cast<double> (i) + 0.5) / static_cast<double> (sbPoints);

        std::vector<double> curPb = pbFreqsInit, curSb = sbFreqsInit;
        int mainLobeStart = 1;
        std::vector<double> bestY;
        double bestRho = 1.0;
        bool everFeasible = false;

        // Whether the sidelobe-CONSTRAINED bisection below ever actually
        // confirmed a feasible rho, as opposed to falling back to the
        // no-sidelobe safety net (see bestY's own "safe fallback,
        // overwritten below if bisection succeeds" comment). That fallback
        // exists so a spectrally-compliant design is still returned when
        // time runs out mid-bisection, but it places NO bound at all on
        // relative sidelobe height - measured directly as the cause of a
        // real bug: under time pressure at a demanding M, the doubling
        // search for a feasible hi can itself eat the whole per-candidate
        // deadline before ever confirming one, leaving the raw
        // unconstrained solution as the final answer - which can have
        // near-100% R_peak, yet was still reported as a genuine "feasible"
        // (compliant) design once the cross-M search started comparing
        // candidates on R_peak directly. Gating the returned feasible flag
        // on this (see below) makes the M-search correctly treat "gave up
        // mid-shaping" the same as "not feasible here", so it keeps
        // looking for a properly-shaped design at another tap count
        // instead of silently accepting whatever the fallback produced.
        bool sidelobeBisectionSucceeded = false;

        // Per-candidate deadline. A prior experiment raising this (or
        // floating it up to whatever remained of overallDeadline) was
        // reverted because it made a demanding spec come back WORSE,
        // non-deterministically - but that was BEFORE the
        // sidelobeBisectionSucceeded gate above existed. At the time, a
        // bisection cut off mid-search fell back to bestY's unconstrained
        // no-sidelobe solution and that fallback was still reported
        // feasible=true, so extra time just meant more chances to grind
        // into that trap at a large, slow-to-shape M instead of moving on.
        // Now that a cut-off bisection is correctly reported infeasible
        // (forcing the M-search to keep looking rather than accept the
        // fallback), the actual failure mode measured on CI hardware is
        // different: for the spec behind the stopband-monotonicity bug,
        // round 0 alone took ~5.3s against this cap on the CI runner,
        // leaving no time to clear the remaining violations - so a
        // genuinely convergeable M was being cut off before it could
        // finish, not saved by a bad fallback. Raised from 6 to 15 seconds
        // (the live-plugin default) - still a hard, fixed PER-CANDIDATE
        // cap (not tied to overallDeadline) so total search time stays
        // bounded via the outer loop's own deadline and attempt-count
        // caps, just wide enough for round 0's LP-heavy setup cost plus a
        // few refinement rounds to actually finish converging - or, for
        // offline bank regeneration (see perCandidateSeconds's own
        // comment on attemptDesign above), wide enough for ALL
        // maxGridRounds rounds to finish regardless of how slow the
        // machine running it is.
        const auto deadline = std::min (overallDeadline, std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration> (std::chrono::duration<double> (perCandidateSeconds)));

        // Raised from 4: a diagnostic dump on the exact spec behind the
        // stopband-monotonicity bug (see designParametricFIR's own
        // comment) showed a specific M landing at -79.69dB after 4 rounds
        // when the target was -80.0dB - a genuine near-miss (0.31dB), not
        // a hard infeasibility, that a real design at that M could clear
        // with more grid-refinement rounds to keep chasing the last few
        // violating points. This is bounded by the same 6-second (or
        // remaining overallDeadline, if tighter) deadline above either
        // way via the loop's own "if (now() > deadline) break" - raising
        // the round count only lets a candidate that's already converging
        // well use more of that same time budget on more rounds, instead
        // of giving up on rounds before it runs out of actual time.
        const int maxGridRounds = 10;
        int gridRound = 0;
        for (; gridRound < maxGridRounds; ++gridRound)
        {
            if (std::chrono::steady_clock::now() > deadline)
                break;

            for (int mlIter = 0; mlIter < 2; ++mlIter)
            {
                auto noSidelobe = tryRho (curPb, curSb, mainLobeStart, 0.0, false);
                if (! noSidelobe.feasible)
                    return { std::vector<double> (static_cast<std::size_t> (numVars), 0.0), false, 0.0, 1.0e300, INT_MAX };
                everFeasible = true;
                if (bestY.empty()) { bestY = noSidelobe.y; bestRho = 1.0e300; } // safe fallback, overwritten below if bisection succeeds

                double lo = 0.0, hi = 1.0;
                while (! tryRho (curPb, curSb, mainLobeStart, hi, true).feasible && hi < 1.0e6)
                {
                    hi *= 2.0;
                    if (std::chrono::steady_clock::now() > deadline) break;
                }
                for (int it = 0; it < 10; ++it)
                {
                    double mid = 0.5 * (lo + hi);
                    if (tryRho (curPb, curSb, mainLobeStart, mid, true).feasible) hi = mid; else lo = mid;
                    if (std::chrono::steady_clock::now() > deadline) break;
                }
                auto final = tryRho (curPb, curSb, mainLobeStart, hi, true);
                if (final.feasible) { bestY = final.y; bestRho = hi; sidelobeBisectionSucceeded = true; }

                auto a = reconstruct (bestY);
                int newStart = 1;
                for (int i = 1; i <= M; ++i)
                {
                    if ((a[static_cast<std::size_t> (i)] >= 0.0) != (a[0] >= 0.0)) { newStart = i; break; }
                    newStart = M + 1;
                }
                if (newStart == mainLobeStart) break;
                mainLobeStart = newStart;
            }

            if (! everFeasible)
                break;

            auto a = reconstruct (bestY);
            std::vector<double> violPb, violSb;
            const int dense = 3000;
            for (int i = 0; i < dense; ++i)
            {
                double f = fc * static_cast<double> (i) / static_cast<double> (dense - 1);
                double resp = amplitudeResponse (a, f, Fs);
                if (resp > 1.0 + 1.0e-6 || resp < gainFloor - 1.0e-6)
                    violPb.push_back (f);
            }
            for (int i = 0; i < dense; ++i)
            {
                double f = stopEdge + (nyquist - stopEdge) * static_cast<double> (i) / static_cast<double> (dense - 1);
                double resp = std::fabs (amplitudeResponse (a, f, Fs));
                if (resp > eps * 1.002)
                    violSb.push_back (f);
            }
            if (violPb.empty() && violSb.empty())
                break;

            // Greedily thin the violation list before injecting: violPb/
            // violSb come from a linear dense sweep (fc/dense and (nyquist-
            // stopEdge)/dense resolution respectively), so a genuinely
            // violating REGION (not just a single frequency) shows up as a
            // long run of consecutive, near-identical entries - taking the
            // first 20/40 in sweep order can inject a cluster of points
            // spaced by only the dense-sweep's own tiny step (a fraction of
            // a Hz for a narrow FreeTransition guard band), rather than 20/
            // 40 points actually spread across the violating span. Measured
            // directly as the root cause of a real bug: those near-
            // duplicate, nearly-parallel constraint rows make the simplex
            // tableau numerically near-degenerate, and it can then report a
            // spurious "infeasible" for a problem that is easily feasible
            // (verified independently: the exact same M/spec solves at once
            // against a fresh, evenly-spaced dense grid). Enforcing a
            // minimum gap between injected points (scaled to the region's
            // own width, not the dense-sweep step) spreads the same point
            // budget across the actual violating span instead of burning it
            // on one tight cluster - both fixing the degeneracy and, as a
            // side effect, covering more of a wide violating region.
            const double pbMinGap = fc * 0.002;
            const double sbMinGap = (nyquist - stopEdge) * 0.002;
            int added = 0;
            double lastAdded = -1.0e300;
            for (double f : violPb)
            {
                if (added >= 20) break;
                if (f - lastAdded < pbMinGap) continue;
                curPb.push_back (f);
                lastAdded = f;
                ++added;
            }
            added = 0;
            lastAdded = -1.0e300;
            for (double f : violSb)
            {
                if (added >= 40) break;
                if (f - lastAdded < sbMinGap) continue;
                curSb.push_back (f);
                lastAdded = f;
                ++added;
            }
        }

        if (! everFeasible)
            return { std::vector<double> (static_cast<std::size_t> (numVars), 0.0), false, 0.0, 1.0e300 };

        auto a = reconstruct (bestY);
        double worstStopbandDb = -1.0e300;
        const int denseCheckPoints = 4000;
        bool sbCompliant = true;
        for (int i = 0; i < denseCheckPoints; ++i)
        {
            double f = stopEdge + (nyquist - stopEdge) * static_cast<double> (i) / static_cast<double> (denseCheckPoints - 1);
            double resp = std::fabs (amplitudeResponse (a, f, Fs));
            double db = 20.0 * std::log10 (std::max (resp, 1.0e-300));
            if (db > worstStopbandDb) worstStopbandDb = db;
            if (db > 20.0 * std::log10 (eps) + 0.2) sbCompliant = false;
        }
        bool pbCompliant = true;
        for (int i = 0; i < denseCheckPoints; ++i)
        {
            double f = fc * static_cast<double> (i) / static_cast<double> (denseCheckPoints - 1);
            double resp = amplitudeResponse (a, f, Fs);
            if (resp > 1.0 + 0.01 || resp < gainFloor - 0.02) pbCompliant = false;
        }

        // Full symmetric tap array, purely to score this candidate's
        // actual temporal concentration (R_peak, settling span) - meeting
        // the spectral spec is necessary but not sufficient for good
        // ringing behaviour (see top-of-file comment), and different
        // stopEdge choices meet the same spec with genuinely different
        // ringing/settling.
        std::vector<double> fullTaps (static_cast<std::size_t> (2 * M + 1));
        for (int m = 0; m <= M; ++m)
        {
            fullTaps[static_cast<std::size_t> (M - m)] = a[static_cast<std::size_t> (m)];
            fullTaps[static_cast<std::size_t> (M + m)] = a[static_cast<std::size_t> (m)];
        }
        const auto fullMetrics = computeTemporalMetrics (fullTaps, Fs);

        // sidelobeBisectionSucceeded is required alongside spectral
        // compliance (see its own comment above): without it, "feasible"
        // would include the raw no-sidelobe fallback, which is spectrally
        // fine but can have almost no ringing suppression (R_peak close to
        // 100%) - a real bug this fixes, not a hypothetical one (measured
        // directly on a demanding real-world spec where the fallback won
        // the cross-M R_peak comparison outright once nothing better was
        // ever tried). Rejecting it here as infeasible lets the M-search
        // correctly move on and look for a tap count where the shaping
        // actually completed, rather than accepting whatever shape the
        // fallback happened to produce.
        const bool feasible = sidelobeBisectionSucceeded && sbCompliant && pbCompliant;
        return { a, feasible, worstStopbandDb, fullMetrics.rPeakPercent, fullMetrics.settlingSampleSpan };
    };

    double totalAvailable = nyquist - fc;
    if (totalAvailable < 1.0) totalAvailable = 1.0;
    const double minSpan = totalAvailable * 0.02;

    // FreeTransition mode (see top-of-file comment): skip the candidate
    // sweep entirely. solveForStopEdge already treats [stopEdge,
    // Nyquist] as the only hard-enforced region and everything below
    // stopEdge as a free transition with no pointwise frequency
    // constraint of its own (only the passband band and the sidelobe/
    // rho ringing bound reach that far) - so this mode is just a single
    // solveForStopEdge call with stopEdge pushed to a narrow guard band
    // right at Nyquist, reusing the exact same solver. Guard width: at
    // least 200 Hz (so the LP row and dense-verify sweep stay
    // numerically meaningful, not a single point), at most 2 kHz or 3%
    // of the available [cutoff, Nyquist] span, whichever is smaller.
    if (spec.stopbandMode == StopbandMode::FreeTransition)
    {
        double guardWidth = std::min (2000.0, totalAvailable * 0.03);
        guardWidth = std::max (guardWidth, 200.0);
        guardWidth = std::min (guardWidth, totalAvailable); // never exceed the available span itself
        double freeStopEdge = nyquist - guardWidth;
        if (freeStopEdge < fc) freeStopEdge = fc;
        return solveForStopEdge (freeStopEdge);
    }

    // FlatMask mode: the original three geometrically-motivated candidates
    // (the paper's own fixed mirror rule, a Kaiser/Bellanger transition
    // estimate, and a near-zero-transition last resort), tried in that
    // order, returning as soon as one is itself dense-verified compliant -
    // unchanged from before this pass. FlatMask is not reachable from the
    // plugin itself (see specFromParameters(), which always uses
    // FreeTransition below) - it exists purely so Tests/
    // DSPTestDetachedPole.cpp can keep validating directly against the
    // source article's own published Case B/C numbers, which are pinned to
    // this exact mirror-width geometry (its documented "-stopbandRejectionDb
    // held flat across the WHOLE mirror band" contract). Letting a
    // "find the lowest R_peak among several widths" search loose here
    // would silently swap in a far narrower enforced region purely
    // because it rings less, breaking that contract and the reference
    // validation that depends on it - the "search more thoroughly"
    // improvement instead lives one level up, in designParametricFIR's own
    // M-search below, which benefits FreeTransition (the plugin's actual,
    // only mode) too and does not have this problem: a larger M under the
    // SAME stopEdge rule only ever adds design freedom, it never changes
    // which region is enforced.
    double mirrorEnforcedWidth = fc;
    mirrorEnforcedWidth = std::min (mirrorEnforcedWidth, totalAvailable * 0.6);
    mirrorEnforcedWidth = std::max (mirrorEnforcedWidth, totalAvailable * 0.05);
    const double mirrorStopEdge = nyquist - mirrorEnforcedWidth;

    auto attempt = solveForStopEdge (mirrorStopEdge);
    if (attempt.feasible)
        return attempt;
    AttemptResult best = attempt;

    double kaiserTransitionWidth = Fs * (spec.stopbandRejectionDb - 7.95) / (14.36 * static_cast<double> (M));
    if (kaiserTransitionWidth < 0.0) kaiserTransitionWidth = 0.0;
    double kaiserStopEdge = fc + kaiserTransitionWidth;
    if (kaiserStopEdge > nyquist - minSpan) kaiserStopEdge = nyquist - minSpan;
    if (kaiserStopEdge < fc) kaiserStopEdge = fc;

    if (std::fabs (kaiserStopEdge - mirrorStopEdge) > 1.0)
    {
        auto attempt2 = solveForStopEdge (kaiserStopEdge);
        if (attempt2.feasible)
            return attempt2;
        if (attempt2.worstStopbandDb < best.worstStopbandDb)
            best = attempt2;
    }

    double narrowStopEdge = fc + minSpan;
    if (narrowStopEdge < fc) narrowStopEdge = fc;
    if (std::fabs (narrowStopEdge - mirrorStopEdge) > 1.0 && std::fabs (narrowStopEdge - kaiserStopEdge) > 1.0)
    {
        auto attempt3 = solveForStopEdge (narrowStopEdge);
        if (attempt3.feasible)
            return attempt3;
        if (attempt3.worstStopbandDb < best.worstStopbandDb)
            best = attempt3;
    }

    return best;
}

} // namespace detail

// maxTapCount caps the FIR half-length search at (maxTapCount-1)/2. The
// plugin uses a fixed value here (see DetachedPoleFilter.h::maxHalfLength)
// so every design, regardless of how many taps it actually needed, can be
// zero-padded to the same fixed length and therefore reports the same
// host latency no matter which slider values are in use.
// overallDeadlineSeconds defaults to the plugin's own real budget (see
// the comment below), but is an explicit parameter - not a hardcoded
// constant - specifically so Tests/DSPTestDetachedPole.cpp can pass a
// much shorter one: that file calls this function roughly twenty times
// to verify correctness (unity DC gain, passband/stopband compliance,
// symmetry, relative comparisons between specs), none of which need the
// full thorough search to already be exercised - meeting the spec at
// all happens quickly, at a small M, regardless of the deadline - and it
// is meant to stay a fast pre-MSVC/JUCE CI gate, not itself take up to
// 19 * 180 seconds.
inline DesignResult designParametricFIR (const FilterSpec& spec, int maxTapCount = 161, double overallDeadlineSeconds = 180.0)
{
    DesignResult result;
    const int maxM = (maxTapCount - 1) / 2;

    int M = std::min (maxM, 9);
    detail::AttemptResult best;
    int bestM = M;
    bool foundFeasible = false;

    // How the search decides when to stop looking for a BETTER M once at
    // least one feasible one has been found (see the loop below). This was
    // previously a fixed relative window (try up to 1.35x the first
    // feasible M, then give up) - measured directly to cause a real
    // regression: loosening the stopband target (e.g. 95dB -> 80dB) makes
    // a much SMALLER M newly feasible, which shrinks this window and can
    // make the search stop before ever re-trying the larger M that gave
    // the better (lower-R_peak) result under the tighter target - even
    // though that exact same larger-M design remains fully valid (and
    // objectively better) under the looser target too, since a design
    // compliant with a deeper stopband requirement is trivially compliant
    // with a shallower one. Relaxing a constraint can only ever enlarge
    // the feasible set, so the best achievable result should never get
    // worse - the search now reflects that directly: instead of a window
    // sized off the first feasible M, it keeps extending as long as larger
    // M's keep meaningfully improving R_peak (see the comparison below),
    // and only gives up after a run of tries that fail to do so. The
    // shared overall deadline below remains the hard backstop against
    // runaway time on demanding specs, same as before.
    constexpr int maxNonImprovingAttempts = 3;
    int nonImprovingAttempts = 0;

    // Separate, harder cap: even while R_peak keeps *technically* clearing
    // the "meaningfully better" bar below, stop chasing it after this many
    // extra attempts past the first feasible M. Measured directly that a
    // loose spec (plenty of stopband headroom to spare) can keep finding
    // another >=10%-relative R_peak reduction for many consecutive larger
    // M's in a row - each one individually reasonable, but each also a
    // full LP solve at a growing tap count, and the run of them together
    // can still eat the whole per-spec deadline chasing steadily smaller
    // absolute gains. This bounds the worst case to a fixed, small number
    // of extra solves regardless of how persistently R_peak keeps
    // improving, while still comfortably covering the jump the original
    // monotonicity bug fix needs (first-feasible to the genuinely better
    // M is a handful of the search's own M-step increments apart).
    constexpr int maxExtraAttemptsAfterFeasible = 8;
    int extraAttemptsAfterFeasible = 0;

    // Overall wall-clock budget across the *whole* M-search. This search
    // now runs alone - Prolate/Peak-Energy have been removed, so there is
    // nothing else competing for the single background design thread -
    // and it only ever runs once per boundary change (sample rate,
    // cutoff, attenuation, stopband target, decay ratio), not on every
    // audio callback or even every UI frame. That trade (a longer wait
    // after the last slider move, in exchange for genuinely searching for
    // the best result rather than stopping at the first one that meets
    // the spec) is the whole point of this pass - see the "search more
    // thoroughly" decision. Past the deadline, the search stops and
    // returns the best result found so far, exactly like hitting the
    // tap-count cap - constraintsMet = false, not a crash or a silent
    // wrong answer.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration> (std::chrono::duration<double> (overallDeadlineSeconds));

    while (true)
    {
        auto attempt = detail::attemptDesign (spec, M, deadline);
        ++result.designAttempts;

        // attemptDesign() has its own explicit all-zero-taps sentinel for
        // "no candidate survived the grid-refinement loop at all" at this
        // M (see its own "return { std::vector<double>(numVars,0.0),
        // false, 0.0, 1.0e300 }" points) - a real design always has a
        // nonzero centre/DC coefficient, so a[0]==0.0 unambiguously flags
        // that sentinel, not a genuine (if non-compliant) result.
        const bool attemptDegenerate = attempt.a.empty() || attempt.a[0] == 0.0;
        const bool bestDegenerate = best.a.empty() || best.a[0] == 0.0;
        const bool wasFeasibleBefore = foundFeasible;
        if (wasFeasibleBefore)
            ++extraAttemptsAfterFeasible;

        if (attempt.feasible)
        {
            // Once at least one M has produced a spec-compliant design,
            // keep searching larger M values (rather than stopping here)
            // for one that's genuinely BETTER. Primary criterion is
            // R_peak (ringing) - the paper's own quality measure - not
            // settling span: a filter's T_0.1% settling span can never
            // exceed roughly half its own tap count (everything beyond
            // the last tap is exactly zero, trivially "settled"), so
            // comparing settling span *across different M* structurally
            // favours whichever M is smallest, regardless of how much it
            // is actually ringing relative to its own length - this is
            // what caused the exact regression this fixes (loosening a
            // stopband target let a much shorter, higher-ringing M win
            // outright: R_peak went from 6.24% at 65 taps/95dB to 9.75%
            // at 43 taps/80dB, even though the 65-tap/95dB design remains
            // fully compliant, and strictly better, at 80dB too - nothing
            // about relaxing that constraint should make the achievable
            // result worse). Settling span is now only a tie-breaker,
            // used when R_peak is close enough to be a wash - this is
            // what "shorter taps given the same other metrics" actually
            // means.
            //
            // The bar for "meaningfully better" scales with the current
            // best R_peak itself (an absolute floor of 0.05 points, or
            // 10% relative, whichever is larger) rather than a fixed tiny
            // absolute amount - measured directly that a fixed 0.05-point
            // floor let almost every larger M count as "improving" by a
            // sliver, since more taps very often buys at least a small
            // R_peak reduction even once the curve has genuinely
            // flattened. That kept resetting the patience counter below
            // and turned what should be a quick search into one that rode
            // every demanding spec's own deadline (each large-M solve
            // itself is not free - see attemptDesign's own comments) -
            // a real, measured regression (multiple specs in this test
            // suite alone went from single-digit-second designs to
            // hitting their full per-spec deadline). Requiring a
            // proportionally larger step before it counts keeps the fix
            // for the original monotonicity bug (that needed a 36%
            // relative jump, 9.75% to 6.24%, comfortably over this bar)
            // while letting the search give up quickly once further gains
            // are genuinely marginal.
            constexpr double rPeakImprovementFloorAbs = 0.05; // percentage points
            constexpr double rPeakImprovementRel = 0.10; // 10%
            bool improved = false;
            if (! foundFeasible)
            {
                best = attempt;
                bestM = M;
                improved = true;
            }
            else
            {
                const double requiredImprovement = std::max (rPeakImprovementFloorAbs, best.rPeakPercent * rPeakImprovementRel);
                const double rPeakDelta = best.rPeakPercent - attempt.rPeakPercent; // >0 => attempt is better
                if (rPeakDelta > requiredImprovement)
                {
                    best = attempt;
                    bestM = M;
                    improved = true;
                }
                else if (rPeakDelta > -requiredImprovement
                         && attempt.settlingSampleSpan < best.settlingSampleSpan)
                {
                    // Near-tie on R_peak (within that same scaled band,
                    // not just a fixed absolute amount) - use settling as
                    // the tie-break. Not counted as a "real" improvement
                    // for the patience counter below, so a string of
                    // near-identical R_peak values (which any small
                    // settling gain would otherwise reset) can't
                    // indefinitely postpone giving up the search.
                    best = attempt;
                    bestM = M;
                }
            }
            if (improved)
                nonImprovingAttempts = 0;
            else if (foundFeasible)
                ++nonImprovingAttempts;
            foundFeasible = true;
        }
        else if (! foundFeasible)
        {
            // No feasible M found yet: keep the best (least-far-from-
            // compliant) NON-degenerate attempt seen across the search so
            // far, not just whichever M was tried last. Unconditionally
            // overwriting on every iteration let a LARGER M's spurious
            // all-zero failure silently stomp an earlier M's perfectly
            // good design - this is what previously surfaced as Minimax/
            // Prolate going completely silent once a background redesign
            // completed, seen both at 44.1 kHz and at 192 kHz.
            if (best.a.empty()
                || (bestDegenerate && ! attemptDegenerate)
                || (! attemptDegenerate && ! bestDegenerate && attempt.worstStopbandDb < best.worstStopbandDb))
            {
                best = attempt;
                bestM = M;
            }
        }
        else if (foundFeasible)
        {
            // Already have a feasible result and this larger M's attempt
            // was NOT feasible (a spurious/degenerate result under time
            // pressure, or a genuinely infeasible M for a spec that is
            // only feasible in a narrow M range) - best simply stays as
            // the last known-good feasible design. This DOES count toward
            // nonImprovingAttempts: measured directly that leaving it
            // uncounted let a run of infeasible/degenerate larger M's
            // (which cost a full, sometimes slow, LP solve each - see
            // attemptDesign's own comments) pass for free against the
            // patience counter, so the search kept paying for several
            // more of them one M-step at a time instead of giving up
            // once it was clearly past the design's feasible range.
            ++nonImprovingAttempts;
        }

        // FlatMask-only safety valve: the "keep searching past the first
        // feasible M, prefer whichever has the lower R_peak" logic above
        // is only sound when every M's own "feasible" flag means compliance
        // over the SAME enforced-stopband width - true for FreeTransition
        // (a single fixed guard-band rule, unchanged by M - see attemptDesign's
        // own comment), but NOT true for FlatMask: attemptDesign's per-M
        // candidate sweep (mirror -> Kaiser -> narrow, see above) can fall
        // through to a much NARROWER candidate at one M than at another, and
        // a narrower enforced region is both easier to satisfy and rings/
        // settles less almost by construction - not because it is a
        // genuinely better full-width design. Comparing R_peak *across*
        // FlatMask M's can therefore let a narrow-candidate M with a
        // technically-true but much weaker "feasible" flag win purely on
        // that basis, silently reintroducing the exact "flat across the
        // WHOLE mirror band" contract violation already found and reverted
        // once for FlatMask's own per-M candidate selection (see attemptDesign's
        // comment) - just one level up, across M instead of within one M.
        // Measured directly: CI (not this development machine) hit exactly
        // this on the Case C validation spec - M=21's narrow fallback
        // candidate won the cross-M R_peak comparison over M=25's proper
        // mirror-band-compliant one, and the resulting "compliant" design
        // was only -19.75 dB over the paper's own 76-96 kHz band (vs. the
        // requested ~98 dB) - not a marginal miss, a completely different,
        // much narrower region being reported as compliant. FlatMask exists
        // purely for that paper validation (see its own top-of-file
        // comment) and is never reached by the plugin, so it loses nothing
        // by keeping the original, pre-"search more thoroughly" behaviour:
        // stop at the first feasible M, exactly as before this pass.
        if (foundFeasible && spec.stopbandMode == StopbandMode::FlatMask) break;

        if (M >= maxM) break;
        if (std::chrono::steady_clock::now() > deadline) break;
        if (foundFeasible && nonImprovingAttempts >= maxNonImprovingAttempts) break;
        if (foundFeasible && extraAttemptsAfterFeasible >= maxExtraAttemptsAfterFeasible) break;
        M = std::min (maxM, M + std::max (1, M / 6));
    }

    int N = 2 * bestM + 1;
    std::vector<double> taps (static_cast<std::size_t> (N));
    for (int m = 0; m <= bestM; ++m)
    {
        taps[static_cast<std::size_t> (bestM - m)] = best.a[static_cast<std::size_t> (m)];
        taps[static_cast<std::size_t> (bestM + m)] = best.a[static_cast<std::size_t> (m)];
    }
    result.taps = taps;
    result.tapCount = N;
    result.constraintsMet = foundFeasible;
    result.achievedStopbandDb = best.worstStopbandDb;
    result.temporal = computeTemporalMetrics (taps, spec.sampleRateHz);
    return result;
}

} // namespace bbk::parametric
