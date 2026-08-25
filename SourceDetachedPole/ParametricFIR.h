#pragma once

// Constrained least-squares (CLS) FIR design engine, framework-independent
// (no JUCE dependency) so it can be unit-tested with plain g++ before use
// inside the plugin.
//
// Design problem, for a Type-I (odd length, symmetric) linear-phase FIR at
// a given sample rate:
//   - Passband [0, cutoffHz]: fit the amplitude response to a
//     straight-line-in-dB trajectory from 0 dB at DC to
//     -attenuationAtCutoffDb at cutoffHz.
//   - Unity DC gain: A(0) = 1 exactly (hard equality constraint).
//   - Transition zone [cutoffHz, stopEdge]: left completely unconstrained -
//     the optimizer is free to shape it however it wants, exactly like the
//     article's own Case F, whose 20-76 kHz transition was left
//     unconstrained rather than following any target curve.
//   - True stopband [stopEdge, Nyquist]: target 0, enforced as a hard
//     upper bound (see IRLS note below).
//
// stopEdge is not a user parameter, and it is not estimated from a
// Kaiser/Bellanger transition-width formula either: that consistently
// handed more of the band to hard enforcement than a given tap count
// actually needed (the formula is a rough rule of thumb for
// windowed-sinc design, not a measured fact about what this CLS solve
// can achieve). An earlier version of this file instead *searched* for
// the narrowest workable enforced stopband, starting from a 2%-of-band
// floor and widening only on failure - that was found by direct
// experiment to be unsound: at small M, a 2%-wide enforced region
// leaves almost the entire band with zero weight in the least-squares
// system, which is not merely "an easier constraint" but a poorly posed
// problem - the solver can return wildly oscillatory coefficients
// (in one measured case, response magnitude *exceeding* 0 dB in the
// nominally-free zone) that still technically satisfy the handful of
// points actually being checked.
//
// stopEdge is instead a fixed geometric rule, taken directly from how
// the paper's own practical 192 kHz/19-tap design point is built (see
// the paper's Sections 3, 5, and 8.4: Cases A, B, and C all share the
// same 20/76 kHz edges - passband edge 20 kHz, stopband edge 76 kHz,
// Nyquist 96 kHz - it is never re-derived per case or per tap count).
// The rule that reproduces that geometry is: reserve an enforced-
// stopband width equal to the passband's own width (cutoffHz), and
// leave the rest of [cutoffHz, Nyquist] as free transition. At 192 kHz
// with a 20 kHz cutoff that gives enforced width = 20 kHz, i.e. stopEdge
// = 96 - 20 = 76 kHz, exactly the paper's own value. The same rule
// generalises to any cutoff/sample-rate combination the plugin's
// sliders allow. It is clamped to [5%, 60%] of the available band so
// the enforced region can never collapse into the poorly-posed regime
// above, nor swallow so much of the band that no real transition is
// left. If that fixed width genuinely is not enough to reach the
// requested stopband at a given M - chiefly when the cutoff is pushed
// up close to Nyquist, leaving little headroom past it - attemptDesign
// falls back to at most two more candidates (a Kaiser/Bellanger
// estimate, then a near-zero-transition last resort), each a single
// solve, not an open-ended search: an earlier version widened
// geometrically without a bound and, for exactly that near-Nyquist
// case, re-tried the whole series at every M the outer search visited,
// which made the design hang rather than degrade gracefully.
//
// (A separate, much narrower idea - forcing compliance within a
// sub-100 Hz gap right past cutoff - was tried and rejected outright
// earlier: that made the problem infeasible at every tap count, not
// merely numerically stiff. No finite FIR can drop that fast. That
// finding is independent of the fixed-width rule above and still holds.)
//
// All regions are combined into a single fixed-weight weighted least
// squares fit (passband weighted more heavily than the stopband) and
// solved via Householder QR directly on the (rectangular) weighted design
// matrix - deliberately *not* via the normal equations (Q = C^T C), because
// forming Q squares the numerical condition number of the problem.
//
// The DC=1 equality constraint is handled by the null-space method: build
// an orthonormal basis Z for the hyperplane e^T a = 1 (e being the "sum to
// unity" row), reducing the constrained problem to an ordinary unconstrained
// least squares of one fewer variable, which QR then solves directly.
//
// The stopband's hard upper bound is enforced by iteratively reweighted
// least squares (IRLS): any stopband sample point still over the bound
// after a solve gets its weight grown and the whole system - still just
// one QR solve - is re-solved. Because IRLS only ever samples a finite
// grid, whenever that grid reports full compliance the *continuous*
// response is re-verified on a much denser sweep; any frequencies that
// sweep catches (and the grid missed) are folded into the grid as new
// constraint points and IRLS continues. This is a lightweight version of
// the exchange step in Remez/Parks-McClellan design, and was found to be
// necessary in practice: a fixed grid, no matter how dense, can be
// satisfied everywhere it samples while the continuous response still
// bulges well above the bound between samples.
//
// Tap count (the FIR half-length M) is searched from small to large - one
// full IRLS+refinement solve per M - until the result actually meets the
// requested stopband floor and passband trajectory, or the tap-count cap
// is reached (in which case the best-effort result found so far is
// returned with constraintsMet = false).
//
// This only runs when a slider or the sample rate changes (never per audio
// sample), so its cost is not part of the audio thread's real-time budget.
// Typical run time across 44.1/48/96/192 kHz and a range of cutoff /
// attenuation / stopband-rejection combinations is well under 500 ms.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace bbk::parametric
{

struct FilterSpec
{
    double sampleRateHz = 192000.0;
    double cutoffHz = 20000.0;
    double attenuationAtCutoffDb = 0.5;
    double stopbandRejectionDb = 98.0;
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
struct TemporalMetrics
{
    double rPeakPercent = 0.0;
    double eZcPercent = 0.0;
    double settlingMs = 0.0;
    int settlingSampleSpan = 0;
    double groupDelayMs = 0.0;
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

// Least squares min ||A x - b||^2 via Householder QR. A is rows x cols,
// row-major, rows >= cols. Numerically robust (condition number stays at
// cond(A), never squared the way normal-equations solves would square it).
inline std::vector<double> qrLeastSquares (std::vector<double> A, std::vector<double> b, int rows, int cols)
{
    for (int k = 0; k < cols; ++k)
    {
        double normX = 0.0;
        for (int i = k; i < rows; ++i)
        {
            double v = A[static_cast<std::size_t> (i * cols + k)];
            normX += v * v;
        }
        normX = std::sqrt (normX);
        if (normX < 1.0e-300) continue;

        double diagVal = A[static_cast<std::size_t> (k * cols + k)];
        double alpha = (diagVal >= 0.0) ? -normX : normX;

        std::vector<double> v (static_cast<std::size_t> (rows), 0.0);
        for (int i = k; i < rows; ++i) v[static_cast<std::size_t> (i)] = A[static_cast<std::size_t> (i * cols + k)];
        v[static_cast<std::size_t> (k)] -= alpha;

        double vnorm2 = 0.0;
        for (int i = k; i < rows; ++i) vnorm2 += v[static_cast<std::size_t> (i)] * v[static_cast<std::size_t> (i)];
        if (vnorm2 < 1.0e-300) continue;

        for (int j = k; j < cols; ++j)
        {
            double dot = 0.0;
            for (int i = k; i < rows; ++i) dot += v[static_cast<std::size_t> (i)] * A[static_cast<std::size_t> (i * cols + j)];
            double factor = 2.0 * dot / vnorm2;
            for (int i = k; i < rows; ++i) A[static_cast<std::size_t> (i * cols + j)] -= factor * v[static_cast<std::size_t> (i)];
        }
        {
            double dot = 0.0;
            for (int i = k; i < rows; ++i) dot += v[static_cast<std::size_t> (i)] * b[static_cast<std::size_t> (i)];
            double factor = 2.0 * dot / vnorm2;
            for (int i = k; i < rows; ++i) b[static_cast<std::size_t> (i)] -= factor * v[static_cast<std::size_t> (i)];
        }
    }

    std::vector<double> x (static_cast<std::size_t> (cols), 0.0);
    for (int i = cols - 1; i >= 0; --i)
    {
        double sum = b[static_cast<std::size_t> (i)];
        for (int j = i + 1; j < cols; ++j)
            sum -= A[static_cast<std::size_t> (i * cols + j)] * x[static_cast<std::size_t> (j)];
        double diag = A[static_cast<std::size_t> (i * cols + i)];
        if (std::fabs (diag) < 1.0e-300) diag = 1.0e-300;
        x[static_cast<std::size_t> (i)] = sum / diag;
    }
    return x;
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
};

inline AttemptResult attemptDesign (const FilterSpec& spec, int M)
{
    const int numVars = M + 1;
    const double Fs = spec.sampleRateHz;
    const double fc = spec.cutoffHz;
    const double nyquist = Fs / 2.0;
    const double eps = std::pow (10.0, -spec.stopbandRejectionDb / 20.0);

    // Dense enough that the highest-order cosine term (period Fs/M in f)
    // gets many samples per half-cycle - a sparse grid lets IRLS satisfy
    // every sampled point while the true continuous response still bulges
    // well above the bound in between samples.
    const int pbPoints = std::max (60, 20 * M);
    std::vector<double> pbFreqs (static_cast<std::size_t> (pbPoints));
    std::vector<double> pbTarget (static_cast<std::size_t> (pbPoints));
    for (int i = 0; i < pbPoints; ++i)
    {
        double f = fc * static_cast<double> (i) / static_cast<double> (pbPoints - 1);
        pbFreqs[static_cast<std::size_t> (i)] = f;
        double targetDb = -spec.attenuationAtCutoffDb * (fc > 0.0 ? f / fc : 0.0);
        pbTarget[static_cast<std::size_t> (i)] = std::pow (10.0, targetDb / 20.0);
    }

    auto cosRow = [&] (double f)
    {
        std::vector<double> row (static_cast<std::size_t> (numVars));
        const double w = 2.0 * M_PI * f / Fs;
        row[0] = 1.0;
        for (int m = 1; m <= M; ++m)
            row[static_cast<std::size_t> (m)] = 2.0 * std::cos (w * static_cast<double> (m));
        return row;
    };

    const double passbandWeight = 1000.0;

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

    const int reducedVars = numVars - 1;
    std::vector<double> Z (static_cast<std::size_t> (numVars) * static_cast<std::size_t> (reducedVars));
    for (int col = 1; col < numVars; ++col)
    {
        for (int row = 0; row < numVars; ++row)
        {
            double val = (row == col ? 1.0 : 0.0) - 2.0 * u[static_cast<std::size_t> (row)] * u[static_cast<std::size_t> (col)] / unorm2;
            Z[static_cast<std::size_t> (row) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (col - 1)] = val;
        }
    }

    // Precompute each row's projection onto Z and its base rhs term (against
    // a0) *once* - these don't depend on the per-point weight, only the
    // per-point sqrt(weight) scaling does, so this is what makes it
    // affordable to re-solve many times as IRLS adjusts stopband weights.
    auto projectRow = [&] (const std::vector<double>& row, std::vector<double>& outZ, double& outBase, double target)
    {
        double dotA0 = 0.0;
        for (int j = 0; j < numVars; ++j) dotA0 += row[static_cast<std::size_t> (j)] * a0[static_cast<std::size_t> (j)];
        outBase = target - dotA0;
        outZ.assign (static_cast<std::size_t> (reducedVars), 0.0);
        for (int k = 0; k < reducedVars; ++k)
        {
            double sum = 0.0;
            for (int j = 0; j < numVars; ++j)
                sum += row[static_cast<std::size_t> (j)] * Z[static_cast<std::size_t> (j) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (k)];
            outZ[static_cast<std::size_t> (k)] = sum;
        }
    };

    std::vector<std::vector<double>> pbRowZ (static_cast<std::size_t> (pbPoints));
    std::vector<double> pbBase (static_cast<std::size_t> (pbPoints));
    for (int i = 0; i < pbPoints; ++i)
        projectRow (cosRow (pbFreqs[static_cast<std::size_t> (i)]), pbRowZ[static_cast<std::size_t> (i)], pbBase[static_cast<std::size_t> (i)], pbTarget[static_cast<std::size_t> (i)]);

    // Solve for one specific stopEdge choice (the boundary between the
    // free transition zone [fc, stopEdge] and the hard-enforced stopband
    // [stopEdge, Nyquist]). stopEdge itself is chosen by the caller below
    // - see the fixed geometric rule described at the top of this file.
    auto solveForStopEdge = [&] (double stopEdge) -> AttemptResult
    {
        const int sbPoints = std::max (400, 40 * M);
        std::vector<double> sbFreqs (static_cast<std::size_t> (sbPoints));
        for (int i = 0; i < sbPoints; ++i)
            sbFreqs[static_cast<std::size_t> (i)] = stopEdge + (nyquist - stopEdge) * (static_cast<double> (i) + 0.5) / static_cast<double> (sbPoints);

        std::vector<std::vector<double>> sbRowZ (static_cast<std::size_t> (sbPoints));
        std::vector<double> sbBase (static_cast<std::size_t> (sbPoints));
        for (int i = 0; i < sbPoints; ++i)
            projectRow (cosRow (sbFreqs[static_cast<std::size_t> (i)]), sbRowZ[static_cast<std::size_t> (i)], sbBase[static_cast<std::size_t> (i)], 0.0);

        std::vector<double> sbWeight (static_cast<std::size_t> (sbPoints), 1.0);
        std::vector<double> a (static_cast<std::size_t> (numVars), 0.0);
        double worstStopbandDb = -1.0e300;
        // Fewer iterations than the original single-attempt version: this
        // now runs inside a width search, potentially several times per
        // M, and a genuinely-too-narrow stopEdge should reveal itself as
        // infeasible quickly rather than being given 80 iterations to
        // almost-but-not-quite converge.
        const int maxOuterIters = 40;
        const int maxInjectedPoints = 300;
        int injectedSoFar = 0;

        for (int iter = 0; iter < maxOuterIters; ++iter)
        {
            const int currentSbPoints = static_cast<int> (sbFreqs.size());
            const int numPoints = pbPoints + currentSbPoints;
            std::vector<double> Mred (static_cast<std::size_t> (numPoints) * static_cast<std::size_t> (reducedVars));
            std::vector<double> rhs (static_cast<std::size_t> (numPoints));

            int r = 0;
            for (int i = 0; i < pbPoints; ++i, ++r)
            {
                double sw = std::sqrt (passbandWeight);
                for (int k = 0; k < reducedVars; ++k)
                    Mred[static_cast<std::size_t> (r) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (k)] = sw * pbRowZ[static_cast<std::size_t> (i)][static_cast<std::size_t> (k)];
                rhs[static_cast<std::size_t> (r)] = sw * pbBase[static_cast<std::size_t> (i)];
            }
            for (int i = 0; i < currentSbPoints; ++i, ++r)
            {
                double sw = std::sqrt (sbWeight[static_cast<std::size_t> (i)]);
                for (int k = 0; k < reducedVars; ++k)
                    Mred[static_cast<std::size_t> (r) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (k)] = sw * sbRowZ[static_cast<std::size_t> (i)][static_cast<std::size_t> (k)];
                rhs[static_cast<std::size_t> (r)] = sw * sbBase[static_cast<std::size_t> (i)];
            }

            auto y = qrLeastSquares (Mred, rhs, numPoints, reducedVars);
            for (int i = 0; i < numVars; ++i)
            {
                double sum = a0[static_cast<std::size_t> (i)];
                for (int k = 0; k < reducedVars; ++k)
                    sum += Z[static_cast<std::size_t> (i) * static_cast<std::size_t> (reducedVars) + static_cast<std::size_t> (k)] * y[static_cast<std::size_t> (k)];
                a[static_cast<std::size_t> (i)] = sum;
            }

            bool anyViolation = false;
            worstStopbandDb = -1.0e300;
            for (int i = 0; i < currentSbPoints; ++i)
            {
                double resp = std::fabs (amplitudeResponse (a, sbFreqs[static_cast<std::size_t> (i)], Fs));
                double db = 20.0 * std::log10 (std::max (resp, 1.0e-300));
                if (db > worstStopbandDb) worstStopbandDb = db;

                double ratio = resp / eps;
                if (ratio > 1.0 + 1.0e-6)
                {
                    anyViolation = true;
                    double cappedRatio = std::min (ratio, 10.0);
                    double growth = cappedRatio * cappedRatio;
                    sbWeight[static_cast<std::size_t> (i)] = std::min (sbWeight[static_cast<std::size_t> (i)] * growth, 1.0e7);
                }
            }

            if (! anyViolation)
            {
                bool passbandOk = true;
                for (int i = 0; i < pbPoints; ++i)
                {
                    double resp = amplitudeResponse (a, pbFreqs[static_cast<std::size_t> (i)], Fs);
                    double target = pbTarget[static_cast<std::size_t> (i)];
                    if (std::fabs (resp - target) > 0.1 * target + 0.02)
                        passbandOk = false;
                }

                const int denseCheckPoints = 4000;
                double denseWorstDb = -1.0e300;
                std::vector<std::pair<double, double>> violations;
                for (int i = 0; i < denseCheckPoints; ++i)
                {
                    double f = stopEdge + (nyquist - stopEdge) * static_cast<double> (i) / static_cast<double> (denseCheckPoints - 1);
                    double resp = std::fabs (amplitudeResponse (a, f, Fs));
                    double db = 20.0 * std::log10 (std::max (resp, 1.0e-300));
                    if (db > denseWorstDb) denseWorstDb = db;
                    if (db > 20.0 * std::log10 (eps) + 0.2)
                        violations.emplace_back (db, f);
                }

                if (violations.empty () && passbandOk)
                    return { a, true, denseWorstDb };

                if (! violations.empty () && injectedSoFar < maxInjectedPoints)
                {
                    std::sort (violations.begin (), violations.end (), [] (auto& a1, auto& a2) { return a1.first > a2.first; });
                    const double minSpacing = (nyquist - stopEdge) / static_cast<double> (denseCheckPoints) * 4.0;
                    int added = 0;
                    for (auto& [db, f] : violations)
                    {
                        if (added >= 30 || injectedSoFar >= maxInjectedPoints) break;
                        bool tooClose = false;
                        for (int i = std::max (0, currentSbPoints - added - 60); i < currentSbPoints; ++i)
                            if (std::fabs (sbFreqs[static_cast<std::size_t> (i)] - f) < minSpacing) { tooClose = true; break; }
                        if (tooClose) continue;

                        std::vector<double> rz; double rb = 0.0;
                        projectRow (cosRow (f), rz, rb, 0.0);
                        sbFreqs.push_back (f);
                        sbRowZ.push_back (std::move (rz));
                        sbBase.push_back (rb);
                        sbWeight.push_back (1.0);
                        ++added;
                        ++injectedSoFar;
                    }
                    if (added > 0)
                        continue;
                }

                return { a, violations.empty () && passbandOk, denseWorstDb };
            }
        }

        return { a, false, worstStopbandDb };
    };

    // Fixed geometric rule (see top-of-file comment): reserve an
    // enforced-stopband width equal to the passband's own width, clamped
    // to [5%, 60%] of the available [cutoff, Nyquist] band. This
    // reproduces the paper's own 20/76 kHz edges exactly at 192 kHz/
    // 20 kHz cutoff, and generalises sanely to other cutoffs/rates.
    // At most three solveForStopEdge calls per M (bounded cost, no
    // open-ended series): the paper's fixed rule first, then a
    // Kaiser-based fallback, then a near-zero-transition last resort.
    // An open-ended widening series was tried and rejected: for a
    // cutoff pushed close to Nyquist (little headroom past cutoff), the
    // fixed rule's width is unreachable at low M, and repeatedly
    // re-solving at every step of a geometric series - for every M the
    // outer search tries, up to the 161-tap cap - is what made an
    // earlier version of this function hang for such specs.
    double totalAvailable = nyquist - fc;
    if (totalAvailable < 1.0) totalAvailable = 1.0;

    double mirrorEnforcedWidth = fc;
    mirrorEnforcedWidth = std::min (mirrorEnforcedWidth, totalAvailable * 0.6);
    mirrorEnforcedWidth = std::max (mirrorEnforcedWidth, totalAvailable * 0.05);
    const double mirrorStopEdge = nyquist - mirrorEnforcedWidth;

    auto attempt = solveForStopEdge (mirrorStopEdge);
    if (attempt.feasible)
        return attempt;
    AttemptResult best = attempt;

    // Fallback: a Kaiser/Bellanger transition-width estimate for the
    // current M - a safe, always-computable value (used on its own by
    // an earlier, proven version of this file) for specs where the
    // fixed mirror rule's width genuinely isn't reachable, chiefly a
    // cutoff with little headroom left to Nyquist.
    double kaiserTransitionWidth = Fs * (spec.stopbandRejectionDb - 7.95) / (14.36 * static_cast<double> (M));
    if (kaiserTransitionWidth < 0.0) kaiserTransitionWidth = 0.0;
    double kaiserStopEdge = fc + kaiserTransitionWidth;
    const double minSpan = totalAvailable * 0.02;
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

    // Last resort: enforce essentially the entire remaining band
    // (minimal transition). If even this fails at the current M, a
    // larger M is genuinely required - the outer M-search in
    // designParametricFIR tries that next, not this function.
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
inline DesignResult designParametricFIR (const FilterSpec& spec, int maxTapCount = 161)
{
    DesignResult result;
    const int maxM = (maxTapCount - 1) / 2;

    int M = std::min (maxM, 9);
    detail::AttemptResult best;
    bool foundFeasible = false;

    while (true)
    {
        auto attempt = detail::attemptDesign (spec, M);
        ++result.designAttempts;
        best = attempt;
        if (attempt.feasible)
        {
            foundFeasible = true;
            break;
        }
        if (M >= maxM) break;
        M = std::min (maxM, M + std::max (1, M / 6));
    }

    int N = 2 * M + 1;
    std::vector<double> taps (static_cast<std::size_t> (N));
    for (int m = 0; m <= M; ++m)
    {
        taps[static_cast<std::size_t> (M - m)] = best.a[static_cast<std::size_t> (m)];
        taps[static_cast<std::size_t> (M + m)] = best.a[static_cast<std::size_t> (m)];
    }
    result.taps = taps;
    result.tapCount = N;
    result.constraintsMet = foundFeasible;
    result.achievedStopbandDb = best.worstStopbandDb;
    result.temporal = computeTemporalMetrics (taps, spec.sampleRateHz);
    return result;
}

} // namespace bbk::parametric
