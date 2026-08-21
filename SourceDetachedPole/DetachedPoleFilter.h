#pragma once

#include <array>

// BBK Detached Pole: reproduces "Case C" from the accompanying article's
// Part Two, "A Practical Challenge to the FIR Frontier: The Detached Real
// Pole" - a 19-tap Type-I linear-phase FIR (Parks-McClellan, 20 kHz
// passband / 76 kHz stopband edges, unity DC gain) cascaded with a single
// first-order real IIR pole whose cut-off (47 kHz) is deliberately
// decoupled from the FIR's own passband edge rather than tied to it. This
// combination reaches lower peak-sidelobe amplitude and total ringing
// energy, and a shorter response duration, than any pure-FIR design at
// this article's tested tap counts - at the cost of a small (~0.5 dB)
// passband droop.
//
// Found by the article's own bounded grid search over passband edge and
// pole cut-off (Part Two Methodology, "Case C was located by a grid
// search over passband edge fp ... and real-pole cut-off fc ..."),
// reproduced independently here via scipy.signal.remez(19, [0, 20000,
// 76000, 96000], [1, 0], weight=[1, 1], fs=192000) for the FIR stage and
// scipy.signal.bilinear with frequency pre-warping for the pole stage,
// and verified to match the article's own published Case C figures
// (Rpeak=9.251%, EZC=1.552%, duration=15 samples/0.078 ms, worst-case
// stopband -116.17 dB, droop -0.50 dB at 20 kHz) to within numerical
// precision.
namespace bbk::detachedpole
{
constexpr int sampleRateHz = 192000;
constexpr int firTapCount = 19;
constexpr int firCentreIndex = 9; // (firTapCount - 1) / 2

constexpr double passbandEdgeHz = 20000.0;
constexpr double stopbandEdgeHz = 76000.0;
constexpr double poleCutoffHz = 47000.0;

// 19-tap Type-I linear-phase FIR stage, unity DC gain.
inline const std::array<double, firTapCount>& firTaps()
{
    static const std::array<double, firTapCount> taps = { {
        0.0007140731720717181,
        -3.996346537482728e-06,
        -0.005516281247398584,
        1.949491617795857e-05,
        0.023137533212360898,
        -4.955388278021481e-05,
        -0.07462639836592817,
        8.22815332045434e-05,
        0.306292317743205,
        0.4999010585312485,
        0.306292317743205,
        8.22815332045434e-05,
        -0.07462639836592817,
        -4.955388278021481e-05,
        0.023137533212360898,
        1.949491617795857e-05,
        -0.005516281247398584,
        -3.996346537482728e-06,
        0.0007140731720717181
    } };
    return taps;
}

// First-order real-pole IIR stage: y[n] = poleB0*w[n] + poleB1*w[n-1] -
// poleA1*y[n-1], where w[n] is the FIR stage's own output. Designed from
// an analogue H(s) = wc/(s+wc) at poleCutoffHz, frequency pre-warped
// (wc_prewarped = 2*Fs*tan(pi*fc/Fs)) and bilinear-transformed at
// sampleRateHz - matches scipy.signal.bilinear with pre-warping, the same
// convention used throughout the accompanying article's Methodology.
constexpr double poleB0 = 0.4918180389323442;
constexpr double poleB1 = 0.4918180389323442;
constexpr double poleA1 = -0.01636392213531156;

// Verified metrics for this exact design, reproduced independently from
// the taps/pole coefficients above (see Tests/DSPTestDetachedPole.cpp)
// and matching the article's own published Case C figures.
constexpr double peakSidelobePercent = 9.251;
constexpr double totalRingingEnergyPercent = 1.552;
constexpr int durationSamples = 15;
constexpr double durationMicroseconds = durationSamples * 1.0e6 / static_cast<double> (sampleRateHz);
constexpr double worstStopbandLevelDb = -116.17;
constexpr double passbandDroopDbAt20k = -0.50;
constexpr double groupDelaySamples = 9.515; // mean across 0-20 kHz, flat to within +/-0.002 samples

// Latency reported to the host: the nearest integer to this design's own
// near-flat group delay (~9.515 samples across the passband). The dry
// (bypass) path is delayed by exactly this many samples so toggling
// bypass never clicks.
constexpr int latencySamples = 10;

// History buffer length: must cover the FIR's own tap count (19) and the
// dry-path delay (10), plus headroom. The pole's own state is tracked
// separately (it is IIR, not part of this buffer).
constexpr int historyLength = 32;
}
