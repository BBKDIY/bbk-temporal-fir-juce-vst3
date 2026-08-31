#pragma once

#include <array>
#include <cstddef>
#include <vector>

// BBK Parametric FIR: shared constants for the single parametric
// constrained-least-squares FIR lowpass (see ParametricFIR.h for the
// design method itself). This plugin used to host a fixed A/B/C/F
// loopback comparator at a hard-locked 192 kHz; it now auto-detects the
// host sample rate and exposes three live controls - cutoff, attenuation
// at cutoff, and minimum stopband rejection - each of which triggers a
// background redesign (see PluginProcessor.cpp).
//
// maxHalfLength bounds how many taps a design may use (2*maxHalfLength+1
// at most). Every design, however few taps it actually needed, is
// zero-padded out to that fixed length and centred on the same middle
// index (see padTapsToFixedLength() below), so the host-reported latency
// (maxHalfLength samples) never changes at runtime regardless of slider
// values or sample rate - only the number of non-zero taps does.
namespace bbk::detachedpole
{
constexpr int maxHalfLength = 80;
constexpr int maxTapCount = 2 * maxHalfLength + 1; // 161
constexpr int latencySamples = maxHalfLength;

// History buffer must cover maxTapCount taps read back from the current
// write position, with headroom for the write index wrapping mid-block.
constexpr int historyLength = 512;

constexpr double defaultCutoffHz = 20000.0;
constexpr double defaultAttenuationDb = 0.5;
constexpr double defaultStopbandRejectionDb = 98.0;

// The exact "near-flat passband, hard 98 dB stopband" operating point
// (192 kHz / 20 kHz cutoff / 94 kHz stopband edge, 19 taps) that
// reproduces the article's own published Case B numbers: -97.98 dB
// worst-case stopband, 3.33% R_peak, 0.61% E_ZC, 0.094 ms settling.
// The article never quantifies "near-flat" numerically (unlike Case
// C's explicit -0.50 dB), so this was found by sweeping
// attenuationAtCutoffDb and matching those published metrics - see
// the "Case B (calibrated near-flat)" block in
// Tests/DSPTestDetachedPole.cpp for the verification. Deliberately not
// user-typable via the attenuation slider: at this scale (thousandths
// of a dB) any host's own value display/automation rounding can
// silently land on a different, non-reproducible point (including
// exactly 0.0 dB, which has no feasible 19-tap solution at all - see
// ParametricFIR.h), so the "Amplitude Relaxation" toggle in the UI
// selects this exact constant instead of relying on typed precision.
//
// Recalibrated after the ParametricFIR.h peak-dominance fix (the LP
// previously left inner taps unconstrained relative to a[0], letting
// some tap counts settle on a spectrally-compliant but non-peak-at-
// centre solution; constraining |a[i]| <= a[0] for every inner tap
// shifted the feasibility landscape near this near-flat point). At
// this scale the achieved-metric-vs-attenuation curve has narrow good/
// bad islands rather than one smooth cliff, so this value was chosen
// by sweeping in 0.000001 dB steps and centring in the widest verified
// stable island (bad neighbours confirmed at 0.002525, 0.002550-0.002565,
// 0.002633-0.002634, 0.002638-0.002641 dB; this constant sits mid-island,
// ~0.00003 dB from either edge) rather than picked at an island boundary.
constexpr double caseBNearFlatAttenuationDb = 0.0026;

// The slider's own range covers every supported sample rate's Nyquist
// (up to 192 kHz -> 96 kHz), so the user can push the cutoff as high as
// "half the sampling rate of the material" at whichever rate is
// actually playing - trading passband extension for a more compact
// impulse response, as the paper's own transition-width sweep predicts
// (Section 2), rather than being capped at a fixed 40 kHz regardless of
// how much headroom the current sample rate actually offers. The
// engine (see PluginProcessor::specFromParameters) separately clamps
// the *effective* cutoff to stay safely below the current sample
// rate's real Nyquist, since the slider range itself cannot depend on
// the sample rate in a JUCE APVTS parameter without breaking host
// automation/state compatibility when the rate changes.
constexpr double minCutoffHz = 10000.0;
constexpr double maxCutoffHz = 96000.0;
constexpr double minAttenuationDb = 0.0;
constexpr double maxAttenuationDb = 3.0;
constexpr double minStopbandRejectionDb = 40.0;
constexpr double maxStopbandRejectionDb = 140.0;

// Build a fixed-length (maxTapCount), zero-padded, centre-aligned tap
// array from a design's own (possibly much shorter) symmetric taps. This
// is what keeps the group delay - and therefore the host-reported latency
// - constant across every slider and sample-rate combination: only the
// span of non-zero taps changes, never the array length or its centre.
inline std::array<double, maxTapCount> padTapsToFixedLength (const std::vector<double>& taps)
{
    std::array<double, maxTapCount> padded {};
    const int n = static_cast<int> (taps.size());
    const int m = (n - 1) / 2;
    const int offset = maxHalfLength - m;
    for (int i = 0; i < n; ++i)
        padded[static_cast<std::size_t> (offset + i)] = taps[static_cast<std::size_t> (i)];
    return padded;
}
}
