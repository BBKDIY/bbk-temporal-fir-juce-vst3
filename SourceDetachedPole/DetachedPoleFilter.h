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
// this scale the achieved-metric-vs-attenuation curve is made up of
// narrow, near-tied LP-degenerate islands rather than one smooth
// cliff - a first attempt at 0.0026 dB sat in a genuinely stable
// island under g++/Linux (verified at 0.000001 dB resolution) but
// landed in a *different*, worse island under the CI's real MSVC
// build (R_peak 5.08%/E_ZC 1.68% instead of 3.32%/0.61%), because
// the tie between islands is close enough that ordinary compiler
// floating-point differences (MSVC vs g++, different simplex pivot
// rounding) can flip which side of a tie the solver lands on. Rather
// than chase a compiler-specific knife edge, this value was instead
// chosen from a *wide*, verified-flat plateau spanning roughly
// 0.00214-0.00252 dB (matching R_peak=3.42%, E_ZC=0.64% at every
// 0.000005 dB step sampled across that whole span, no internal
// ties), and set to the middle of it for maximum margin against any
// platform's floating-point rounding.
constexpr double caseBNearFlatAttenuationDb = 0.0023;

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

// The "do nothing but delay" tap set: unity at the fixed centre index,
// zero everywhere else - a pure latencySamples-sample delay, no filtering
// at all. Used as a safe immediate placeholder when the sample rate
// changes: the real design for the new rate can legitimately take several
// seconds for some cutoff/rate combinations (a cutoff pushed close to a
// low sample rate's Nyquist demands a very narrow, hard-to-satisfy guard
// band - see ParametricFIR.h), far too long to block prepareToPlay, which
// hosts expect back in milliseconds. Rather than either blocking the host
// or briefly running a design computed for the *previous* sample rate
// (whose cutoff would land at the wrong frequency entirely once
// reinterpreted at the new rate), the plugin installs this identity
// pass-through immediately and hands the real design to the same
// background worker and crossfade mechanism already used for live slider
// changes - see PluginProcessor::prepareToPlay().
inline std::array<double, maxTapCount> identityTaps()
{
    std::array<double, maxTapCount> taps {};
    taps[static_cast<std::size_t> (maxHalfLength)] = 1.0;
    return taps;
}
}
