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

constexpr double minCutoffHz = 1000.0;
constexpr double maxCutoffHz = 40000.0;
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
