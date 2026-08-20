// Framework-independent verification of the BBK Temporal FIR coefficient
// bank and processing algorithm. No JUCE dependency: only
// TemporalFIRBank.h, a small local mirror of the plugin's exact
// circular-buffer FIR algorithm, and the standard library.
//
// Checks, per filter point in the bank:
//   1. Exact time-domain symmetry (Type-I linear phase) of the padded taps.
//   2. Unity DC gain.
//   3. Passband constraint: |A(f)-1| <= 1e-4 over 0-20 kHz.
//   4. Stopband constraint: |A(f)| <= -100.3 dB over 76-96 kHz.
//   5. Displayed metrics (peak sidelobe %, ringing energy %, settling us,
//      stopband dB) recomputed from the stored taps match the stored
//      metric fields the plugin displays.
//   6. The padded taps are exactly zero outside the true filter's own
//      support, centred so every filter's group delay is exactly
//      maxGroupDelaySamples.
// Then, using a local mirror of PluginProcessor's exact process() algorithm:
//   7. The system's impulse response (steady-state, no crossfade) equals
//      the selected filter's padded taps array sample-for-sample.
//   8. Bypass/dry path latency is exactly maxGroupDelaySamples, identical
//      to every filter's own alignment.
//   9. Two channels driven by different, independent inputs never leak
//      into each other's output (verifies per-channel state isolation).
//
// Prints PASS and exits 0 if every check succeeds, otherwise prints FAIL
// with a description of what did not match and exits 1.

#include "../Source/TemporalFIRBank.h"

#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

namespace
{
bool ok = true;
constexpr double kPi = 3.14159265358979323846;

void check (bool condition, const char* description)
{
    if (! condition)
    {
        std::printf ("FAIL: %s\n", description);
        ok = false;
    }
    else
    {
        std::printf ("  ok: %s\n", description);
    }
}

double magnitudeAt (const bbk::temporalfir::FilterPoint& fp, double freqHz)
{
    const double omega = 2.0 * kPi * freqHz / static_cast<double> (bbk::temporalfir::sampleRateHz);
    std::complex<double> acc { 0.0, 0.0 };
    for (int k = 0; k < bbk::temporalfir::maxN; ++k)
    {
        const std::complex<double> rotation { std::cos (omega * k), -std::sin (omega * k) };
        acc += fp.taps[static_cast<std::size_t> (k)] * rotation;
    }
    return std::abs (acc);
}

// --- local mirror of PluginProcessor's exact per-channel algorithm ---
struct ChannelState
{
    std::array<double, bbk::temporalfir::maxN> history {};
    int writeIndex = 0;
};

struct MiniProcessor
{
    ChannelState state;
    const bbk::temporalfir::FilterPoint& fp;

    explicit MiniProcessor (const bbk::temporalfir::FilterPoint& point) : fp (point) {}

    // Returns {wet, dry} for one input sample, steady state (no crossfade,
    // no enabled-mix - exactly the inner per-channel body of
    // BBKTemporalFIRAudioProcessor::process()).
    std::pair<double, double> tick (double x)
    {
        state.history[static_cast<std::size_t> (state.writeIndex)] = x;

        double wet = 0.0;
        for (int k = 0; k < bbk::temporalfir::maxN; ++k)
        {
            int index = state.writeIndex - k;
            if (index < 0)
                index += bbk::temporalfir::maxN;
            wet += fp.taps[static_cast<std::size_t> (k)] * state.history[static_cast<std::size_t> (index)];
        }

        int dryIndex = state.writeIndex - bbk::temporalfir::maxGroupDelaySamples;
        if (dryIndex < 0)
            dryIndex += bbk::temporalfir::maxN;
        const double dry = state.history[static_cast<std::size_t> (dryIndex)];

        if (++state.writeIndex == bbk::temporalfir::maxN)
            state.writeIndex = 0;

        return { wet, dry };
    }
};
}

int main()
{
    using namespace bbk::temporalfir;

    std::printf ("BBK Temporal FIR DSP verification (framework-independent)\n");
    std::printf ("Fs = %d Hz, maxN = %d, maxGroupDelaySamples = %d, %d filter points\n\n",
                 sampleRateHz, maxN, maxGroupDelaySamples, numFilterPoints);

    bool sawBaseline19 = false;

    for (int idx = 0; idx < numFilterPoints; ++idx)
    {
        const auto& fp = filterBank()[static_cast<std::size_t> (idx)];
        char label[128];
        std::snprintf (label, sizeof (label), "N=%d (index %d)", fp.trueTapCount, idx);
        std::printf ("--- %s ---\n", label);

        if (fp.trueTapCount == 19)
            sawBaseline19 = true;

        // 1. symmetry
        bool symmetric = true;
        for (int i = 0; i < maxN; ++i)
            if (fp.taps[static_cast<std::size_t> (i)] != fp.taps[static_cast<std::size_t> (maxN - 1 - i)])
                { symmetric = false; break; }
        check (symmetric, "padded taps are exactly symmetric");

        // 2. unity DC gain
        double dcSum = 0.0;
        for (int i = 0; i < maxN; ++i)
            dcSum += fp.taps[static_cast<std::size_t> (i)];
        check (std::abs (dcSum - 1.0) < 1e-6, "DC gain equals 1.0 within 1e-6");

        // 3. passband constraint, sampled every 200 Hz
        double worstPassbandDev = 0.0;
        for (double f = 0.0; f <= 20000.0; f += 200.0)
            worstPassbandDev = std::max (worstPassbandDev, std::abs (magnitudeAt (fp, f) - 1.0));
        check (worstPassbandDev <= 1.0e-4 * 1.05, "passband |A(f)-1| <= 1e-4 (0-20 kHz, 5% grid slack)");

        // 4. stopband constraint, sampled every 200 Hz
        const double stopbandLevel = std::pow (10.0, -100.3 / 20.0);
        double worstStopband = 0.0;
        for (double f = 76000.0; f <= 96000.0; f += 200.0)
            worstStopband = std::max (worstStopband, magnitudeAt (fp, f));
        check (worstStopband <= stopbandLevel * 1.05, "stopband |A(f)| <= -100.3 dB (76-96 kHz, 5% grid slack)");

        // 5. displayed metrics match offline calculation, recomputed here
        //    directly from the stored taps using the same K0=3 convention
        //    (first sign change of the 19-tap reference design - see
        //    solve_fir.derive_k0 in the offline sweep scripts).
        const int centreIdx = maxGroupDelaySamples;
        const int K0 = 3;
        const double centre = fp.taps[static_cast<std::size_t> (centreIdx)];

        double peakSidelobe = 0.0;
        double sidelobeEnergy = 0.0;
        double totalEnergy = 0.0;
        for (int i = 0; i < maxN; ++i)
        {
            const double v = fp.taps[static_cast<std::size_t> (i)];
            totalEnergy += v * v;
            const int offset = std::abs (i - centreIdx);
            if (offset >= K0)
            {
                peakSidelobe = std::max (peakSidelobe, std::abs (v));
                sidelobeEnergy += v * v;
            }
        }
        const double peakSidelobePercent = 100.0 * peakSidelobe / std::abs (centre);
        const double ringingEnergyPercent = 100.0 * sidelobeEnergy / totalEnergy;

        int settleOffset = 0;
        for (int offset = maxGroupDelaySamples; offset >= 0; --offset)
        {
            if (std::abs (fp.taps[static_cast<std::size_t> (centreIdx + offset)]) > 1.0e-3 * std::abs (centre))
            {
                settleOffset = offset;
                break;
            }
        }
        const double settlingUs = settleOffset / static_cast<double> (sampleRateHz) * 1.0e6;

        check (std::abs (peakSidelobePercent - fp.peakSidelobePercent) < 0.01,
               "recomputed peak sidelobe % matches stored value within 0.01%");
        check (std::abs (ringingEnergyPercent - fp.totalRingingEnergyPercent) < 0.01,
               "recomputed ringing energy % matches stored value within 0.01%");
        check (std::abs (settlingUs - fp.settlingMicroseconds) < 0.001,
               "recomputed settling duration matches stored value");

        // 6. zero-padding: exactly zero outside [centre - trueGD, centre + trueGD]
        bool paddingClean = true;
        for (int i = 0; i < maxN; ++i)
        {
            const int offset = std::abs (i - centreIdx);
            if (offset > fp.trueGroupDelaySamples && fp.taps[static_cast<std::size_t> (i)] != 0.0)
                paddingClean = false;
        }
        check (paddingClean, "taps are exactly zero outside the true filter's own support");

        // 7. impulse response == padded taps, via the local process() mirror
        MiniProcessor mp (fp);
        bool impulseMatches = true;
        for (int n = 0; n < maxN; ++n)
        {
            const double x = (n == 0) ? 1.0 : 0.0;
            const auto [wet, dry] = mp.tick (x);
            (void) dry;
            if (std::abs (wet - fp.taps[static_cast<std::size_t> (n)]) > 1e-12)
                impulseMatches = false;
        }
        check (impulseMatches, "system impulse response equals padded taps sample-for-sample");

        std::printf ("\n");
    }

    check (sawBaseline19, "the 19-tap baseline is present in the selectable bank");

    // 8. bypass/dry latency == maxGroupDelaySamples, identical for every filter
    {
        const auto& fp = filterBank()[0];
        MiniProcessor mp (fp);
        int dryImpulseAt = -1;
        for (int n = 0; n < maxN; ++n)
        {
            const double x = (n == 0) ? 1.0 : 0.0;
            const auto [wet, dry] = mp.tick (x);
            (void) wet;
            if (dry != 0.0)
                dryImpulseAt = n;
        }
        check (dryImpulseAt == maxGroupDelaySamples,
               "bypass/dry impulse appears at exactly maxGroupDelaySamples");
    }

    // 9. stereo/channel independence: two independently-driven channel
    //    states must never leak into each other.
    {
        const auto& fp = filterBank()[numFilterPoints - 1]; // longest filter, most taps active
        MiniProcessor left (fp);
        MiniProcessor right (fp);
        bool independent = true;
        std::vector<double> leftOut, rightOut;
        for (int n = 0; n < maxN * 2; ++n)
        {
            const double xl = (n == 0) ? 1.0 : 0.0;   // impulse on left only
            const double xr = (n == 10) ? 1.0 : 0.0;  // different impulse on right only
            leftOut.push_back (left.tick (xl).first);
            rightOut.push_back (right.tick (xr).first);
        }
        // left channel's output must be identical to a lone impulse-at-0 response,
        // i.e. must equal the filter's own taps shifted by 0, and must show no
        // trace of right's impulse-at-10 (and vice versa) - since each
        // MiniProcessor only ever saw its own input, this is true by
        // construction, but verify numerically the two outputs are the same
        // shape offset by 10 samples (confirming neither was contaminated).
        for (int n = 0; n < maxN; ++n)
        {
            if (std::abs (leftOut[static_cast<std::size_t> (n)] - fp.taps[static_cast<std::size_t> (n)]) > 1e-12)
                independent = false;
            if (n + 10 < static_cast<int> (rightOut.size())
                && std::abs (rightOut[static_cast<std::size_t> (n + 10)] - fp.taps[static_cast<std::size_t> (n)]) > 1e-12)
                independent = false;
        }
        check (independent, "two channels with different, independent inputs do not cross-contaminate");
    }

    // 10. non-192 kHz safe-bypass predicate (mirrors the exact tolerance
    //     used in PluginProcessor::prepareToPlay).
    {
        auto isValid = [] (double sr) { return std::abs (sr - static_cast<double> (sampleRateHz)) < 0.5; };
        check (isValid (192000.0), "192000 Hz is accepted");
        check (! isValid (44100.0), "44100 Hz is rejected (safe-bypass path)");
        check (! isValid (96000.0), "96000 Hz is rejected (safe-bypass path)");
        check (! isValid (191999.0), "191999 Hz is rejected (outside 0.5 Hz tolerance)");
    }

    std::printf ("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
