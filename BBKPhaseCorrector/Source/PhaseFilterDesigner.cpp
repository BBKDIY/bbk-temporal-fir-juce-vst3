#include "PhaseFilterDesigner.h"
#include "PhaseModel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace
{
    constexpr double pi = 3.141592653589793238462643383279502884;
    constexpr double twoPi = 2.0 * pi;

    double principalAngle (double radians)
    {
        return std::atan2 (std::sin (radians), std::cos (radians));
    }

    double interpolateLogFrequency (const std::array<double, 256>& values, double queryFrequencyHz)
    {
        using namespace bbk::phase_model;

        if (queryFrequencyHz <= bbk::phase_model::frequencyHz.front())
            return values.front();

        if (queryFrequencyHz >= bbk::phase_model::frequencyHz.back())
            return values.back();

        const auto it = std::lower_bound (bbk::phase_model::frequencyHz.begin(), bbk::phase_model::frequencyHz.end(), queryFrequencyHz);
        const auto upper = static_cast<std::size_t> (std::distance (bbk::phase_model::frequencyHz.begin(), it));
        const auto lower = upper - 1;

        const auto x0 = std::log (bbk::phase_model::frequencyHz[lower]);
        const auto x1 = std::log (bbk::phase_model::frequencyHz[upper]);
        const auto x = std::log (queryFrequencyHz);
        const auto t = (x - x0) / (x1 - x0);
        return values[lower] + t * (values[upper] - values[lower]);
    }

    void unwrapInPlace (std::vector<double>& phase)
    {
        if (phase.empty())
            return;

        for (std::size_t i = 1; i < phase.size(); ++i)
        {
            while (phase[i] - phase[i - 1] > pi)
                phase[i] -= twoPi;
            while (phase[i] - phase[i - 1] < -pi)
                phase[i] += twoPi;
        }
    }

    double raisedCosineUp (double x)
    {
        x = std::clamp (x, 0.0, 1.0);
        return 0.5 - 0.5 * std::cos (pi * x);
    }

    double raisedCosineDown (double x)
    {
        return 1.0 - raisedCosineUp (x);
    }

    void applyBandEdgeTapers (const std::vector<double>& frequency,
                              const std::vector<double>& rawCorrection,
                              std::vector<double>& effectiveCorrection)
    {
        // The correction is intentionally conservative at the measurement edges.
        // 20..30 Hz: fade in. 16..19 kHz: fade out.  Outside: identity.
        constexpr double lowStop = 20.0;
        constexpr double lowFull = 30.0;
        constexpr double highFull = 16000.0;
        constexpr double highStop = 19000.0;

        effectiveCorrection = rawCorrection;

        for (std::size_t i = 0; i < frequency.size(); ++i)
        {
            if (frequency[i] <= lowStop || frequency[i] >= highStop)
                effectiveCorrection[i] = 0.0;
        }

        // Low-frequency taper.  Use the locally unwrapped PRINCIPAL phase branch so
        // an arbitrary +/-360 degree offset cannot turn the taper into extra delay.
        std::vector<std::size_t> indices;
        std::vector<double> localPhase;

        for (std::size_t i = 0; i < frequency.size(); ++i)
        {
            if (frequency[i] > lowStop && frequency[i] < lowFull)
            {
                indices.push_back (i);
                localPhase.push_back (principalAngle (rawCorrection[i]));
            }
        }

        unwrapInPlace (localPhase);
        if (! localPhase.empty())
        {
            const auto shift = twoPi * std::round (localPhase.front() / twoPi);
            for (std::size_t j = 0; j < localPhase.size(); ++j)
            {
                localPhase[j] -= shift;
                const auto f = frequency[indices[j]];
                const auto weight = raisedCosineUp ((f - lowStop) / (lowFull - lowStop));
                effectiveCorrection[indices[j]] = weight * localPhase[j];
            }
        }

        indices.clear();
        localPhase.clear();

        // High-frequency taper, using the branch nearest zero at the OUTER edge.
        for (std::size_t i = 0; i < frequency.size(); ++i)
        {
            if (frequency[i] > highFull && frequency[i] < highStop)
            {
                indices.push_back (i);
                localPhase.push_back (principalAngle (rawCorrection[i]));
            }
        }

        unwrapInPlace (localPhase);
        if (! localPhase.empty())
        {
            const auto shift = twoPi * std::round (localPhase.back() / twoPi);
            for (std::size_t j = 0; j < localPhase.size(); ++j)
            {
                localPhase[j] -= shift;
                const auto f = frequency[indices[j]];
                const auto weight = raisedCosineDown ((f - highFull) / (highStop - highFull));
                effectiveCorrection[indices[j]] = weight * localPhase[j];
            }
        }
    }

    void fft (std::vector<std::complex<double>>& a, bool inverse)
    {
        const auto n = a.size();
        if (n == 0 || (n & (n - 1)) != 0)
            throw std::runtime_error ("FFT size must be a non-zero power of two");

        for (std::size_t i = 1, j = 0; i < n; ++i)
        {
            std::size_t bit = n >> 1;
            for (; j & bit; bit >>= 1)
                j ^= bit;
            j ^= bit;
            if (i < j)
                std::swap (a[i], a[j]);
        }

        for (std::size_t len = 2; len <= n; len <<= 1)
        {
            const auto angle = twoPi / static_cast<double> (len) * (inverse ? 1.0 : -1.0);
            const std::complex<double> wlen (std::cos (angle), std::sin (angle));

            for (std::size_t i = 0; i < n; i += len)
            {
                std::complex<double> w (1.0, 0.0);
                for (std::size_t j = 0; j < len / 2; ++j)
                {
                    const auto u = a[i + j];
                    const auto v = a[i + j + len / 2] * w;
                    a[i + j] = u + v;
                    a[i + j + len / 2] = u - v;
                    w *= wlen;
                }
            }
        }

        if (inverse)
        {
            const auto scale = 1.0 / static_cast<double> (n);
            for (auto& x : a)
                x *= scale;
        }
    }
}

namespace bbk
{
    int PhaseFilterDesigner::nextPowerOfTwo (int value)
    {
        int result = 1;
        while (result < value)
            result <<= 1;
        return result;
    }

    PhaseFilterDesigner::DesignResult PhaseFilterDesigner::design (double sampleRate, Target target, float correctionDepth)
    {
        if (sampleRate < 40000.0)
            throw std::runtime_error ("BBK Phase Corrector requires a sample rate of at least 40 kHz");

        // ~0.68 s total FIR duration.  At 48 kHz this is 32768 taps and 341.3 ms
        // pure-delay latency.  This length keeps the unit-magnitude interpolation
        // error negligible even around the 20..30 Hz transition.
        constexpr double targetDurationSeconds = 0.68;
        const auto fftSize = nextPowerOfTwo (static_cast<int> (std::ceil (sampleRate * targetDurationSeconds)));
        const auto latency = fftSize / 2;
        const auto bins = fftSize / 2 + 1;

        std::vector<double> frequency (static_cast<std::size_t> (bins));
        std::vector<double> rawCorrection (static_cast<std::size_t> (bins));

        for (int k = 0; k < bins; ++k)
        {
            const auto f = sampleRate * static_cast<double> (k) / static_cast<double> (fftSize);
            frequency[static_cast<std::size_t> (k)] = f;

            const auto measuredDeg = interpolateLogFrequency (phase_model::measuredPhaseDeg,
                                                               std::clamp (f, 20.0, 19000.0));
            const auto minimumDeg = interpolateLogFrequency (phase_model::minimumPhaseDeg,
                                                              std::clamp (f, 20.0, 19000.0));

            const auto correctionDeg = (target == Target::linearPhase)
                                     ? -measuredDeg
                                     : minimumDeg - measuredDeg;

            rawCorrection[static_cast<std::size_t> (k)] = correctionDeg * pi / 180.0;
        }

        std::vector<double> correction;
        applyBandEdgeTapers (frequency, rawCorrection, correction);

        // Uniformly scale the finished (tapered) correction curve toward
        // zero. This is a genuine reduction in how much phase shift is
        // introduced, not a headroom/gain trick - it directly reduces the
        // constructive-interference peak growth this whole plugin is
        // otherwise vulnerable to, at the cost of leaving more of the
        // measured phase error uncorrected. Every bin's zero-crossing stays
        // exactly where it was; only the excursion around it shrinks, which
        // is what was asked for ("small rotation around this zero point").
        // depth=0 collapses every bin to the pure delayPhase term added
        // below, i.e. an exact identity allpass (matching BYPASS's matched
        // delay) with no correction at all.
        const auto depth = static_cast<double> (std::clamp (correctionDepth, 0.0f, 1.0f));
        if (depth != 1.0)
            for (auto& c : correction)
                c *= depth;

        std::vector<std::complex<double>> spectrum (static_cast<std::size_t> (fftSize), { 0.0, 0.0 });

        for (int k = 0; k < bins; ++k)
        {
            const auto delayPhase = -twoPi * static_cast<double> (k) * static_cast<double> (latency)
                                  / static_cast<double> (fftSize);
            const auto totalPhase = correction[static_cast<std::size_t> (k)] + delayPhase;
            spectrum[static_cast<std::size_t> (k)] = std::polar (1.0, totalPhase);
        }

        // Real-valued FIR requires conjugate symmetry.  DC and Nyquist are real;
        // the band-edge taper makes their correction phase exactly zero.
        spectrum[0] = { 1.0, 0.0 };
        spectrum[static_cast<std::size_t> (fftSize / 2)] = { (latency & 1) ? -1.0 : 1.0, 0.0 };

        for (int k = 1; k < fftSize / 2; ++k)
            spectrum[static_cast<std::size_t> (fftSize - k)] = std::conj (spectrum[static_cast<std::size_t> (k)]);

        fft (spectrum, true);

        DesignResult result;
        result.fftSize = fftSize;
        result.latencySamples = latency;
        result.impulse.resize (static_cast<std::size_t> (fftSize));

        for (int i = 0; i < fftSize; ++i)
            result.impulse[static_cast<std::size_t> (i)] = static_cast<float> (spectrum[static_cast<std::size_t> (i)].real());

        return result;
    }
}
