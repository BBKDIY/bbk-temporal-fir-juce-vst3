// Offline sweep tool for the BBK Parametric FIR prebuilt filter bank.
// NOT part of the plugin build - a standalone throwaway-style tool (like
// this session's diagnostic probes) that calls the real design engine
// directly to characterize R_peak/T_0.1% across tap counts, so the
// results can be post-processed into a precomputed filter bank header.
//
// Usage:
//   sweep_bank <sampleRateHz> <attenuationDb> <mMin> <mMax> <perCandidateSeconds> <maxNonImprovingAfterFeasible> <outCsv>
//
// Appends one row per M to outCsv (creating it with a header row if it
// doesn't exist yet), skipping any (sampleRateHz,attenuationDb,M) already
// present so it is safe to re-invoke after a timeout/interruption. Stops
// early once maxNonImprovingAfterFeasible consecutive M's in a row have
// failed to beat the best feasible R_peak seen so far (0 disables early
// stopping - always run to mMax).

#include "ParametricFIR.h"

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <sstream>
#include <set>
#include <string>
#include <vector>

int main (int argc, char** argv)
{
    if (argc != 8)
    {
        std::fprintf (stderr, "usage: %s <sampleRateHz> <attenuationDb> <mMin> <mMax> <perCandidateSeconds> <maxNonImprovingAfterFeasible> <outCsv>\n", argv[0]);
        return 1;
    }

    const double sampleRateHz = std::atof (argv[1]);
    const double attenuationDb = std::atof (argv[2]);
    const int mMin = std::atoi (argv[3]);
    const int mMax = std::atoi (argv[4]);
    const double perCandidateSeconds = std::atof (argv[5]);
    const int maxNonImprovingAfterFeasible = std::atoi (argv[6]);
    const std::string outCsv = argv[7];

    // Normalise rate/atten to the exact text this program itself writes to
    // the CSV (see rateKey/attenKey below), NOT the raw argv text - "0.10"
    // on the command line must match a "0.1" already written to the file,
    // or resume/dedup silently fails to recognise its own prior output as
    // done and reruns (and re-appends!) everything from scratch.
    char rateKeyEarly[64];
    std::snprintf (rateKeyEarly, sizeof (rateKeyEarly), "%.6g", sampleRateHz);
    char attenKeyEarly[64];
    std::snprintf (attenKeyEarly, sizeof (attenKeyEarly), "%.6g", attenuationDb);
    const std::string rateKeyStr = rateKeyEarly;
    const std::string attenKeyStr = attenKeyEarly;

    // Resume support: read whatever rows already exist and skip them, and
    // reconstruct the running "best feasible R_peak" / non-improving
    // streak state from them so early-stop logic still works correctly
    // across a re-invocation after a timeout.
    std::set<std::string> done;
    bool haveFeasible = false;
    double bestRPeak = 1.0e300;
    int nonImproving = 0;
    bool stoppedEarly = false;
    {
        std::ifstream in (outCsv);
        std::string line;
        bool first = true;
        while (std::getline (in, line))
        {
            if (first) { first = false; continue; } // header
            std::stringstream ss (line);
            std::string rateStr, attenStr, mStr, tapStr, feasStr, wsbStr, rpkStr;
            std::getline (ss, rateStr, ',');
            std::getline (ss, attenStr, ',');
            std::getline (ss, mStr, ',');
            std::getline (ss, tapStr, ',');
            std::getline (ss, feasStr, ',');
            std::getline (ss, wsbStr, ',');
            std::getline (ss, rpkStr, ',');
            if (rateStr != rateKeyStr || attenStr != attenKeyStr)
                continue; // belongs to a different (rate,atten) pair sharing this file
            done.insert (rateStr + "," + attenStr + "," + mStr);
            const bool feasible = feasStr == "1";
            const double rPeak = std::atof (rpkStr.c_str());
            if (feasible)
            {
                if (! haveFeasible || rPeak < bestRPeak - 1.0e-9)
                {
                    bestRPeak = rPeak;
                    nonImproving = 0;
                }
                else
                {
                    ++nonImproving;
                }
                haveFeasible = true;
            }
            else if (haveFeasible)
            {
                ++nonImproving;
            }
        }
    }
    if (haveFeasible && maxNonImprovingAfterFeasible > 0 && nonImproving >= maxNonImprovingAfterFeasible)
        stoppedEarly = true;

    const bool needHeader = std::ifstream (outCsv).peek() == std::ifstream::traits_type::eof();
    std::ofstream out (outCsv, std::ios::app);
    if (needHeader)
        out << "sampleRateHz,attenuationDb,M,tapCount,feasible,worstStopbandDb,rPeakPercent,settlingSampleSpan,settlingMs,taps\n";

    bbk::parametric::FilterSpec spec;
    spec.sampleRateHz = sampleRateHz;
    spec.cutoffHz = 18500.0;
    spec.attenuationAtCutoffDb = attenuationDb;
    spec.stopbandRejectionDb = 95.0;
    spec.stopbandMode = bbk::parametric::StopbandMode::FreeTransition;
    spec.sidelobeDecayRatio = 1.0;

    char rateKey[64];
    std::snprintf (rateKey, sizeof (rateKey), "%.6g", sampleRateHz);
    char attenKey[64];
    std::snprintf (attenKey, sizeof (attenKey), "%.6g", attenuationDb);

    if (stoppedEarly)
    {
        std::printf ("Fs=%s atten=%s already stopped early on a previous run (nonImproving=%d >= %d) - nothing to do\n",
            rateKey, attenKey, nonImproving, maxNonImprovingAfterFeasible);
        return 0;
    }

    for (int M = mMin; M <= mMax; ++M)
    {
        char mKey[16];
        std::snprintf (mKey, sizeof (mKey), "%d", M);
        const std::string key = std::string (rateKey) + "," + attenKey + "," + mKey;
        if (done.count (key) != 0)
        {
            std::printf ("skip (already done): Fs=%s atten=%s M=%d\n", rateKey, attenKey, M);
            continue;
        }

        const auto t0 = std::chrono::steady_clock::now();
        const auto deadline = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration> (std::chrono::duration<double> (perCandidateSeconds));
        auto attempt = bbk::parametric::detail::attemptDesign (spec, M, deadline);
        const double elapsedS = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();

        const int N = 2 * M + 1;
        std::vector<double> taps (static_cast<std::size_t> (N), 0.0);
        const bool degenerate = attempt.a.empty() || attempt.a[0] == 0.0;
        if (! degenerate)
        {
            for (int m = 0; m <= M; ++m)
            {
                taps[static_cast<std::size_t> (M - m)] = attempt.a[static_cast<std::size_t> (m)];
                taps[static_cast<std::size_t> (M + m)] = attempt.a[static_cast<std::size_t> (m)];
            }
        }
        auto temporal = bbk::parametric::computeTemporalMetrics (taps, sampleRateHz);

        out << rateKey << "," << attenKey << "," << M << "," << N << ","
            << (attempt.feasible ? 1 : 0) << ","
            << attempt.worstStopbandDb << ","
            << attempt.rPeakPercent << ","
            << temporal.settlingSampleSpan << ","
            << temporal.settlingMs << ",\"";
        for (std::size_t i = 0; i < taps.size(); ++i)
        {
            if (i) out << ";";
            out << taps[i];
        }
        out << "\"\n";
        out.flush();

        std::printf ("Fs=%s atten=%s M=%d taps=%d feasible=%d worstSB=%.4fdB Rpeak=%.4f%% T01=%.4fms elapsed=%.2fs\n",
            rateKey, attenKey, M, N, attempt.feasible ? 1 : 0, attempt.worstStopbandDb, attempt.rPeakPercent, temporal.settlingMs, elapsedS);

        if (attempt.feasible)
        {
            if (! haveFeasible || attempt.rPeakPercent < bestRPeak - 1.0e-9)
            {
                bestRPeak = attempt.rPeakPercent;
                nonImproving = 0;
            }
            else
            {
                ++nonImproving;
            }
            haveFeasible = true;
        }
        else if (haveFeasible)
        {
            ++nonImproving;
        }

        if (haveFeasible && maxNonImprovingAfterFeasible > 0 && nonImproving >= maxNonImprovingAfterFeasible)
        {
            std::printf ("  -> stopping early: %d M's in a row without beating best R_peak=%.4f%%\n", nonImproving, bestRPeak);
            break;
        }
    }

    return 0;
}
