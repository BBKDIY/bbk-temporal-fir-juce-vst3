#pragma once

// Hand-written lookup helpers over the auto-generated PresetFilterBank.h -
// kept in a separate file so regenerating the bank (see PresetSweep/
// generate_bank_header.py) never risks clobbering this logic.

#include "PresetFilterBank.h"

#include <cmath>

namespace bbk::detachedpole::presetbank
{

// The bank was swept at 10 fixed attenuation-at-cutoff steps, 0.05 dB
// apart (see PresetSweep/sweep_bank.cpp's own invocation history / the
// generate_bank_header.py docstring) - snapping to the nearest one is
// what lets the Attenuation slider still move continuously in Default
// mode while every position resolves to one of the 10 precomputed
// results, instantly, rather than requiring an exact match.
inline double snapToNearestAttenuationStep (double attenuationDb)
{
    double best = 0.05;
    double bestDist = 1.0e300;
    for (int i = 1; i <= 10; ++i)
    {
        const double step = 0.05 * static_cast<double> (i);
        const double dist = std::fabs (attenuationDb - step);
        if (dist < bestDist) { bestDist = dist; best = step; }
    }
    return best;
}

// Finds the bank entry for the given sample rate (matched within 0.5 Hz -
// the same tolerance used elsewhere in this plugin for "is this the rate
// a design was actually computed for", see PluginProcessor.cpp) and the
// attenuation step nearest attenuationDb. Returns nullptr if sampleRateHz
// isn't one of the 7 rates the bank was swept for - an untabled/unusual
// host rate - so the caller can fall back to a live Custom-mode design
// instead of silently doing nothing.
inline const BankEntry* findEntry (double sampleRateHz, double attenuationDb)
{
    const double snappedAtten = snapToNearestAttenuationStep (attenuationDb);
    for (const auto& e : bank())
    {
        // Matched against nominalAttenuationDb (the fixed 0.05dB-spaced
        // step), NOT attenuationAtCutoffDb - a fine-tune pass over the
        // sweep can leave the entry's actual design point nudged slightly
        // off its nominal step (see PresetFilterBank.h's own comment), so
        // matching on the exact design value would break this lookup for
        // any entry that fine-tuning improved.
        if (std::fabs (e.sampleRateHz - sampleRateHz) <= 0.5
            && std::fabs (e.nominalAttenuationDb - snappedAtten) <= 1.0e-6)
            return &e;
    }
    return nullptr;
}

} // namespace bbk::detachedpole::presetbank
