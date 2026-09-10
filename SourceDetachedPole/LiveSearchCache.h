#pragma once

// Automatic memoisation of every completed Custom-mode live search (see
// PluginProcessor.cpp::run()), keyed by exact FilterSpec - deliberately
// separate from UserPresetOverrides.h, which only ever holds filters the
// user chose to keep via "Save as Default". This cache exists purely for
// speed: designParametricFIR() can legitimately take up to several minutes
// for a demanding spec (see ParametricFIR.h), so if you drag Attenuation
// back to a value you already tried - five minutes ago, or in an earlier
// session entirely - there is no reason to pay that cost again. Every
// point you've ever visited stays instant, not just the ones you
// explicitly kept as a default.
//
// One shared, per-install file, same reasoning as UserPresetOverrides.h: a
// spec you've already searched once should stay instant the next time you
// land on it, in any project or host.

#include "ParametricFIR.h"

#include <juce_core/juce_core.h>

#include <vector>

namespace bbk::detachedpole::searchcache
{

struct CacheEntry
{
    bbk::parametric::FilterSpec spec;
    int tapCount = 0;
    double achievedStopbandDb = 0.0;
    std::vector<double> taps; // natural length == tapCount, unity DC gain
};

// Kept in oldest-first insertion order in memory (see PluginProcessor.cpp,
// which evicts from the front once this many entries are held) and
// persisted in that same order, so the file can't grow without bound over
// a long session of continuous slider dragging. Each entry is at most a
// few KB as text (up to maxTapCount doubles, ~20 bytes each), so this caps
// the file at a few MB in the worst case.
constexpr int maxEntries = 1000;

inline juce::File getCacheFile()
{
    return juce::File::getSpecialLocation (juce::File::userApplicationDataDirectory)
             .getChildFile ("BBKDetachedPole")
             .getChildFile ("LiveSearchCache.xml");
}

// Reads every cached search result from disk. Returns an empty vector
// (not an error) if the file doesn't exist yet - the normal state before
// any Custom-mode search has ever completed.
inline std::vector<CacheEntry> loadAll()
{
    std::vector<CacheEntry> result;
    auto file = getCacheFile();
    if (! file.existsAsFile())
        return result;

    auto xml = juce::XmlDocument::parse (file);
    if (xml == nullptr || ! xml->hasTagName ("BBKDetachedPoleLiveSearchCache"))
        return result;

    for (auto* child : xml->getChildIterator())
    {
        if (! child->hasTagName ("Entry"))
            continue;

        CacheEntry e;
        e.spec.sampleRateHz = child->getDoubleAttribute ("sampleRateHz");
        e.spec.cutoffHz = child->getDoubleAttribute ("cutoffHz");
        e.spec.attenuationAtCutoffDb = child->getDoubleAttribute ("attenuationAtCutoffDb");
        e.spec.stopbandRejectionDb = child->getDoubleAttribute ("stopbandRejectionDb");
        e.spec.stopbandMode = static_cast<bbk::parametric::StopbandMode> (child->getIntAttribute ("stopbandMode"));
        e.spec.sidelobeDecayRatio = child->getDoubleAttribute ("sidelobeDecayRatio", 1.0);
        e.tapCount = child->getIntAttribute ("tapCount");
        e.achievedStopbandDb = child->getDoubleAttribute ("achievedStopbandDb");

        juce::StringArray tapStrings;
        tapStrings.addTokens (child->getStringAttribute ("taps"), ",", "");
        for (auto& s : tapStrings)
            if (s.isNotEmpty())
                e.taps.push_back (s.getDoubleValue());

        // Guards against a hand-edited or truncated/corrupted file - a
        // mismatched count means the taps string didn't round-trip, so
        // this entry can't be trusted as a real filter.
        if (e.tapCount > 0 && static_cast<int> (e.taps.size()) == e.tapCount)
            result.push_back (std::move (e));
    }
    return result;
}

// Rewrites the whole file from the given entry list - the caller (see
// PluginProcessor.cpp's run()) already applied the maxEntries cap
// in-memory before calling this.
inline bool saveAll (const std::vector<CacheEntry>& entries)
{
    juce::XmlElement root ("BBKDetachedPoleLiveSearchCache");
    for (auto& e : entries)
    {
        auto* child = root.createNewChildElement ("Entry");
        child->setAttribute ("sampleRateHz", e.spec.sampleRateHz);
        child->setAttribute ("cutoffHz", e.spec.cutoffHz);
        child->setAttribute ("attenuationAtCutoffDb", e.spec.attenuationAtCutoffDb);
        child->setAttribute ("stopbandRejectionDb", e.spec.stopbandRejectionDb);
        child->setAttribute ("stopbandMode", static_cast<int> (e.spec.stopbandMode));
        child->setAttribute ("sidelobeDecayRatio", e.spec.sidelobeDecayRatio);
        child->setAttribute ("tapCount", e.tapCount);
        child->setAttribute ("achievedStopbandDb", e.achievedStopbandDb);

        juce::String tapsStr;
        for (std::size_t i = 0; i < e.taps.size(); ++i)
        {
            if (i != 0)
                tapsStr << ",";
            tapsStr << juce::String (e.taps[i], 15);
        }
        child->setAttribute ("taps", tapsStr);
    }

    auto file = getCacheFile();
    file.getParentDirectory().createDirectory();
    return root.writeTo (file);
}

} // namespace bbk::detachedpole::searchcache
