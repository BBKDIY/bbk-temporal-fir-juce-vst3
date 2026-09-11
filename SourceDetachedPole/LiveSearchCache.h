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

// An entry now remembers up to bbk::parametric::topCandidateCount ranked
// candidates for its spec (best R_peak first - see ParametricFIR.h::
// RankedCandidate and designParametricFIR()'s own top-N tracking), not just
// the single winner: revisiting an exact spec later instantly recalls all
// of them, so the editor's top-N table has real alternatives to offer
// without re-searching, exactly as if the search had just finished again.
// candidates[0] is always what gets published on a plain cache hit (see
// PluginProcessor.cpp::requestBoundaryRedesign()) - there is no separate
// "active index" here the way UserPresetOverrides.h has one, since a cache
// hit is never a deliberate user choice the way a saved override is; the
// user can still re-pick any of the other candidates from the table after
// the fact, same as right after a live search.
struct CacheEntry
{
    bbk::parametric::FilterSpec spec;
    std::vector<bbk::parametric::RankedCandidate> candidates; // best-R_peak-first, up to topCandidateCount
};

// Kept in oldest-first insertion order in memory (see PluginProcessor.cpp,
// which evicts from the front once this many entries are held) and
// persisted in that same order, so the file can't grow without bound over
// a long session of continuous slider dragging. Each entry is at most a
// few tens of KB as text (up to topCandidateCount candidates, each up to
// maxTapCount doubles at ~20 bytes each), so this caps the file at a few
// tens of MB in the worst case.
constexpr int maxEntries = 1000;

inline juce::File getCacheFile()
{
    return juce::File::getSpecialLocation (juce::File::userApplicationDataDirectory)
             .getChildFile ("BBKDetachedPole")
             .getChildFile ("LiveSearchCache.xml");
}

// Reads every cached search result from disk. Returns an empty vector
// (not an error) if the file doesn't exist yet - the normal state before
// any Custom-mode search has ever completed. Also transparently upgrades
// the older one-candidate-per-entry file format (a single <Entry> with its
// own tapCount/achievedStopbandDb/taps attributes, no nested <Candidate>
// children) into a one-candidate candidates list, so entries cached before
// this top-N change keep working exactly as before rather than silently
// vanishing.
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

        bool anyCandidateChild = false;
        for (auto* candidateChild : child->getChildIterator())
        {
            if (! candidateChild->hasTagName ("Candidate"))
                continue;
            anyCandidateChild = true;

            bbk::parametric::RankedCandidate c;
            c.tapCount = candidateChild->getIntAttribute ("tapCount");
            c.achievedStopbandDb = candidateChild->getDoubleAttribute ("achievedStopbandDb");

            juce::StringArray tapStrings;
            tapStrings.addTokens (candidateChild->getStringAttribute ("taps"), ",", "");
            for (auto& s : tapStrings)
                if (s.isNotEmpty())
                    c.taps.push_back (s.getDoubleValue());

            if (c.tapCount > 0 && static_cast<int> (c.taps.size()) == c.tapCount)
                e.candidates.push_back (std::move (c));
        }

        // Older, pre-top-N file format: the single filter lived directly on
        // the <Entry> element itself, not in a nested <Candidate>. Read it
        // the same way as before and treat it as a one-entry list.
        if (! anyCandidateChild)
        {
            bbk::parametric::RankedCandidate c;
            c.tapCount = child->getIntAttribute ("tapCount");
            c.achievedStopbandDb = child->getDoubleAttribute ("achievedStopbandDb");

            juce::StringArray tapStrings;
            tapStrings.addTokens (child->getStringAttribute ("taps"), ",", "");
            for (auto& s : tapStrings)
                if (s.isNotEmpty())
                    c.taps.push_back (s.getDoubleValue());

            if (c.tapCount > 0 && static_cast<int> (c.taps.size()) == c.tapCount)
                e.candidates.push_back (std::move (c));
        }

        // Guards against a hand-edited or truncated/corrupted file - no
        // usable candidate surviving means this entry can't be trusted as
        // a real filter, same reasoning the old single-candidate check used.
        if (! e.candidates.empty())
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

        for (auto& c : e.candidates)
        {
            auto* candidateChild = child->createNewChildElement ("Candidate");
            candidateChild->setAttribute ("tapCount", c.tapCount);
            candidateChild->setAttribute ("achievedStopbandDb", c.achievedStopbandDb);

            juce::String tapsStr;
            for (std::size_t i = 0; i < c.taps.size(); ++i)
            {
                if (i != 0)
                    tapsStr << ",";
                tapsStr << juce::String (c.taps[i], 15);
            }
            candidateChild->setAttribute ("taps", tapsStr);
        }
    }

    auto file = getCacheFile();
    file.getParentDirectory().createDirectory();
    return root.writeTo (file);
}

} // namespace bbk::detachedpole::searchcache
