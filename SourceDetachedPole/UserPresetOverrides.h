#pragma once

// Persistence for user-saved filter overrides ("Save as Default" in the
// editor - see PluginEditor.cpp/PluginProcessor.cpp::saveTopCandidateAsOverride).
// This header only does I/O and serialisation; matching an override against
// the current spec (specsEqual) is the processor's own job, using the same
// exact-comparison helper it already uses for the redesign-memory dedup.
//
// One shared, per-install file (not per-project/per-DAW-session): the whole
// point of a saved override is "whenever I'm back at this exact operating
// point, use my filter" - that should hold regardless of which project or
// host you're in, the same as the compiled-in factory bank already does.

#include "ParametricFIR.h"

#include <juce_core/juce_core.h>

#include <vector>

namespace bbk::detachedpole::useroverrides
{

// An override now remembers up to bbk::parametric::topCandidateCount ranked
// candidates for its spec (see ParametricFIR.h::RankedCandidate and
// designParametricFIR()'s own top-N tracking), not just one - "Save as
// Default" persists the whole top-N list a search found, tagged with which
// ONE of them the user actually picked (activeIndex), so revisiting this
// exact spec later - in Default mode or via an exact-spec Custom lookup -
// both instantly loads the user's chosen filter AND lets the editor's top-N
// table offer the same alternatives again without re-searching. activeIndex
// is always a valid index into candidates (enforced on load - see loadAll()).
struct OverrideEntry
{
    bbk::parametric::FilterSpec spec;
    std::vector<bbk::parametric::RankedCandidate> candidates; // best-R_peak-first, up to topCandidateCount
    int activeIndex = 0;
};

inline juce::File getOverrideFile()
{
    return juce::File::getSpecialLocation (juce::File::userApplicationDataDirectory)
        .getChildFile ("BBKDetachedPole")
        .getChildFile ("UserPresetOverrides.xml");
}

// Reads every saved override from disk. Returns an empty vector (not an
// error) if the file doesn't exist yet - the normal state before the user
// has ever clicked "Save as Default". Also transparently upgrades the older
// one-candidate-per-override file format (a single <Override> with its own
// tapCount/achievedStopbandDb/taps attributes, no nested <Candidate>
// children) into a one-candidate candidates list, so overrides saved before
// this top-N change keep working exactly as before rather than silently
// vanishing.
inline std::vector<OverrideEntry> loadAll()
{
    std::vector<OverrideEntry> result;
    auto file = getOverrideFile();
    if (! file.existsAsFile())
        return result;

    auto xml = juce::XmlDocument::parse (file);
    if (xml == nullptr || ! xml->hasTagName ("BBKDetachedPoleUserOverrides"))
        return result;

    for (auto* child : xml->getChildIterator())
    {
        if (! child->hasTagName ("Override"))
            continue;

        OverrideEntry e;
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
        // the <Override> element itself, not in a nested <Candidate>. Read
        // it the same way as before and treat it as a one-entry list.
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

        if (e.candidates.empty())
            continue; // nothing usable survived (corrupted/truncated entry) - drop it, same as before

        e.activeIndex = child->getIntAttribute ("activeIndex", 0);
        if (e.activeIndex < 0 || e.activeIndex >= static_cast<int> (e.candidates.size()))
            e.activeIndex = 0; // guards a corrupted/out-of-range index the same way the size check above guards taps

        result.push_back (std::move (e));
    }
    return result;
}

// Rewrites the whole file from the given entry list (the caller - see
// saveTopCandidateAsOverride() - already merged/replaced in-memory before
// calling this, since "update this one entry" isn't naturally expressible
// against a flat file without reading it back first anyway).
inline bool saveAll (const std::vector<OverrideEntry>& entries)
{
    juce::XmlElement root ("BBKDetachedPoleUserOverrides");
    for (auto& e : entries)
    {
        auto* child = root.createNewChildElement ("Override");
        child->setAttribute ("sampleRateHz", e.spec.sampleRateHz);
        child->setAttribute ("cutoffHz", e.spec.cutoffHz);
        child->setAttribute ("attenuationAtCutoffDb", e.spec.attenuationAtCutoffDb);
        child->setAttribute ("stopbandRejectionDb", e.spec.stopbandRejectionDb);
        child->setAttribute ("stopbandMode", static_cast<int> (e.spec.stopbandMode));
        child->setAttribute ("sidelobeDecayRatio", e.spec.sidelobeDecayRatio);
        child->setAttribute ("activeIndex", e.activeIndex);

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

    auto file = getOverrideFile();
    file.getParentDirectory().createDirectory();
    return root.writeTo (file);
}

} // namespace bbk::detachedpole::useroverrides
