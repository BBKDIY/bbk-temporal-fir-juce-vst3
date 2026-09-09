#pragma once

// Persistence for user-saved filter overrides ("Save as Default" in the
// editor - see PluginEditor.cpp/PluginProcessor.cpp::saveCurrentAsOverride).
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

struct OverrideEntry
{
    bbk::parametric::FilterSpec spec;
    int tapCount = 0;
    double achievedStopbandDb = 0.0;
    std::vector<double> taps; // natural length == tapCount, unity DC gain
};

inline juce::File getOverrideFile()
{
    return juce::File::getSpecialLocation (juce::File::userApplicationDataDirectory)
             .getChildFile ("BBKDetachedPole")
             .getChildFile ("UserPresetOverrides.xml");
}

// Reads every saved override from disk. Returns an empty vector (not an
// error) if the file doesn't exist yet - the normal state before the user
// has ever clicked "Save as Default".
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

// Rewrites the whole file from the given entry list (the caller - see
// saveCurrentAsOverride() - already merged/replaced in-memory before
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

    auto file = getOverrideFile();
    file.getParentDirectory().createDirectory();
    return root.writeTo (file);
}

} // namespace bbk::detachedpole::useroverrides
