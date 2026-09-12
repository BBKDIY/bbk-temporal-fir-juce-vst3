#pragma once

// Persistence for user-saved filter overrides - both the plain, per-exact-
// spec kind (a top-N table row's own "Save" button - see PluginEditor.cpp/
// PluginProcessor.cpp::saveTopCandidateAsOverride) and the numbered preset
// slots (savePresetSlot()/loadPresetSlot()) that replaced the old single
// "Default" toggle. This header only does I/O and serialisation; matching
// an override against the current spec (specsEqual), and deciding which
// entry (if any) currently occupies a given preset slot, is the processor's
// own job.
//
// One shared, per-install file (not per-project/per-DAW-session): the whole
// point of a saved override is "whenever I'm back at this exact operating
// point, use my filter" - that should hold regardless of which project or
// host you're in, the same as the compiled-in factory bank already does.
// loadAll()/saveAll() below both take an optional file argument for exactly
// this reason: BBKDetachedPoleAudioProcessor::exportPresets()/importPresets()
// reuse this same file format, and this same parsing/writing code, to let a
// preset bank travel between installs as one ordinary file a user can email,
// message, or drop in a shared folder.

#include "ParametricFIR.h"

#include <juce_core/juce_core.h>

#include <array>
#include <vector>

namespace bbk::detachedpole::useroverrides
{

// How many general-purpose preset slots the editor offers (see
// PluginEditor.h's presetLoadButtons/presetSaveButtons and
// BBKDetachedPoleAudioProcessor::loadPresetSlot()/savePresetSlot()). This
// replaced the old single "Default" toggle entirely: rather than one
// instant operating point per sample rate (picked automatically as
// "whichever override was saved most recently"), the user gets 5
// independently addressable bookmarks - "save whatever I'm listening to
// right now as Preset 3", recalled later without needing to remember which
// combination of cutoff/attenuation/stopband/decay produced it in the
// first place.
constexpr int numPresetSlots = 5;

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

    // -1 for a plain override, saved via a top-N table row's own per-
    // candidate "Save" button - recalled only when its own EXACT spec
    // recurs (see requestBoundaryRedesign()'s overrideEntry lookup), same
    // as always. 0..numPresetSlots-1 means this override is ALSO the
    // current occupant of that numbered preset slot - see
    // BBKDetachedPoleAudioProcessor::loadPresetSlot()/savePresetSlot(),
    // which is what replaced the old "Default" toggle: loading a preset
    // slot forces cutoff/attenuation/stopband/decay to THIS entry's own
    // spec (recalling it exactly, whatever it was searched with) rather
    // than requiring the spec to already match what's currently dialed in.
    // At most one entry may claim a given slot at a time - saving a new
    // preset into an occupied slot clears this field on whatever entry
    // held it before (that entry isn't deleted, it just stops being a
    // preset - it's still a plain override for its own exact spec).
    //
    // Multiple entries CAN share the exact same spec now, each tagged with
    // a different slot (or one left untagged, -1): two different top-N
    // candidates from the very same Auto-mode search share an identical
    // spec (only their chosen candidate/activeIndex differs - see
    // RankedCandidate's own comment in ParametricFIR.h), and saving both
    // into two different presets is an expected, supported use of this
    // field, not a conflict. See BBKDetachedPoleAudioProcessor::
    // savePresetSlot()/saveTopCandidateAsOverride()'s own comments for how
    // their save-time dedup logic accounts for this (only ever replacing a
    // plain, non-preset entry or this exact slot's own prior occupant -
    // never a different slot's entry, even one sharing this spec), and
    // loadPresetSlot()'s own comment for why recalling a specific slot
    // republishes that slot's own captured entry directly rather than
    // trusting a spec-only lookup that would otherwise be ambiguous
    // whenever duplicates like this exist.
    int presetSlot = -1;
};

inline juce::File getOverrideFile()
{
    return juce::File::getSpecialLocation (juce::File::userApplicationDataDirectory)
        .getChildFile ("BBKDetachedPole")
        .getChildFile ("UserPresetOverrides.xml");
}

// Reads every saved override from disk. Returns an empty vector (not an
// error) if the file doesn't exist yet - the normal state before the user
// has ever saved anything. Also transparently upgrades the older
// one-candidate-per-override file format (a single <Override> with its own
// tapCount/achievedStopbandDb/taps attributes, no nested <Candidate>
// children) into a one-candidate candidates list, so overrides saved before
// this top-N change keep working exactly as before rather than silently
// vanishing.
//
// file defaults to the shared per-install override file (getOverrideFile())
// but can be pointed at any other file - this is what
// BBKDetachedPoleAudioProcessor::importPresets() uses to read a bank a
// friend exported (see saveAll()'s own comment on the matching export
// path), without needing a second, parallel implementation of this same
// parsing logic.
inline std::vector<OverrideEntry> loadAll (const juce::File& file = getOverrideFile())
{
    std::vector<OverrideEntry> result;
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

        // Missing attribute (every file written before preset slots existed,
        // including ones tagged isDefaultChoice="true" under the old
        // now-removed Default toggle) defaults to -1: a plain, non-preset
        // override, still fully recallable for its own exact spec exactly
        // as before - it just doesn't automatically claim a numbered preset
        // slot on upgrade. See OverrideEntry::presetSlot's own comment.
        e.presetSlot = child->getIntAttribute ("presetSlot", -1);
        if (e.presetSlot < -1 || e.presetSlot >= numPresetSlots)
            e.presetSlot = -1; // guard a corrupted/out-of-range slot the same way activeIndex is guarded above

        result.push_back (std::move (e));
    }

    // Guard against two entries claiming the same preset slot - shouldn't
    // happen from this file's own writer (saveTopCandidateAsOverride()/
    // savePresetSlot() always clear the previous occupant first), but a
    // hand-edited file, a corrupted write, or an imported file merged in
    // some unexpected way could still produce one. First entry in file
    // order wins; every later duplicate claimant is demoted to a plain
    // (non-preset) override rather than silently leaving two "Preset 3"
    // buttons disagreeing about what they'd load.
    {
        std::array<bool, static_cast<std::size_t> (numPresetSlots)> slotClaimed {};
        for (auto& e : result)
        {
            if (e.presetSlot < 0)
                continue;
            auto idx = static_cast<std::size_t> (e.presetSlot);
            if (slotClaimed[idx])
                e.presetSlot = -1;
            else
                slotClaimed[idx] = true;
        }
    }

    return result;
}

// Rewrites the whole file from the given entry list (the caller - see
// saveTopCandidateAsOverride() - already merged/replaced in-memory before
// calling this, since "update this one entry" isn't naturally expressible
// against a flat file without reading it back first anyway).
//
// file defaults to the shared per-install override file, same as loadAll()
// above - see its own comment on why a caller would ever pass a different
// one (exportPresets()).
inline bool saveAll (const std::vector<OverrideEntry>& entries, const juce::File& file = getOverrideFile())
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
        child->setAttribute ("presetSlot", e.presetSlot);

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

    file.getParentDirectory().createDirectory();
    return root.writeTo (file);
}

} // namespace bbk::detachedpole::useroverrides
