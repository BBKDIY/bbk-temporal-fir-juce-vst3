#!/usr/bin/env python3
"""Select the prebuilt filter bank entries from the raw sweep CSV and emit
SourceDetachedPole/PresetFilterBank.h.

Each entry is keyed by a fixed NOMINAL attenuation step (0.05, 0.10, ...,
0.50 dB - what PresetBankLookup.h's snapToNearestAttenuationStep() snaps
the user's slider to). The ACTUAL attenuation the winning filter was
designed at can differ slightly from the nominal step: a fine-tune pass
sweeps +/-0.01/+/-0.02 dB around each nominal step's own coarse winner
(same tap count, since that rarely needs to change over such a small
attenuation range) looking for a genuinely better nearby operating point,
the same way a user fine-tuning by ear would nudge the Attenuation slider
a little either way. parent_map.json (built alongside the sweep, not
re-derivable from the attenuation value alone - a fine-tune offset can
land closer to a NEIGHBOURING nominal step than its own parent) says which
nominal step every row in the CSV belongs to.

Selection rule per nominal (sampleRateHz, nominal attenuationDb):
  1. Restrict to rows that are feasible AND sane (worstStopbandDb within
     [-110, -85] dB - the sweep targeted -95dB, so a real converged hit
     lands within a few tenths of that; anything wildly outside this
     window, including the exact-0.0 infeasible sentinel and the
     numerically-degenerate positive-dB readings seen at some larger M's,
     is discarded rather than risking it being picked as "best").
  2. M_best = the sane row with the lowest R_peak; R_min = its R_peak.
  3. "Near-best" set = sane rows with R_peak <= R_min * 1.05 (5% relative
     - deliberately tight: this is a tie zone for step 4 below, not the
     wide pool the settling-time override used to compare across, so it
     should only catch rows that are genuinely close to R_min, not merely
     "not terrible").
  4. fastest = the near-best row(s) with the shortest T_0.1% (settlingMs).
     Rows within a tiny epsilon of the fastest settlingMs (same tap count
     virtually always means bit-identical settlingMs, since it's a
     quantised sample count) form the final tie group.
  5. Within that tie group, break the tie by whichever of E_ZC (lower
     better) or center-tap gain (higher better) shows the larger RELATIVE
     improvement over the worst-in-group value on that axis; the row
     winning by the larger margin is chosen. (E_ZC and center-tap aren't
     sweep-CSV columns - they're recomputed here from the stored taps
     using the same formulas as bbk::parametric::computeTemporalMetrics.)
  6. The old "switch away from M_best only if settling improves by >40%"
     override no longer applies globally - step 3's tight near-best band
     already only lets genuinely-comparable-R_peak rows compete, so
     picking the fastest-settling one among them (then breaking further
     ties by E_ZC/center-tap) IS the whole rule now.
"""
import csv
import json
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_CSV = os.path.join(_SCRIPT_DIR, "bank_sweep.csv")
FINE_CSV = os.path.join(_SCRIPT_DIR, "bank_sweep_finetune.csv")
PARENT_MAP = os.path.join(_SCRIPT_DIR, "parent_map.json")
OUT_HEADER = os.path.join(_SCRIPT_DIR, "..", "PresetFilterBank.h")

SANE_STOPBAND_LO = -110.0
SANE_STOPBAND_HI = -85.0
NEAR_BEST_REL = 1.05
SETTLING_TIE_EPS_MS = 1.0e-6

def load_rows(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            r["sampleRateHz"] = float(r["sampleRateHz"])
            r["attenuationDb"] = float(r["attenuationDb"])
            r["M"] = int(r["M"])
            r["tapCount"] = int(r["tapCount"])
            r["feasible"] = r["feasible"] == "1"
            r["worstStopbandDb"] = float(r["worstStopbandDb"])
            r["rPeakPercent"] = float(r["rPeakPercent"])
            r["settlingSampleSpan"] = int(r["settlingSampleSpan"])
            r["settlingMs"] = float(r["settlingMs"])
            r["taps"] = [float(x) for x in r["taps"].split(";")] if r["taps"] else []
            rows.append(r)
    return rows

def is_sane(r):
    return r["feasible"] and SANE_STOPBAND_LO <= r["worstStopbandDb"] <= SANE_STOPBAND_HI

def temporal_extras(taps):
    """E_ZC (%) and center-tap gain (%), mirroring computeTemporalMetrics."""
    n = len(taps)
    if n == 0:
        return 0.0, 0.0
    peak_idx = max(range(n), key=lambda i: abs(taps[i]))
    peak_abs = abs(taps[peak_idx])
    if peak_abs <= 0.0:
        return 0.0, 0.0
    center_tap_pct = taps[peak_idx] * 100.0
    peak_sign = 1.0 if taps[peak_idx] >= 0.0 else -1.0

    lobe_start = 0
    for i in range(peak_idx - 1, -1, -1):
        if taps[i] * peak_sign < 0.0:
            lobe_start = i + 1
            break
        if i == 0:
            lobe_start = 0
    lobe_end = n - 1
    for i in range(peak_idx + 1, n):
        if taps[i] * peak_sign < 0.0:
            lobe_end = i - 1
            break
        if i == n - 1:
            lobe_end = n - 1

    total_energy = 0.0
    outside_energy = 0.0
    for i in range(n):
        v = taps[i]
        total_energy += v * v
        if i < lobe_start or i > lobe_end:
            outside_energy += v * v
    e_zc_pct = (outside_energy / total_energy) * 100.0 if total_energy > 0.0 else 0.0
    return e_zc_pct, center_tap_pct

def select_pair(rows):
    sane = [r for r in rows if is_sane(r)]
    if not sane:
        return None, "no sane feasible row"

    m_best = min(sane, key=lambda r: r["rPeakPercent"])
    r_min = m_best["rPeakPercent"]
    near_best = [r for r in sane if r["rPeakPercent"] <= r_min * NEAR_BEST_REL]

    fastest_ms = min(r["settlingMs"] for r in near_best)
    tie_group = [r for r in near_best if r["settlingMs"] <= fastest_ms + SETTLING_TIE_EPS_MS]

    if len(tie_group) == 1:
        chosen = tie_group[0]
        note = f"near-best + fastest-settling, no further tie (R_peak {chosen['rPeakPercent']:.3f}%)"
        return chosen, note

    for r in tie_group:
        r["_eZc"], r["_centerTap"] = temporal_extras(r["taps"])

    worst_ezc = max(r["_eZc"] for r in tie_group)
    worst_ct = min(r["_centerTap"] for r in tie_group)

    best = None
    best_margin = -1.0
    best_axis = None
    for r in tie_group:
        ezc_gain = (worst_ezc - r["_eZc"]) / worst_ezc if worst_ezc > 0.0 else 0.0
        ct_gain = (r["_centerTap"] - worst_ct) / worst_ct if worst_ct > 0.0 else 0.0
        margin, axis = (ezc_gain, "E_ZC") if ezc_gain >= ct_gain else (ct_gain, "center-tap")
        if margin > best_margin:
            best_margin = margin
            best = r
            best_axis = axis

    chosen = best
    note = (f"tie-break within {len(tie_group)}-way settling tie by {best_axis} "
            f"(R_peak {chosen['rPeakPercent']:.3f}%, E_ZC {chosen['_eZc']:.3f}%, "
            f"centerTap {chosen['_centerTap']:.2f}%)")
    return chosen, note

def fmt_double(x):
    return repr(x)

def main():
    rows = load_rows(SRC_CSV)
    try:
        rows += load_rows(FINE_CSV)
    except FileNotFoundError:
        print(f"WARNING: {FINE_CSV} not found - proceeding with coarse sweep only", file=sys.stderr)

    with open(PARENT_MAP) as f:
        parent_list = json.load(f)
    parent_map = {(round(rate, 6), round(atten, 6)): round(parent, 6) for rate, atten, parent in parent_list}

    pairs = {}
    unmapped = 0
    for r in rows:
        key = (round(r["sampleRateHz"], 6), round(r["attenuationDb"], 6))
        nominal = parent_map.get(key)
        if nominal is None:
            unmapped += 1
            continue
        pairs.setdefault((key[0], nominal), []).append(r)
    if unmapped:
        print(f"NOTE: {unmapped} rows had no parent-map entry (ignored)", file=sys.stderr)

    entries = []
    for key in sorted(pairs.keys()):
        chosen, note = select_pair(pairs[key])
        rate, nominal_atten = key
        if chosen is None:
            print(f"WARNING: Fs={rate} atten={nominal_atten}: {note} - SKIPPING, no entry generated", file=sys.stderr)
            continue
        ezc, ct = temporal_extras(chosen["taps"])
        actual_atten = chosen["attenuationDb"]
        print(f"Fs={rate:.6g} nominal_atten={nominal_atten:.6g} -> actual_atten={actual_atten:.6g} "
              f"M={chosen['M']} taps={chosen['tapCount']} R_peak={chosen['rPeakPercent']:.4f}% "
              f"T01={chosen['settlingMs']:.4f}ms E_ZC={ezc:.4f}% centerTap={ct:.3f}%  ({note})")
        entries.append((rate, nominal_atten, actual_atten, chosen))

    print(f"\n{len(entries)} entries selected out of {len(pairs)} pairs")

    with open(OUT_HEADER, "w") as f:
        f.write("#pragma once\n\n")
        f.write("// Auto-generated by SourceDetachedPole/PresetSweep/generate_bank_header.py\n")
        f.write("// from bank_sweep.csv (plus a +/-0.01/+/-0.02dB fine-tune pass). Do not\n")
        f.write("// hand-edit - regenerate from the sweep if the design method, cutoff,\n")
        f.write("// stopband target, or selection rule changes.\n")
        f.write("//\n")
        f.write("// Every entry is a precomputed bbk::parametric::designParametricFIR() result\n")
        f.write("// at a fixed cutoff (18500 Hz), stopband target (95 dB), and sidelobe decay\n")
        f.write("// (1.0, flat/undecayed) - only sample rate and attenuation-at-cutoff vary.\n")
        f.write("// This is the plugin's \"Default\" mode: an instant lookup instead of a live\n")
        f.write("// multi-second LP search, for the specific operating point found (by ear) to\n")
        f.write("// work well across the board. Taps are stored at their natural (unpadded)\n")
        f.write("// length - callers pad to the plugin's own fixed maxTapCount the same way a\n")
        f.write("// live Custom-mode design's taps are padded (see padTapsToFixedLength in\n")
        f.write("// DetachedPoleFilter.h), so Default and Custom modes share one latency\n")
        f.write("// contract.\n")
        f.write("//\n")
        f.write("// nominalAttenuationDb is the fixed 0.05dB-spaced step the Attenuation\n")
        f.write("// slider snaps to (see PresetBankLookup.h::snapToNearestAttenuationStep) -\n")
        f.write("// lookups match on THIS field. attenuationAtCutoffDb is the exact value the\n")
        f.write("// filter was actually designed at, which a fine-tune pass may have nudged\n")
        f.write("// slightly off the nominal step (e.g. 0.504 instead of 0.50) if that gave a\n")
        f.write("// genuinely better result - informational only, not used for matching.\n")
        f.write("\n#include <vector>\n\n")
        f.write("namespace bbk::detachedpole::presetbank\n{\n\n")
        f.write("constexpr double presetCutoffHz = 18500.0;\n")
        f.write("constexpr double presetStopbandRejectionDb = 95.0;\n")
        f.write("constexpr double presetSidelobeDecayRatio = 1.0;\n\n")
        f.write("struct BankEntry\n{\n")
        f.write("    double sampleRateHz;\n")
        f.write("    double nominalAttenuationDb;  // matched against by findEntry()\n")
        f.write("    double attenuationAtCutoffDb; // exact design point (see file comment)\n")
        f.write("    int tapCount;\n")
        f.write("    double rPeakPercent;\n")
        f.write("    double settlingMs;\n")
        f.write("    double achievedStopbandDb;\n")
        f.write("    std::vector<double> taps; // natural length == tapCount, unity DC gain\n")
        f.write("};\n\n")
        f.write(f"constexpr int numBankEntries = {len(entries)};\n\n")
        f.write("inline const std::vector<BankEntry>& bank()\n{\n")
        f.write("    static const std::vector<BankEntry> table = {\n")
        for rate, nominal_atten, actual_atten, chosen in entries:
            taps_str = ", ".join(fmt_double(t) for t in chosen["taps"])
            f.write(f"        {{ {fmt_double(rate)}, {fmt_double(nominal_atten)}, {fmt_double(actual_atten)}, "
                    f"{chosen['tapCount']}, {fmt_double(chosen['rPeakPercent'])}, "
                    f"{fmt_double(chosen['settlingMs'])}, {fmt_double(chosen['worstStopbandDb'])},\n")
            f.write(f"          {{ {taps_str} }} }},\n")
        f.write("    };\n")
        f.write("    return table;\n")
        f.write("}\n\n")
        f.write("} // namespace bbk::detachedpole::presetbank\n")

    print(f"\nWrote {OUT_HEADER}")

if __name__ == "__main__":
    main()
