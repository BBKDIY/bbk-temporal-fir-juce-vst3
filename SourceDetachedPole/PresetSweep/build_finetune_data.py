#!/usr/bin/env python3
"""Regenerates parent_map.json and bank_sweep_finetune.csv - the fine-tune
pass over bank_sweep.csv's coarse (0.05dB-spaced) sweep.

For each (sampleRateHz, nominal attenuation) pair's current coarse winner
(lowest R_peak among sane rows at that exact nominal attenuation), this
sweeps +/-0.01dB and +/-0.02dB around it - same tap count as the winner,
since that rarely needs to change over such a small attenuation range -
looking for a genuinely better nearby operating point, the same way a user
fine-tuning by ear would nudge the Attenuation slider a little either way.

Requires SourceDetachedPole/PresetSweep/sweep_bank.cpp already built (see
that file's own usage comment) at ../../.. relative build path or on PATH
as `sweep_bank`; pass its path as the sole argument if it lives elsewhere.

Usage: python3 build_finetune_data.py [path/to/sweep_bank]
"""
import csv
import json
import os
import subprocess
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_CSV = os.path.join(_SCRIPT_DIR, "bank_sweep.csv")
OUT_CSV = os.path.join(_SCRIPT_DIR, "bank_sweep_finetune.csv")
OUT_MAP = os.path.join(_SCRIPT_DIR, "parent_map.json")

SANE_STOPBAND_LO = -110.0
SANE_STOPBAND_HI = -85.0
OFFSETS_DB = [0.01, -0.01, 0.02, -0.02]  # matches the shipped bank's own fine-tune range
SOLVER_CAP_SECONDS = 30
TOOL_BUDGET_SECONDS = 40

def load_rows(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            r["sampleRateHz"] = float(r["sampleRateHz"])
            r["attenuationDb"] = float(r["attenuationDb"])
            r["M"] = int(r["M"])
            r["feasible"] = r["feasible"] == "1"
            r["worstStopbandDb"] = float(r["worstStopbandDb"])
            r["rPeakPercent"] = float(r["rPeakPercent"])
            rows.append(r)
    return rows

def is_sane(r):
    return r["feasible"] and SANE_STOPBAND_LO <= r["worstStopbandDb"] <= SANE_STOPBAND_HI

def main():
    sweep_bank_bin = sys.argv[1] if len(sys.argv) > 1 else "sweep_bank"

    rows = load_rows(SRC_CSV)
    pairs = {}
    for r in rows:
        pairs.setdefault((r["sampleRateHz"], r["attenuationDb"]), []).append(r)

    parent_map = {}
    fine_points = []
    for (rate, atten), pair_rows in pairs.items():
        sane = [r for r in pair_rows if is_sane(r)]
        if not sane:
            continue
        winner_m = min(sane, key=lambda r: r["rPeakPercent"])["M"]
        parent_map[(round(rate, 6), round(atten, 6))] = round(atten, 6)
        for off in OFFSETS_DB:
            fine_atten = round(atten + off, 6)
            if fine_atten <= 0.0:
                continue
            parent_map[(round(rate, 6), fine_atten)] = round(atten, 6)
            fine_points.append((rate, fine_atten, winner_m))

    print(f"{len(fine_points)} fine-tune points to sweep across {len(pairs)} coarse pairs")

    if os.path.exists(OUT_CSV):
        os.remove(OUT_CSV)
    for rate, atten, m in fine_points:
        subprocess.run([sweep_bank_bin, f"{rate:g}", f"{atten:g}", str(m), str(m),
                         str(TOOL_BUDGET_SECONDS), "0", OUT_CSV, str(SOLVER_CAP_SECONDS)],
                        check=True)

    with open(OUT_MAP, "w") as f:
        json.dump([[k[0], k[1], v] for k, v in parent_map.items()], f)
    print(f"Wrote {OUT_MAP} ({len(parent_map)} entries) and {OUT_CSV}")

if __name__ == "__main__":
    main()
