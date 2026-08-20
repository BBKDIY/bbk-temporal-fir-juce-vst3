"""
Run the full N=19..127 (odd) minimum-peak-sidelobe FIR sweep, compute the
article's time-domain metrics for every accepted design, and produce:
  - fir_sweep_full.csv      every accepted design
  - fir_sweep_pareto.csv    the Pareto-efficient subset (peak sidelobe % vs
                             settling duration), with N=19 force-included

Metric conventions (documented here since this sweep extends the article
rather than reproducing a result already in it):

  - "sidelobe region"       |offset from centre| >= K0 samples, where K0 is
                             the first sign-change offset of a 19-tap
                             Parks-McClellan reference design at the same
                             band edges (see solve_fir.derive_k0). Fixed as
                             an absolute sample count, not scaled with N.
  - peak sidelobe %          100 * max(|h[sidelobe region]|) / h[centre]
  - total ringing energy %   100 * sum(h[sidelobe region]^2) / sum(h^2)
  - settling duration (us)   ONE-SIDED: samples from centre outward to the
                             last tap whose |value| still exceeds 0.1% of
                             the centre tap, converted to microseconds
                             (divide by fs, *1e6). This is half the total
                             pre+post temporal footprint, reported one-sided
                             to match common DSP "settling time" usage.
  - worst passband error dB  20*log10(1 + max|A(f)-1|) over the passband
  - worst stopband level dB  20*log10(max|A(f)|) over the stopband
  - group delay              (N-1)/2 samples = (N-1)/(2*fs) seconds
"""
import numpy as np
import csv
import time
from solve_fir import solve_one, derive_k0, FS, PASSBAND_EDGE, STOPBAND_EDGE, NYQUIST
from scipy.signal import freqz

VERIFY_POINTS = 131072
SETTLE_THRESHOLD = 1e-3  # 0.1% of peak


def metrics_for(h, N, k0):
    c = N // 2
    peak = h[c]

    side_idx_offsets = np.arange(k0, c + 1)
    side_vals = h[c + side_idx_offsets]
    peak_sidelobe_pct = 100.0 * np.max(np.abs(side_vals)) / peak

    total_energy = np.sum(h ** 2)
    # side_vals covers offsets >= k0 on one side; the mirror side is identical by symmetry
    ringing_energy = 2.0 * np.sum(side_vals ** 2) if k0 > 0 else np.sum(h ** 2) - h[c] ** 2
    total_ringing_energy_pct = 100.0 * ringing_energy / total_energy

    # settling: outermost offset (from centre) where |h| still > 0.1% of peak
    settle_offset = 0
    for offset in range(c, -1, -1):
        if abs(h[c + offset]) > SETTLE_THRESHOLD * abs(peak):
            settle_offset = offset
            break
    settling_us = settle_offset / FS * 1e6

    w, H = freqz(h, worN=VERIFY_POINTS, fs=FS)
    pb_mask = w <= PASSBAND_EDGE
    sb_mask = w >= STOPBAND_EDGE
    worst_pb_dev = np.max(np.abs(np.abs(H[pb_mask]) - 1.0))
    worst_pb_db = 20 * np.log10(1.0 + worst_pb_dev)
    worst_sb_db = 20 * np.log10(np.max(np.abs(H[sb_mask])))

    group_delay_samples = c
    group_delay_us = c / FS * 1e6

    return dict(
        N=N,
        peak_sidelobe_pct=peak_sidelobe_pct,
        total_ringing_energy_pct=total_ringing_energy_pct,
        settling_us=settling_us,
        worst_passband_error_db=worst_pb_db,
        worst_stopband_level_db=worst_sb_db,
        group_delay_samples=group_delay_samples,
        group_delay_us=group_delay_us,
    )


def main():
    k0 = derive_k0()
    print("k0 =", k0)

    Ns = list(range(19, 128, 2))
    accepted = []
    rejected = []

    t0 = time.time()
    for N in Ns:
        h = solve_one(N, k0, verbose=False)
        if h is None:
            print(f"N={N}: REJECTED (infeasible or failed verification)")
            rejected.append(N)
            continue
        m = metrics_for(h, N, k0)
        m["taps"] = h.tolist()
        accepted.append(m)
        print(f"N={N:3d}  sidelobe={m['peak_sidelobe_pct']:6.2f}%  "
              f"ringing_energy={m['total_ringing_energy_pct']:6.2f}%  "
              f"settle={m['settling_us']:7.2f}us  "
              f"stopband={m['worst_stopband_level_db']:8.2f}dB  "
              f"pb_err={m['worst_passband_error_db']:.5f}dB")

    print(f"\ntotal time: {time.time()-t0:.1f}s, accepted {len(accepted)}/{len(Ns)}, rejected: {rejected}")

    # --- full sweep CSV ---
    with open("fir_sweep_full.csv", "w", newline="") as f:
        wtr = csv.writer(f)
        wtr.writerow(["N", "peak_sidelobe_pct", "total_ringing_energy_pct", "settling_us",
                      "worst_passband_error_db", "worst_stopband_level_db",
                      "group_delay_samples", "group_delay_us"])
        for m in accepted:
            wtr.writerow([m["N"], m["peak_sidelobe_pct"], m["total_ringing_energy_pct"],
                          m["settling_us"], m["worst_passband_error_db"], m["worst_stopband_level_db"],
                          m["group_delay_samples"], m["group_delay_us"]])

    # --- Pareto frontier: minimize peak_sidelobe_pct AND minimize settling_us ---
    def dominates(a, b):
        return (a["peak_sidelobe_pct"] <= b["peak_sidelobe_pct"] and
                a["settling_us"] <= b["settling_us"] and
                (a["peak_sidelobe_pct"] < b["peak_sidelobe_pct"] or
                 a["settling_us"] < b["settling_us"]))

    pareto = []
    for cand in accepted:
        if not any(dominates(other, cand) for other in accepted if other is not cand):
            pareto.append(cand)

    pareto_Ns = {p["N"] for p in pareto}
    if 19 not in pareto_Ns:
        baseline = next(m for m in accepted if m["N"] == 19)
        pareto.append(baseline)
        print("N=19 was not Pareto-efficient - force-included as required baseline.")

    pareto.sort(key=lambda m: m["settling_us"])

    with open("fir_sweep_pareto.csv", "w", newline="") as f:
        wtr = csv.writer(f)
        wtr.writerow(["N", "peak_sidelobe_pct", "total_ringing_energy_pct", "settling_us",
                      "worst_passband_error_db", "worst_stopband_level_db",
                      "group_delay_samples", "group_delay_us"])
        for m in pareto:
            wtr.writerow([m["N"], m["peak_sidelobe_pct"], m["total_ringing_energy_pct"],
                          m["settling_us"], m["worst_passband_error_db"], m["worst_stopband_level_db"],
                          m["group_delay_samples"], m["group_delay_us"]])

    print(f"\nPareto-efficient set ({len(pareto)} points):")
    for m in pareto:
        print(f"  N={m['N']:3d}  sidelobe={m['peak_sidelobe_pct']:6.2f}%  settle={m['settling_us']:7.2f}us")

    # save full taps for the pareto set as JSON for the C++ codegen step
    import json
    with open("fir_sweep_pareto_taps.json", "w") as f:
        json.dump(pareto, f, indent=2)
    with open("fir_sweep_full_taps.json", "w") as f:
        json.dump(accepted, f, indent=2)


if __name__ == "__main__":
    main()
