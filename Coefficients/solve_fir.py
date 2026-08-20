"""
BBK Temporal FIR - offline coefficient generation.

For every odd tap count N in a given list, solves the frequency-constrained
minimum-peak-sidelobe Type-I linear-phase FIR design problem via linear
programming:

    minimize    t
    subject to  -t <= h[m] <= t              for every free coefficient m
                                               whose offset from centre is
                                               >= K0 samples (sidelobe region)
                |A(f) - 1| <= PASSBAND_TOL     for f in [0, PASSBAND_EDGE]
                A(0) = 1  (exact)
                |A(f)|    <= STOPBAND_LEVEL    for f in [STOPBAND_EDGE, NYQUIST]

A(f) is the zero-phase amplitude response of the symmetric Type-I FIR,
linear in the free (half) coefficients, so the whole problem is a linear
program solvable with scipy.optimize.linprog (HiGHS).

K0 (the main-lobe exclusion half-width, in samples) is fixed once, from the
first sign change of a 19-tap Parks-McClellan/Remez reference design at the
same band edges - not re-derived per N. See derive_k0().
"""
import numpy as np
from scipy.optimize import linprog
from scipy.signal import remez, freqz
import json

FS = 192000.0
PASSBAND_EDGE = 20000.0
STOPBAND_EDGE = 76000.0
NYQUIST = 96000.0
PASSBAND_TOL = 1e-4
STOPBAND_DB = -100.3
STOPBAND_LEVEL = 10 ** (STOPBAND_DB / 20.0)
VERIFY_POINTS = 131072


def derive_k0():
    """First sign-change offset (samples from centre) of the 19-tap
    reference equiripple design at the project's band edges."""
    h = remez(19, [0, PASSBAND_EDGE, STOPBAND_EDGE, NYQUIST], [1, 0], weight=[1, 1], fs=FS)
    c = len(h) // 2
    prev_sign = np.sign(h[c])
    for offset in range(1, c + 1):
        s = np.sign(h[c + offset])
        if s != 0 and s != prev_sign:
            return offset
        if s != 0:
            prev_sign = s
    raise RuntimeError("no sign change found in reference design")


def amplitude_coeffs(freqs, c):
    """Return matrix M with A(f) = M @ x, x = [h[0..c]] (h[c] = centre).
    M[:, m] = 2*cos(2*pi*f*(c-m)/fs) for m < c, and 1 for m == c."""
    freqs = np.asarray(freqs, dtype=np.float64)
    M = np.zeros((len(freqs), c + 1), dtype=np.float64)
    for m in range(c):
        k = c - m
        M[:, m] = 2.0 * np.cos(2.0 * np.pi * freqs * k / FS)
    M[:, c] = 1.0
    return M


def solve_one(N, k0, max_refine=8, verbose=False):
    """Solve the minimum-peak-sidelobe LP for tap count N. Returns the full
    symmetric coefficient array (length N) or None if infeasible/rejected."""
    c = N // 2
    if c - k0 < 0:
        return None  # filter too short to have any sidelobe-region freedom

    # sidelobe free-coefficient indices: m = 0 .. (c - k0)
    sidelobe_idx = list(range(0, c - k0 + 1))

    pb_grid = np.linspace(1.0, PASSBAND_EDGE, 400)  # f=0 handled as equality
    sb_grid = np.linspace(STOPBAND_EDGE, NYQUIST, 900)

    nvars = c + 2  # x[0..c] plus t
    t_col = nvars - 1

    for refine_iter in range(max_refine):
        Mpb = amplitude_coeffs(pb_grid, c)
        Msb = amplitude_coeffs(sb_grid, c)

        A_ub = []
        b_ub = []

        # passband: A(f) <= 1+tol ; -A(f) <= -(1-tol)
        for row in Mpb:
            r = np.zeros(nvars); r[:c + 1] = row
            A_ub.append(r); b_ub.append(1.0 + PASSBAND_TOL)
            A_ub.append(-r); b_ub.append(-(1.0 - PASSBAND_TOL))

        # stopband: A(f) <= S ; -A(f) <= S
        for row in Msb:
            r = np.zeros(nvars); r[:c + 1] = row
            A_ub.append(r); b_ub.append(STOPBAND_LEVEL)
            A_ub.append(-r); b_ub.append(STOPBAND_LEVEL)

        # sidelobe bound: x[m] - t <= 0 ; -x[m] - t <= 0
        for m in sidelobe_idx:
            r = np.zeros(nvars); r[m] = 1.0; r[t_col] = -1.0
            A_ub.append(r); b_ub.append(0.0)
            r = np.zeros(nvars); r[m] = -1.0; r[t_col] = -1.0
            A_ub.append(r); b_ub.append(0.0)

        # equality: A(0) = 1
        row0 = amplitude_coeffs(np.array([0.0]), c)[0]
        A_eq = [np.concatenate([row0, [0.0]])]
        b_eq = [1.0]

        obj = np.zeros(nvars); obj[t_col] = 1.0
        bounds = [(-2.0, 2.0)] * (c + 1) + [(0.0, 2.0)]

        res = linprog(obj, A_ub=np.array(A_ub), b_ub=np.array(b_ub),
                       A_eq=np.array(A_eq), b_eq=np.array(b_eq),
                       bounds=bounds, method="highs")

        if not res.success:
            if verbose:
                print(f"  N={N}: LP infeasible at refine iter {refine_iter} ({res.message})")
            return None

        x = res.x[:c + 1]
        h = np.concatenate([x, x[:c][::-1]]) if N % 2 == 1 else None

        # verify on fine grid, find worst violations to add back
        fgrid = np.linspace(0.0, NYQUIST, VERIFY_POINTS)
        Mf = amplitude_coeffs(fgrid, c)
        Af = Mf @ x

        pb_mask = fgrid <= PASSBAND_EDGE
        sb_mask = fgrid >= STOPBAND_EDGE

        pb_violation = np.abs(Af[pb_mask] - 1.0) - PASSBAND_TOL
        sb_violation = np.abs(Af[sb_mask]) - STOPBAND_LEVEL

        worst_pb = pb_violation.max() if pb_violation.size else -1
        worst_sb = sb_violation.max() if sb_violation.size else -1

        TOLERANCE = 2e-7
        if worst_pb <= TOLERANCE and worst_sb <= TOLERANCE:
            return h  # clean

        # add worst-violating frequencies back into the design grid
        if worst_pb > TOLERANCE:
            worst_pb_freqs = fgrid[pb_mask][np.argsort(-pb_violation)[:20]]
            pb_grid = np.unique(np.concatenate([pb_grid, worst_pb_freqs]))
        if worst_sb > TOLERANCE:
            worst_sb_freqs = fgrid[sb_mask][np.argsort(-sb_violation)[:20]]
            sb_grid = np.unique(np.concatenate([sb_grid, worst_sb_freqs]))

        if verbose:
            print(f"  N={N}: refine iter {refine_iter}, worst_pb={worst_pb:.2e}, worst_sb={worst_sb:.2e}")

    if verbose:
        print(f"  N={N}: FAILED to converge cleanly after {max_refine} refinements")
    return None


if __name__ == "__main__":
    k0 = derive_k0()
    print("k0 (main-lobe exclusion half-width, samples):", k0)
