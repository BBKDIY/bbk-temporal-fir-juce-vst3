# BBK Temporal FIR

A real-time A/B tool for the time-domain Pareto trade-off between peak
sidelobe amplitude and settling duration, at a fixed frequency-domain
specification:

- Fs = 192000 Hz
- Passband: 0-20 kHz, `|A(f)-1| <= 1e-4`
- Stopband: 76-96 kHz, `|A(f)| <= -100.3 dB`
- Type-I, odd-length, exactly symmetric, linear-phase FIR

This extends the accompanying article's "black curve" methodology - which
demonstrates the tradeoff at a single, fixed tap count - into an explicit
sweep across tap count as an additional, explored degree of freedom, and
turns the resulting Pareto frontier into something you can move a slider
across in real time and actually listen to.

## Why not just Parks-McClellan/Remez at each tap count?

Because minimum-*peak-sidelobe* and minimum-*stopband-ripple* (what
Parks-McClellan actually optimises) are different objectives, and the
article is explicit that they need not share an optimum. Confirmed
directly here: a plain Remez design at N=19 with these exact edges lands
at roughly 61-65% peak sidelobe; the LP-optimal minimum-peak-sidelobe
design at the same N and same edges reaches 14.4%. See `Coefficients/`.

## How the coefficient bank was built

1. `solve_fir.py` - `derive_k0()` designs a 19-tap Parks-McClellan
   reference filter at the project's exact band edges and finds the first
   sign change outward from the centre tap. That offset (K0 = 3 samples)
   is fixed once and used as the main-lobe exclusion boundary for every
   tap count in the sweep - not re-derived per N. This convention is an
   extension of the article's own methodology, not a value reported in
   it, and is documented here for that reason.
2. For every odd N from 19 to 127, `solve_fir.solve_one()` sets up the
   Type-I zero-phase amplitude response as a linear function of the free
   (half) coefficients and solves a linear program: minimise the peak
   sidelobe tap magnitude subject to the passband/stopband constraints
   above (plus an exact `A(0)=1` equality), with iterative dense-grid
   constraint refinement so the design is verified clean on a
   131072-point frequency grid, not just the coarser LP design grid.
3. `run_sweep.py` computes the article's time-domain metrics for every
   accepted design (peak sidelobe %, total ringing energy %, one-sided
   settling duration, worst passband/stopband levels, group delay),
   saves the full sweep to `Coefficients/fir_sweep_full.csv`, computes
   the 2-objective Pareto-efficient frontier (minimise peak sidelobe %,
   minimise settling duration; ringing energy is displayed but not used
   to prune the frontier), force-includes the N=19 baseline even though
   it is independently Pareto-efficient here, and saves
   `Coefficients/fir_sweep_pareto.csv`.
4. `gen_cpp_header.py` zero-pads every Pareto-set filter's native N-tap
   array, centred, out to 127 samples (`Source/TemporalFIRBank.h`).
   Padding a symmetric FIR with equal zeros on both sides does not change
   its frequency response - it only reports the same impulse response at
   a longer nominal length, which is mathematically identical to running
   the native filter and adding `(63 - nativeGroupDelay)` samples of pure
   extra delay, but lets the plugin use one shared 127-tap convolution
   loop and one shared history buffer for every selectable filter, with
   no per-filter special-casing.

All of this runs offline, in this repository, with Python/NumPy/SciPy.
**The compiled plugin needs none of it at runtime** - it only contains
the precomputed `TemporalFIRBank.h` data.

## Plugin

- BYPASS/ACTIVE toggle.
- TEMPORAL TRADE-OFF slider: snaps only to the Pareto-efficient tap
  counts in the bank (no coefficient interpolation between them).
  SHORTEST RESPONSE at the left, LOWEST PEAK SIDELOBE at the right.
- Live readout: tap count, peak sidelobe %, ringing energy %, settling
  time, stopband level, host sample rate.
- Every selectable filter (and bypass) shares exactly the same latency -
  `maxGroupDelaySamples = 63` samples at 192 kHz (the N=127 ceiling's own
  group delay) - reported to the host via `setLatencySamples()`, constant
  regardless of which filter is selected.
- Switching filters crossfades the old and new filter's outputs in
  parallel over ~15 ms; toggling bypass crossfades dry/wet over ~10 ms.
  Neither can click, because the currently-audible signal never changes
  discontinuously.
- Host rate must be exactly 192000 Hz or the plugin hard-bypasses and
  shows "REQUIRES 192 kHz" - no internal resampling.

## Validation

`Tests/DSPTest.cpp` is framework-independent (no JUCE) and checks, for
every filter in the bank: exact symmetry, unity DC gain, the passband and
stopband constraints on a fine frequency grid, that the metrics the
plugin displays match a from-scratch recomputation from the stored taps,
and that the taps are exactly zero outside each filter's own true
support. A local mirror of the plugin's exact per-sample algorithm then
checks the system's impulse response equals the padded taps directly,
that bypass and every filter share identical latency, and that two
independently-driven channels never leak into each other. See
`VALIDATION.txt` for a saved run. This test is a required, blocking step
in CI before the plugin itself is built.

## Windows build

Prerequisites:
- Visual Studio 2022 with Desktop development with C++
- CMake 3.22+
- Git (only needed if JUCE is not copied into `./JUCE`)

From "Developer Command Prompt for VS 2022":

    cmake -S . -B build -G "Visual Studio 17 2022" -A x64
    cmake --build build --config Release --target BBKTemporalFIR_VST3

Expected bundle:

    build\BBKTemporalFIR_artefacts\Release\VST3\BBK Temporal FIR.vst3

Copy the entire `.vst3` bundle to:

    C:\Program Files\Common Files\VST3\

JUCE is pinned to 8.0.15. If a `JUCE` folder exists beside this
CMakeLists, the project uses that local checkout instead of downloading
JUCE.

## Cloud build (no local toolchain needed)

Push to `main`/`master`, or trigger the workflow manually - GitHub
Actions (`.github/workflows/build-windows.yml`) builds on a real Windows
MSVC runner and uploads the compiled `.vst3` as a workflow artifact. No
Visual Studio, CMake, JUCE, Python, or Git is required on the machine
that will actually run the plugin.
