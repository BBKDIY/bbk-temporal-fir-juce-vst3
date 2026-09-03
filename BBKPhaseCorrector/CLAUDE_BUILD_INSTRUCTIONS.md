# Claude handoff: BBK Phase Corrector VST3

## Goal

Build and validate the supplied Windows x64 JUCE VST3 for Audirvana. This is the next experiment after the earlier Black-19 VST3 project. Please reuse the Windows/JUCE/VST3 build fixes and install/testing approach that already worked for Black-19 where applicable.

The intended output is:

`BBK Phase Corrector.vst3`

Do not redesign the phase model during the first build. The DSP source in this package has already been numerically smoke-tested outside JUCE. If JUCE/API/build fixes are required, preserve the DSP behaviour unless a real bug is demonstrated.

## What the plugin must do

It has three live modes:

1. **BYPASS**
   - Dry signal only.
   - Must retain the same overall delay as the active correction modes so A/B comparisons do not jump in time.

2. **MIN PHASE**
   - Pure phase correction toward the minimum phase implied by the heavily smoothed measured magnitude response.
   - Removes the smoothed *excess phase* only.
   - No intentional magnitude EQ.

3. **LINEAR PHASE**
   - Pure phase correction toward a linear-phase overall system response, apart from the common plugin latency.
   - No intentional magnitude EQ.

Switching between modes must be click-free. The current implementation crossfades over 25 ms.

## Measurement/model provenance

The filter was derived from the REW measurement:

`reference/left new covers Sep 2.mdat`

This measurement includes the complete upstream/acoustic chain used for the experiment: DAC -> preamp -> power amp -> crossover -> drivers -> microphone, with REW timing reference. The purpose of the smooth model is to follow broad phase behaviour and deliberately reject narrow room/reflection wiggles.

Reference plots are included in `reference/`:

- `measured_phase_relative.png`
- `phase_fit_comparison.png`
- `phase_corrected_impulse_comparison.png`

The actual embedded correction model is in:

- `Source/PhaseModel.h`
- `phase_model.csv`

## DSP design that should be preserved

- Measured phase approximation: cubic smoothing spline in `log10(f)`.
- Minimum-phase target: derived from the heavily smoothed measured magnitude using a real-cepstrum reconstruction.
- Full correction region: 30 Hz to 16 kHz.
- Smooth fade-in: 20 to 30 Hz.
- Smooth fade-out: 16 to 19 kHz.
- Outside 20 Hz to 19 kHz the correction tends to identity, apart from common latency.
- The active correction transfer functions are phase-only. The measured SPL curve is **not** applied as EQ.
- FIR duration is approximately 0.68 s, rounded to a power-of-two tap count at the host sample rate.

Typical tap count / matched delay:

| Host rate | FIR taps | Common delay |
|---:|---:|---:|
| 44.1 kHz | 32768 | ~371.5 ms |
| 48 kHz | 32768 | ~341.3 ms |
| 96 kHz | 65536 | ~341.3 ms |
| 192 kHz | 131072 | ~341.3 ms |

This deliberately large latency is acceptable: this plugin is for playback/listening experiments, not live monitoring.

## Numerical validation already performed

At 48 kHz, the exact C++ FIR designer was exported and re-FFT'd at 16x frequency resolution.

Magnitude deviation:

| Band | Minimum-phase mode | Linear-phase mode |
|---|---:|---:|
| 20 Hz–19 kHz | -0.0102 to +0.0128 dB | -0.0056 to +0.0058 dB |
| 30 Hz–19 kHz | -0.0041 to +0.0047 dB | -0.0045 to +0.0046 dB |
| 100 Hz–19 kHz | -0.00054 to +0.00048 dB | -0.00073 to +0.00073 dB |

The standalone designer was also smoke-tested at 44.1, 48, 96 and 192 kHz.

## Build

The project uses CMake and currently fetches JUCE 9.0.0.

Preferred build command from a VS 2022 x64-capable PowerShell environment:

```powershell
./build-windows.ps1
```

Equivalent manual build:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --parallel
```

Expected bundle location:

`build\BBKPhaseCorrector_artefacts\Release\VST3\BBK Phase Corrector.vst3`

If the Black-19 project required specific CMake generator, JUCE path, SDK, signing, bundle-copy, or Audirvana scan workarounds, use the already-proven approach here too. Build-system/JUCE compatibility edits are fine; preserve the phase-filter design.

## First validation checklist

Please do all of the following before returning the build:

- Build Release x64 VST3 successfully.
- Confirm the `.vst3` bundle is complete, not merely a DLL copied out of the bundle.
- Confirm the plugin is stereo and accepts normal audio with no MIDI requirements.
- Confirm it loads/scans in the same VST3 test host or Audirvana setup that worked for Black-19.
- Confirm **BYPASS / MIN PHASE / LINEAR PHASE** are visible and switchable.
- Confirm switching does not click badly or reset playback.
- Confirm bypass is latency-matched to the correction modes.
- Confirm reported host latency is sensible for the current sample rate.
- Confirm 44.1, 48, 96 and 192 kHz initialise without assertion/crash if those rates are available in the test host.
- Run the included standalone designer smoke test if useful.
- If practical, render an impulse through BYPASS, MIN PHASE and LINEAR PHASE and verify that the two active modes alter phase/IR while keeping the magnitude essentially unchanged.

## Important: what not to "fix" casually

- Do not shorten the FIR just to reduce latency. The long FIR was chosen intentionally to make low-frequency phase correction nearly magnitude-neutral.
- Do not apply the measured SPL as inverse EQ.
- Do not convert this into a conventional linear-phase magnitude equaliser.
- Do not remove the matched dry delay in BYPASS.
- Do not replace the embedded model with point-by-point room-phase inversion.
- Do not force zero latency.

If you find a genuine DSP or latency bug, document it and fix it, but distinguish that from a build/API issue.

## Deliverables back to Bojidar

Please return:

1. The compiled `BBK Phase Corrector.vst3` bundle.
2. The final source tree if you had to make build fixes.
3. A short list of changes made relative to this package.
4. The exact install path / scan steps used for Audirvana if they differ from the Black-19 setup.
5. Any validation result or impulse render you generated.

The immediate listening test is simply:

**BYPASS -> MIN PHASE -> LINEAR PHASE**

at matched volume and matched latency.
