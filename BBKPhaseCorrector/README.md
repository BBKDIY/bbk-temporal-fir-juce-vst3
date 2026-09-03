# BBK Phase Corrector VST3

Experimental phase-only VST3 derived from the REW measurement `left new covers Sep 2.mdat`.

## Modes

- **BYPASS**: dry signal delayed by exactly the same reported latency as the correction modes.
- **MIN PHASE**: removes the smoothed excess phase and targets the minimum phase implied by a heavily smoothed measured magnitude response.
- **LINEAR PHASE**: removes the smoothed measured phase and targets linear phase, apart from the plugin's pure delay.

The active correction transfer function is designed with unit magnitude. The measured SPL curve is **not** applied as EQ.

## Phase model

- Measured phase approximation: cubic smoothing spline in `log10(f)`, using a 20 degree RMS smoothing target.
- Minimum-phase target: real-cepstrum reconstruction from the measured magnitude after 0.8 dB RMS smoothing.
- Full correction region: 30 Hz to 16 kHz.
- Raised-cosine fade-in: 20 to 30 Hz.
- Raised-cosine fade-out: 16 to 19 kHz.
- Outside 20 Hz to 19 kHz: plugin correction is identity (apart from the common latency).

The model is sampled at 256 log-spaced frequency points and embedded in `Source/PhaseModel.h`.

## FIR / latency

The FIR duration is about 0.68 seconds and is rounded up to a power of two for each host sample rate. Typical values:

| Host rate | FIR taps | Phase/dry matched latency |
|---:|---:|---:|
| 44.1 kHz | 32768 | 371.5 ms |
| 48 kHz | 32768 | 341.3 ms |
| 96 kHz | 65536 | 341.3 ms |
| 192 kHz | 131072 | 341.3 ms |

This is deliberately long because the experiment is playback-only. The extra latency keeps magnitude error from the finite FIR extremely small at the low-frequency transition.

## Build on Windows

Requirements:

- Visual Studio 2022 with **Desktop development with C++**
- CMake
- Git
- Internet access during CMake configure (JUCE 9.0.0 is fetched automatically)

From PowerShell:

```powershell
./build-windows.ps1
```

or manually:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --parallel
```

The VST3 bundle should appear at:

`build\BBKPhaseCorrector_artefacts\Release\VST3\BBK Phase Corrector.vst3`

Copy that bundle to the VST3 location scanned by Audirvana, then rescan plugins.

## A/B behaviour

Use the plugin's own **BYPASS** button for listening comparisons. It keeps the same FIR latency and crossfades over 25 ms, so switching modes does not create a timing jump or click.

## Validation status

The standalone phase-filter designer has been smoke-tested locally at 44.1, 48, 96 and 192 kHz. The full JUCE target still needs a Windows/JUCE compile in Codex or Visual Studio, because this environment does not contain the JUCE source tree or a Windows VST3 toolchain.

## Licensing note

JUCE has its own licensing terms. Review the JUCE licence that applies before distributing a compiled plugin commercially.

### 48 kHz numerical phase-only check

The exact C++ FIR designer was exported and re-FFT'd at 16x frequency resolution:

| Band | Minimum-phase mode | Linear-phase mode |
|---|---:|---:|
| 20 Hz–19 kHz | -0.0102 to +0.0128 dB | -0.0056 to +0.0058 dB |
| 30 Hz–19 kHz | -0.0041 to +0.0047 dB | -0.0045 to +0.0046 dB |
| 100 Hz–19 kHz | -0.00054 to +0.00048 dB | -0.00073 to +0.00073 dB |

The slightly larger error right at the 20–30 Hz phase fade-in is finite-FIR interpolation, not intentional magnitude EQ.
