BBK Phase Corrector - Claude handoff package

Start with: CLAUDE_BUILD_INSTRUCTIONS.md

The package contains:
- complete JUCE/CMake VST3 source
- embedded phase model and CSV model data
- Windows build script
- standalone DSP smoke-test sources/tools
- original REW .mdat measurement
- phase/impulse reference plots
- technical README

Primary goal: build a Windows x64 VST3 for Audirvana without redesigning the already-validated phase-only DSP.
