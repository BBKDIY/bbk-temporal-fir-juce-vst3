$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --parallel

Write-Host ""
Write-Host "Build complete. VST3 should be under:"
Write-Host "  build\BBKPhaseCorrector_artefacts\Release\VST3\BBK Phase Corrector.vst3"
