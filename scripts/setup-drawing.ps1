param(
    [string]$Python = 'python',
    [string]$MingwBin = 'C:\msys64\ucrt64\bin'
)
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
if (-not (Test-Path '.venv/Scripts/python.exe')) {
    & $Python -m venv .venv
    if ($LASTEXITCODE) { throw 'venv creation failed' }
}
$drawingPython = (Resolve-Path '.venv/Scripts/python.exe').Path
$env:PATH = $MingwBin + ';' + $env:PATH
& $drawingPython -m pip install 'pip==26.2.1'
if ($LASTEXITCODE) { throw 'pip setup failed' }
& $drawingPython -m pip wheel 'toppra==0.6.3' --no-deps --no-binary=toppra --wheel-dir build/dependency-wheels '--config-settings=--build-option=build_ext' '--config-settings=--build-option=--compiler=mingw32' '--config-settings=--build-option=--define=MS_WIN64'
if ($LASTEXITCODE) { throw 'TOPPRA build failed; stop before pipeline execution' }
$drawingWheel = Get-ChildItem 'build/dependency-wheels/toppra-0.6.3-*.whl' | Select-Object -First 1
& $drawingPython -m pip install $drawingWheel.FullName
if ($LASTEXITCODE) { throw 'TOPPRA installation failed' }
& $drawingPython -m pip install -r requirements-drawing.txt
if ($LASTEXITCODE) { throw 'Dependency installation failed' }
& $drawingPython scripts/probe_dependencies.py
if ($LASTEXITCODE) { throw 'Dependency numerical probe failed; do not execute drawings' }
