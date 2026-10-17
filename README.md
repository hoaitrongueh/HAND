# HAND

Franka Panda kinematics and autonomous DXF drawing in MuJoCo. Python handles CAD,
Mink pose IK and TOPPRA timing; C++ validates trajectories and drives the actual
bounded MuJoCo actuators. The existing positional DLS solver remains a baseline.

## Draw a DXF

Verified on Windows with CPython 3.10.5, MinGW UCRT64/Ninja and MuJoCo 3.16.0 SDK.
Use the matching native SDK and Python MuJoCo version. From this directory:

```powershell
./scripts/setup-drawing.ps1
$env:MUJOCO_DIR = "C:/path/to/mujoco-3.16.0-windows-x86_64"
cmake -S . -B build -G Ninja -DHAND_BUILD_MUJOCO=ON
cmake --build build -j 4
./.venv/Scripts/python -m drawing.run tests/fixtures/house.dxf --output outputs/my-house
```

The output directory must be new. Configure CAD placement, fixed tool quaternion
and lift in `configs/drawing.json`. CAD units are preserved; drawings are never
automatically fitted to the robot. `--plan-only` runs planning without execution.
Inputs outside the supported geometry or robot workspace fail explicitly.

VS Code includes build, dependency-check and test tasks, plus drawing launch
configurations. Set `MUJOCO_DIR` before starting VS Code. Tasks expect the UCRT64
tools in `C:/msys64/ucrt64/bin` and Python in `.venv/Scripts/python.exe`.

Each run produces `planned.csv`, `executed.csv`, a compiled model, JSON diagnostics,
and PNG/SVG overlays. Ink traces use actual post-step tool positions near the
configured virtual plane. There is no force-controlled paper-contact simulation.

Replay a completed run in the MuJoCo viewer:

```powershell
./.venv/Scripts/python simulation/replay_drawing.py outputs/my-house --speed 4 --frame-skip 8
```

```powershell
./.venv/Scripts/python -m pytest -q --disable-warnings
ctest --test-dir build --output-on-failure
```

See [architecture, interfaces and dependency assessment](docs/drawing-architecture.md)
and [batch report, numerical results and known failures](docs/drawing-validation.md).
The new Python suite passed 43 tests. One existing positional IK joint-limit
regression remains failing in CTest; it is documented and preserved.
