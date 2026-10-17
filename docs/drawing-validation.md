# Drawing pipeline implementation batches — 2026-10-17

The first complete CAD-to-actuator pipeline executed three representative DXF
drawings and a tilted-tool variant using the actual Panda affine servos and
MuJoCo stepping. All four completed with 100% measured drawing coverage at 1 mm
tolerance. There were no skipped drawing sections or execution failures.

## Reviewable batches and changed files

| Batch | Files | Interface / verified result |
| --- | --- | --- |
| A | `drawing/__init__.py`, `drawing/geometry.py`, `drawing/preprocess.py`, `scripts/make_drawing_fixtures.py`, `tests/fixtures/{house,curves,dense,unreachable,malformed}.dxf`, `tests/test_geometry.py`, `requirements-geometry.txt` | Ordered `hand.strokes.v1`, original CAD units/WCS, conditional DP, analytic circle/bulge subdivision, strict rejection. House 2 strokes/136 samples; curves 3/107; dense 1/49. |
| A interfaces | `include/hand/drawing.hpp`, `src/drawing.cpp`, `tests/test_drawing.cpp`, `CMakeLists.txt` | C++ `ToolPose`, `DrawingSample`, `PathLimits`, `validate_drawing_trajectory`; rejects invalid pose, joints, timestamps, limits, continuity and unsafe pen boundaries. |
| B | `drawing/planning.py`, `configs/drawing.json`, `tests/test_planning.py` | `ToolPose` position + normalized wxyz; Mink full-pose backend, isolated planning/collision data, `hand.joint_path.v1`, explicit lift/descent/pen states, bounded retries then atomic plan rejection. |
| C | `drawing/timing.py`, `drawing/diagnostics.py`, `drawing/run.py`, `include/hand/execution.hpp`, `src/execution.cpp`, `apps/draw.cpp`, `tests/test_timing.py`, `tests/test_execution.py`, `CMakeLists.txt` | TOPPRA adapter → `hand.trajectory.v1` CSV → C++ validation/actuators → executed CSV; independent full-pose FK, derivative/collision checks, guarded execution, length-weighted coverage. |
| D | `tests/test_pipeline_acceptance.py`, `pytest.ini`, `scripts/probe_dependencies.py`, `scripts/setup-drawing.ps1`, `requirements-drawing.txt`, `README.md`, `docs/drawing-architecture.md`, this file | Executable acceptance and failure isolation; dependency builds, numerical probes, license records and reproducible commands. |

`src/ik.cpp`, its header, original preserved implementation, existing applications,
Panda vendor XML/assets and `simulation/run.py` were not modified. Generated
models, traces and environments remain in ignored folders.

## Numerical outcomes

Maxima over complete runs, SI units unless labeled. Coverage refers to actual
pen-on trace segments, matched by stroke ID; blue PNG/SVG traces are executed FK.

| Drawing | Pose waypoints | Geometric approximation max (mm) | IK position max (µm) | IK orientation max (rad) | Joint waypoint step max (rad) | Execution steps | Simulation time (s) | Joint tracking max (rad) | Cartesian tracking max (mm) | Orientation tracking max (rad) | Coverage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| House | 320 | 0 | 7.90794 | 2.67964e-8 | .00935095 | 33601 | 67.202 | 8.80490e-5 | .0736101 | .000139099 | 100% |
| Curves | 320 | .0481527 | 7.90793 | 2.50766e-8 | .00899533 | 32576 | 65.152 | 8.81242e-5 | .0788902 | .000147293 | 100% |
| Dense sine polyline | 129 | .0184336 | 9.90503 | 2.65882e-8 | .00935095 | 16093 | 32.186 | 8.12624e-5 | .0576521 | .000123035 | 100% |
| House, tool tilted .1 rad | 320 | 0 | 7.90798 | 6.98160e-6 | .00934780 | 33492 | 66.984 | 8.79602e-5 | .0752568 | .000142884 | 100% |

Geometric approximation uses exact circular sagitta or measured original vertex
distance to simplified chords. The configured global approximation bound is .05
mm for these millimetre fixtures. House and dense timing maximum velocities are
approximately .05784 rad/s; maximum acceleration stays below .36 rad/s², against
configured .2 rad/s and .4 rad/s² limits. Maximum timed joint increment is below
.000116 rad. Full-pose FK interpolation residual remains below .01 mm for these
runs. Detailed sampled/curve derivative maxima are in individual JSON reports.

Whole-pipeline measured wall times were 20.321 s (house), 18.580 s (curves), 6.498 s
(dense), and 20.758 s (tilted), with parallel validation work also running. These
are environment observations, not real-time or performance guarantees. House
requested/covered stroke length is .2647106781 m; curves .2069771168 m; dense
.0602076076 m. The controller executed meaningful joint excursions (~.5 rad),
with nonzero actuator forces; it did not write planned qpos into live physics.

## Commands actually executed and evidence

Commands ran from the repository root.

```powershell
python -m venv .venv
./.venv/Scripts/python -m pip install ezdxf==1.4.3 mink==1.1.0 toppra==0.6.3 mujoco==3.16.0 pytest
# Initial TOPPRA build failed: MSVC unavailable; separate installed candidates.
./.venv/Scripts/python -m pip install ezdxf==1.4.3 mink==1.1.0 mujoco==3.16.0 numpy==2.2.6 scipy==1.15.3 pytest==8.4.2
./.venv/Scripts/python -m pip install --upgrade pip
# Unconfigured MinGW attempt failed CPython pointer-size check.
./.venv/Scripts/python -m pip wheel toppra==0.6.3 --no-deps --wheel-dir build/dependency-wheels --config-settings=--build-option=build_ext --config-settings=--build-option=--compiler=mingw32 --config-settings=--build-option=--define=MS_WIN64
./.venv/Scripts/python -m pip install build/dependency-wheels/toppra-0.6.3-cp310-cp310-win_amd64.whl
./.venv/Scripts/python scripts/probe_dependencies.py
./.venv/Scripts/python scripts/make_drawing_fixtures.py
cmake --build build -j 4
ctest --test-dir build --output-on-failure
./.venv/Scripts/python -m pytest -q --disable-warnings --basetemp=build/pytest-final
./.venv/Scripts/python -m drawing.run tests/fixtures/house.dxf --output outputs/final-house
./.venv/Scripts/python -m drawing.run tests/fixtures/curves.dxf --output outputs/final-curves
./.venv/Scripts/python -m drawing.run tests/fixtures/dense.dxf --output outputs/final-dense
./.venv/Scripts/python -m drawing.run tests/fixtures/house.dxf --config outputs/tilted-config.json --output outputs/final-tilted
./.venv/Scripts/python -m drawing.run tests/fixtures/unreachable.dxf --output outputs/rejected-unreachable
./.venv/Scripts/python -m drawing.run tests/fixtures/malformed.dxf --output outputs/rejected-malformed
./.venv/Scripts/python -m pip check
git diff --check
```

Tilted configuration is the default tool quaternion left-multiplied by
`mink.SO3.exp([.1,0,0])`; the CAD plane stays unchanged. The plane is an explicit
executor argument, allowing tool tilt independently of plane orientation.

- Final pytest: **43 passed**, 59 upstream pyparsing deprecation warnings, 67.75 s.
  Log: `outputs/pytest-final.log`. Includes 3 real drawing acceptances, malformed
  and unreachable pipeline cases, bounded failed IK, orientation-only IK change,
  initial/near joint limits, quaternion validation, TOPPRA failure with no fallback,
  trajectory parsing/validation and real C++ stepping. New C++ contract test passes.
- Full CTest: **36/37 passed**; original `panda_ik.reachable_joint_limit` still fails,
  just as before implementation (original suite was 35/36). Residual
  .2515847643610436 m, 7 updates, joint_limit. Log: `outputs/ctest-drawing.log`.
  No baseline test was hidden or changed. Focused CTest core/contract: 2/2 passed.
- `pip check`: no broken requirements. `git diff --check`: clean.
- Artifacts: `outputs/final-{house,curves,dense,tilted}/` each contains
  `strokes.json`, `joint_path.json`, `planned.csv`, `executed.csv`, `model.mjb`,
  `execution.json`, `report.json`, `executor.log`, `traces.png`, and `traces.svg`.
- Dependency probe: `outputs/dependency-probe.json`; exact installed packages:
  `outputs/drawing-environment.lock.txt`. Aggregate: `outputs/final-results.json`.

## Failures, resolution and remaining risks

Malformed DXF fails in geometry. The unreachable fixture fails pose IK after three
bounded attempts and produces no execution trace. Invalid CSV numerical values,
joint bounds, quaternion, pen state, time, model FK and timestep are rejected
before stepping. No unvalidated recovery motion or partially planned drawing is
executed. These failures are intentional passing tests.

Two TOPPRA compatibility failures were found and resolved before successful
adoption: the default time spline exceeded acceleration limits, and unit-interval
parameter scaling failed a feasible curve edge. See `drawing-architecture.md` for
their measured reproductions and validated configuration fixes. Initial failed
run artifacts under `outputs/acceptance-curves` and `outputs/run-curves` remain as
historical evidence; final run artifacts and final acceptance tests passed.

Remaining limits: Windows/MuJoCo 3.16.0 validation only, a massless virtual pen and
drawing plane rather than force-controlled paper contact, explicitly limited DXF
entity support, sampled collision checks, and conservative stops at waypoints.
Only bounded retries followed by halt are implemented; no skip/recovery planner
is claimed. QP dependency qpsolvers is LGPL-3.0; retain relevant license/source
obligations when packaging. Root project licensing remains the owner's decision.

The baseline positional IK defect is preserved for comparison and remains visible
independently of the drawing solver.
