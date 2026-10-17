# Autonomous drawing implementation

Pipeline: ordered DXF → geometric sampling → full tool pose → feasible joint
path → library timing → C++ actuator execution → measured drawing trace. Existing
`src/ik.cpp`, `docs/panda_ik_original.cpp`, and `simulation/run.py` remain baselines.

## Repository inspection (2026-10-17, Asia/Saigon)

- Existing C++ DLS solves hand-body **position only**, using MuJoCo FK/Jacobians.
  It modifies caller-supplied planning data; it is not an execution controller.
- Python reference executes real `ctrl` position targets through `mj_step`.
- Panda Menagerie model: seven hinge joints, eight general actuators (last is
  gripper tendon); affine bias implements position/velocity feedback. Arm gains
  4500/3500/2000, bounded joint targets, torque bounds 87 or 12 Nm. Default dt 2 ms.
- No pen, tool site, drawing plane, geometry parser, pose IK or timing dependency
  existed. A separate model adapter must add a tool site without editing vendor XML.
- Existing CMake expects Windows SDK layout `include/`, `lib/mujoco.lib`,
  `bin/mujoco.dll`. Installed SDK 3.16.0 works with existing MinGW/Ninja build.
- Fresh existing CTest: 35/36 passed; `panda_ik.reachable_joint_limit` failed,
  residual 0.2515847643610436 m, 7 updates, joint_limit. Keep visible as baseline.
- No repository root license was found. Distribution licensing remains an owner
  decision; dependency notices must accompany any redistribution.

## Stable boundaries

1. `hand.strokes.v1` JSON (`Drawing.to_dict`): DXF `$INSUNITS`, metres per CAD unit,
   ordered entity handles, original WCS coordinates in CAD units, closed flag,
   source/simplified vertex counts, approximation bound in CAD units. No ordering,
   unit guessing, deduplication, auto-fit, reflection, or stroke merging.
2. Tool target: world position in metres + normalized quaternion **wxyz**. CAD
   placement is explicit translation + proper rotation; fixed planar tool
   orientation is independent of CAD traversal direction.
3. Joint path: seven Panda arm angles in radians, pose residuals, stroke ID and
   pen state. Planning owns separate MuJoCo data. Every transition is validated.
4. Timed command stream: versioned CSV, monotonic seconds, seven joint targets,
   expected full tool pose, pen state and stroke ID. Pen-up motion is explicit.
   Header: `# hand.trajectory.v1`, then
   `time,q1,q2,q3,q4,q5,q6,q7,x,y,z,qw,qx,qy,qz,pen_down,stroke`.
   All commands align with the model's physics timestep. The drawing plane origin
   and unit normal are explicit executor arguments, independent of tool tilt.
5. Execution: C++ owns fresh physics state; initial home reset only, then actuator
   controls + `mj_step`. Record actual joints, tool pose, actuator effort and state.
   Planned FK never writes live physics qpos. Fail closed on invalid commands.

## Batch A geometry scope

Supported: LINE, ARC, CIRCLE, LWPOLYLINE (including bulges), 2D/3D POLYLINE.
OCS-to-WCS conversion preserves extrusion/orientation. Unsupported entities fail
the whole import, rather than silently reducing coverage. Unitless drawings need
an explicit unit override. Empty, corrupt and degenerate geometry is rejected.

Only sufficiently dense *linear* polylines are simplified. Douglas-Peucker is not
duplicate-line detection. Curves use exact circular sagitta subdivision; all
chords are then bounded by maximum length. Approximation bound is the maximum of
curve deviation and simplification tolerance (they are applied to disjoint types).
All tolerances are expressed in source CAD units; placement converts to metres.

## Candidate dependencies and sources

- ezdxf 1.4.3: MIT, Python + Windows wheel; units are metadata and coordinates
  remain unitless until explicitly converted. [Units](https://ezdxf.readthedocs.io/en/stable/concepts/units.html),
  [license](https://github.com/mozman/ezdxf/blob/master/LICENSE).
- Mink 1.1.0: Apache-2.0; differential SE(3) IK on the exact MuJoCo model, avoiding
  a second URDF kinematics description; QP backend and bounded iteration adapter
  must be tested. [Upstream](https://github.com/kevinzakka/mink).
  Its qpsolvers 4.13.0 dependency is **LGPL-3.0** (installed metadata); DAQP 0.10.3
  is MIT. They are imported as separate Python packages, with no changes to their
  internals. Preserve licenses and source availability obligations if distributing
  the environment; no proprietary distribution compatibility is asserted here.
- TOPPRA 0.6.3: MIT; velocity/acceleration constrained path timing, not torque or
  collision certification. Windows source build and actual output limits require
  a probe before integration. [Upstream](https://github.com/hungpham2511/toppra).
- MuJoCo 3.16.0 and Panda model: Apache-2.0; Eigen: MPL-2.0 core (retain vendor
  notices). Installed package metadata/licenses will be recorded after testing.

Failure policy: bounded local IK retries; if no validated recovery exists, halt
before physics execution. No unvalidated jump or silent skipped drawing section.
Safe halt of simulation means stop stepping and disable pen marking, not a claim
of certified hardware braking. Hardware and force-controlled ink contact excluded.

## Verified integration choices

Mink's `FrameTask` solves both translation and rotation. The adapter freezes the
finger DOFs with equality constraints, enables configuration limits, warm starts
from the previous accepted waypoint, and performs at most three attempts of 100
updates each. Failure restores the previous accepted planning configuration and
aborts the complete plan. It never skips a section or launches a partial plan.
Joint edges are checked at five fractions and again at every timed command for
joint limits, collision and full-pose FK residual. These are sampled feasibility
checks, not a mathematical continuous collision certificate.

TOPPRA 0.6.3 was successfully built with the existing 64-bit MinGW compiler using
standard setuptools options. CPython 3.10's Windows `pyconfig.h` defines MS_WIN64
inside its MSVC branch; MinGW needed `build_ext --define=MS_WIN64`. No vendor
source was patched and no system compiler was installed. The initial default
MSVC build and unconfigured MinGW build failed and were not used.

The default `ParametrizeSpline` exceeded the acceleration probe limit: 1.1303218251
rad/s² against a 1 rad/s² constraint. `ParametrizeConstAccel` with a dense grid
passed: 0.3162277654 rad/s maximum velocity, 1.000000000000067 rad/s² maximum
acceleration. The adapter uses 90% of configured limits and independently checks
analytic trajectory samples and command finite differences. Timing stops at every
joint waypoint, including sampled points on straight/curved strokes. This is
conservative and intentionally slower than smooth whole-stroke timing.

A unit path parameter interval produced a false TOPPRA infeasibility on curve
edge 173 (joint increment 0.00179449143 rad, Cartesian length 0.000992998017 m).
Parameterizing each edge by its largest joint displacement makes max |dq/ds| = 1
and resolves this numerical conditioning failure without changing joint geometry.
The complete curve fixture is an acceptance regression for this adapter choice.

The C++ executor independently checks the CSV, joints, derivatives, home state,
full-pose FK, sampled collisions, plane, timestep and affine servo definitions
before any live stepping. It computes position-servo feedforward for velocity
damping and MuJoCo bias forces; **bounded model actuators** still generate every
execution force. Live qpos is initialized only by a home keyframe reset. Execution
guards halt on control bounds, tracking error, joint limits, penetration, physics
warnings or trace write failure. Pen marking is enabled only when requested and
within 2 mm of both the target and drawing plane, with orientation error < .01 rad.

## Reproduction

From the repository root in PowerShell (tested with CPython 3.10.5, 64-bit,
MinGW UCRT64, Ninja, and native MuJoCo 3.16.0):

```powershell
./scripts/setup-drawing.ps1
$env:MUJOCO_DIR = "C:/path/to/mujoco-3.16.0-windows-x86_64"
cmake -S . -B build -G Ninja -DHAND_BUILD_MUJOCO=ON
cmake --build build -j 4
./.venv/Scripts/python scripts/probe_dependencies.py
./.venv/Scripts/python -m pytest -q --disable-warnings
ctest --test-dir build --output-on-failure
./.venv/Scripts/python -m drawing.run tests/fixtures/house.dxf --output outputs/my-house-run
./.venv/Scripts/python -m drawing.run tests/fixtures/curves.dxf --output outputs/my-curves-run
./.venv/Scripts/python -m drawing.run tests/fixtures/dense.dxf --output outputs/my-dense-run
```

Output directories must be new to prevent stale evidence. `--plan-only` produces
geometry, joint path, timed commands and compiled model without execution. CAD-only
use: install `requirements-geometry.txt`, then
`python -m drawing.preprocess INPUT.dxf OUTPUT.json`. Geometry tolerances are CAD
units; default DXF import uses 2 unit max segment, .05 unit curve deviation and
.02 unit simplification at 32+ polyline vertices. Drawing size and placement are
never automatically fitted. `configs/drawing.json` specifies placement in metres,
CAD-frame quaternion, independent fixed tool quaternion, lift and IK settings.
`--velocity` / `--acceleration` set uniform arm limits in radians/s and radians/s².

Each run records source SHA-256, configuration, package versions, geometry error,
IK residuals, path continuity, timing limits, exact executor command, simulation
time, wall time, tracking, failures and length-weighted coverage. Binary MJB files
require matching MuJoCo versions. `planned.csv` and `executed.csv` are separate
artifacts; the latter contains actual post-step site pose, joints, servo controls,
forces, requested/actual pen state and tracking residuals. PNG/SVG overlays use
only recorded executed positions for their blue ink trace.

## Product limits and scope

- This first product draws by tracing the actual massless tool-tip position near
  a virtual plane. Pen-up/down controls marking and physically lifts/descends the
  tool. There is no force-controlled paper contact, ink deposition physics, or
  pen actuator in the vendor model. The acceptance is actuator-driven tool motion.
- Planar CAD must lie in CAD XY; rotated/translated drawing planes and independently
  tilted fixed tool orientations are represented. Arbitrary 3D strokes, varying
  orientation DXF import, and unsupported DXF entity adapters are not implemented.
- Drawing/path sampling has a 200,000-point geometry budget; executor CSV budget
  is 2,000,000 commands. Inputs exceeding these limits fail explicitly.
- Coverage is a length-weighted estimate over sampled CAD chord midpoints and
  actual pen-on segments of the same stroke, at 1 mm tolerance. Geometry
  approximation error, pose IK residual and actuator tracking remain distinct.
- No torque-feasibility proof, jerk bound, global IK completeness or certified
  recovery planner is claimed. Bounded retries then halt are the implemented
  recovery policy. Physics checks guard the actual execution.
- No approved architectural requirements were replaced. Library configuration,
  supported DXF entity subset and conservative stops are explicit implementation
  choices. Existing positional IK and Python reference are untouched.
