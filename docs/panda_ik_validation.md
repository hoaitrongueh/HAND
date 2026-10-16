# Panda position IK adversarial validation

Executed locally on **2026-10-17, Asia/Saigon**, against the supplied working tree, not the published repository. The solver defects below are deliberately retained. Full CTest exits **8**: **31 entries executed, 17 passed, 14 failed**. A separate MuJoCo-disabled build executed the original 2D test: **1/1 passed**. Across these final configurations: **32 entries, 18 passed, 14 failed**; earlier exploratory reruns are not added to these totals.

## Changes and extraction boundary

| File | Change |
| --- | --- |
| `include/hand/ik.hpp` | Position-only solver interface, unchanged defaults, result/termination/step trace |
| `src/ik.cpp` | Small extraction of the local demo loop; same full-DOF Jacobian, LDLT DLS, seven named-joint mapping and transactional range validation |
| `apps/panda.cpp` | Thin model/home setup and solver demonstration |
| `tests/test_panda_ik.cpp` | Standalone C++17 tests, deterministic stress cases, CSV evidence and runtime summaries; no testing framework added |
| `CMakeLists.txt` | Optional `hand_ik` library, Panda test executable, CTest entries/timeouts and MuJoCo DLL copying using existing dependencies |
| `README.md` | Test execution and evidence/report instructions |
| `docs/panda_ik_validation.md` | This report |

The existing 2D implementation/tests and third-party/model files were not edited. Pre-existing changes to VS Code files, simulation code, build configuration and README were preserved. All generated binaries, fixtures, logs and CSV files are under `build/`.

Default damping **0.05**, step size **0.5**, tolerance **0.001 m**, and budget **100** remain unchanged. `IkOptions` permits explicitly shortened budgets for the terminal-boundary and atomicity tests; these changes are confined to those calls. The solver still checks convergence before each update, computes DLS with all `nv` columns and commits only `joint1` through `joint7`. It still lacks finite-value validation. None of these behaviors was repaired.

Added observations are `success`, final error, committed-update count, termination reason, and proposal error/acceptance trace. A rejected proposal does not increment the update count. Model/body validation originally done by the demo is also represented at the new callable boundary, including null pointer handling. The model/data must match, with current `mj_forward` results; the API does not validate mismatched allocations. No `home` keyframe is required by the solver itself. The demo still requires `home`, returns its original zero exit status after an IK failure, and now prints a concise trace/result instead of per-iteration matrices. Tests inspect `IkResult` directly.

The original local demo source and executed baseline output are preserved in `build/ik_validation/panda_before_extraction.cpp` and `original_baseline.txt`. The extracted demo was also executed, with output in `extracted_baseline.txt`. Its baseline is unchanged:

| Update count | Error (m) |
| --- | --- |
| 0 | 0.020000 |
| 1 | 0.0104409 |
| 2 | 0.0054479 |
| 3 | 0.00284176 |
| 4 | 0.00148214 |
| 5 | 0.00077301406450712537 |

The regression permits absolute differences of `6e-8 m` in the supplied rounded intermediate values and `6e-9 m` in the final value, and requires five updates.

## Build, execution and evidence

Environment: Windows, MSYS2 UCRT64 GNU C++ **16.2.0**, CMake **4.4.3**, Ninja **1.13.2**, **Debug**, vendored Eigen **5.0.0**, native MuJoCo **3.16.0**. No local dependency blocker occurred. The source/model paths and runtime versions appear in every test log; file hashes are recorded in `build/ik_validation/environment.txt`.

From the repository root, using the existing UCRT64 tools on PATH:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DHAND_BUILD_MUJOCO=ON -DMUJOCO_DIR="C:/Users/lethu/Downloads/mujoco-3.16.0-windows-x86_64"
cmake --build build
ctest --test-dir build --output-on-failure --output-log build/ik_validation/ctest.log
```

All required targets built successfully without compiler diagnostics. Full CTest took **9.68 s** on this run. Each Panda entry has a **60 s** timeout; iteration loops are bounded. Red cases have the `known_defect` label, with no `WILL_FAIL` inversion or disabled tests. For diagnosis:

```powershell
ctest --test-dir build -L known_defect --output-on-failure
./build/test_panda_ik.exe reachable_joint_limit
./build/test_panda_ik.exe stress
```

Each invocation overwrites only its own case's CSV/trace files. Normal execution works from any current directory: model/evidence paths are set by CMake. The demo retains its repository-relative model path. CTest uses `build/` as its working directory. Two test-only XML fixtures are generated there; original model files remain unchanged.

Optional-dependency compatibility was also configured/built/executed:

```powershell
cmake -S . -B build/core_only -G Ninja -DCMAKE_BUILD_TYPE=Debug -DHAND_BUILD_MUJOCO=OFF
cmake --build build/core_only
ctest --test-dir build/core_only --output-on-failure
```

Evidence under `build/ik_validation/`:

- `ctest.log`, `ctest_console.txt`, `ctest_exit_code.txt`: full execution/failures and exit status; `build/Testing/Temporary/LastTest.log` includes all passing-case output.
- `configure.log`, `build.log`, `build_all.log`, `core_*`: configuration/build and optional-off test evidence.
- `<case>.csv`: 17-digit initial full qpos, target, FK witness when applicable, final full qpos, success/error/update count/reason, solver time, increasing/stagnant-step counts and net-worse flag.
- `<case>_trace.csv`: every attempted proposal, error before/after and whether committed.
- `stress.csv`: **all 180 stress cases**, including each failure; console output also prints every individual configuration and result. The seed is **1212239428 / 0x48414e44**.
- `stress_first_run.csv`: earlier seed run for determinism comparison. All 180 numerical/outcome records matched across reruns when timing was excluded.

The final main run made **247 logged solver calls plus six direct model/entity guard checks**. CTest entries are groups of scenarios rather than one entry per solve. The original 2D CTest entry remained separate.

## Coverage and results

Passing groups: original `kinematics`; Panda `baseline`, `directions`, `tolerance`, `displacements`, `configurations`, `near_limits`, `jacobian`, `indexing`, `atomic_rejection`, `disabled_limits`, `invalid_model`, `missing_joint`, `unreachable`, `constrained_target`, `singular`, `stress`.

Failing groups: `missing_joint_at_target`, `last_update`, `nonfinite_state`, `invalid_state_limits`, `reachable_joint_limit`, plus `nan_*`, `inf_*`, `neginf_*` on each of X/Y/Z (**nine** invalid-target entries).

Passing exploration groups do **not** imply all targets converged. Those groups verify finite bounded results, accurate status/error reporting for success, state safety, and record failures. The strict `reachable_joint_limit` case preserves the recovery weakness separately.

| Investigation | Observation |
| --- | --- |
| Home ±20 mm X/Y/Z | All six converged |
| Already within tolerance | Zero/0.5/0.999 mm offsets converged without movement; 1.001 mm required an update |
| Increasing +X displacement | 5/20/50/100/200 mm converged in 3/5/7/8/10 updates; 400 mm exhausted 100 updates; 800 mm rejected a joint-limit proposal after one update |
| Twelve deterministic valid starts, small Cartesian offset | All converged; exact starts/results in `configurations.csv` |
| Fourteen individual joint-boundary starts | 10 converged, 4 rejected; all finite states stayed inside enabled limits |
| Position Jacobian finite differences | 12 configurations × 3 perturbations (`1e-5`, `1e-6`, `1e-7 rad`), central difference; maximum discrepancy `2.2204460492503131e-9 m/rad`, below `2e-8` tolerance |
| Difficult actual Panda configuration | Deterministic 2048-configuration SVD scan found condition number **271.605**, singular values **1.0174591, 0.8315901, 0.00374610**; central-difference discrepancy `2.8072e-10`; nearby FK-proven target converged in five updates |
| Exact rank-zero test fixture | All position-Jacobian entries zero; DLS stayed finite and state unchanged, repeated 100 stagnant updates, then exhausted budget |
| Index mapping | Standard seven arm mappings verified; test-only Panda with an unrelated free-joint sibling shifts qpos by 7 and DOFs by 6; baseline still converged in five updates |
| Gripper/non-arm safety | Non-arm qpos unchanged, including asymmetric gripper coordinates and unrelated free-body qpos in the shifted fixture |
| Atomic rejection | No partial update when a late arm proposal violates a tight enabled limit, or when rejection follows an earlier accepted update |
| Disabled joint limits | Deliberately unusable stored ranges were ignored when `jnt_limited` was false; baseline converged |
| Clearly unreachable target | `(10,10,10) m` rejected; sum of root-to-hand translation norms is only **1.3192623 m**, a conservative geometric upper bound |
| Constraint-only unreachable target | Test-local model locks every arm range to home; its position workspace is a singleton, so home +20 mm X is provably excluded and rejected |

The poorly conditioned Panda start is:

```text
q = [-2.4427293307672846, -1.6852958025744003, 2.7287551538828709,
     -0.48462995868177261, 0.28392809436633248, 2.6797512575100875, 1.2910989730146292]
target = [0.67930003125645255, 0.44257459885577921, 0.053326685656516604]
```

Perturbation in the test changes joint2 toward the middle of its range by 0.02 rad to construct the reachable target. The near-straight Panda example is measured separately and is not claimed to be exactly singular. The exact singular fixture is synthetic and explicitly identified as such.

## Deterministic stress and benchmarks

`std::mt19937` and an explicit integer-to-unit-interval conversion produce the sample sequence; `uniform_real_distribution` is avoided. Seven arm angles are sampled inside **5% margins of actual model ranges**. Reachable target positions are produced with MuJoCo FK from the recorded witness joint configurations. Even-numbered reachable IDs start from home; odd-numbered IDs start from a second valid random configuration. Arbitrary Cartesian points are sampled independently in **[-1.25,1.25)^3 m** from valid random starts, without a reachability assertion.

| Metric | FK-proven reachable | Arbitrary Cartesian |
| --- | ---: | ---: |
| Cases | 120 | 60 |
| Converged | **63 (52.5%)** | **3 (5.0%)** |
| Joint-limit termination | 57 (47.5%) | 56 (93.3%) |
| Iteration-limit termination | 0 | 1 |
| Updates, median / p95, all outcomes | 9 / 13 | 0 / 11 |
| Updates, median / p95, successes only | 11 / 15 | 12 / 12 |
| Solver runtime, median / p95, microseconds | 235.5 / 566.3 | 26.9 / 317.6 |
| Final error, median / p95, metres, all outcomes | 0.000968988 / 1.215509 | 1.270967 / 2.199338 |
| Cases with an error-increasing accepted update | 0 | 6 (10%) |
| Cases with stagnant accepted updates | 0 | 0 |
| Cases ending worse than their initial error | 0 | 0 |

Reachable targets from home: **50/60 (83.3%)**. Reachable targets from random valid starts: **13/60 (21.7%)**. These are outcomes for these exact sample conditions, not universal solver success probabilities. Known reachable failure IDs (zero-based) are:

```text
1,2,3,5,6,7,9,13,15,17,19,21,23,27,28,29,31,33,35,37,39,40,43,45,47,48,
51,53,55,56,57,59,61,62,65,67,68,69,71,75,76,77,81,83,85,91,93,94,95,101,
103,105,109,111,115,117,119
```

All failed arbitrary cases are also individually recorded; their failure is **not** evidence of unreachability. Witness validity means joint limits and FK reachability only: collision-free reachability, dynamics, self-collision, velocity/torque limits, orientation and path feasibility are outside this position-only suite.

Timing uses `steady_clock` only around `solve_panda_ik`. Model loading, reset/target FK, independent post-solve FK verification, output and CSV writes are excluded. Solver-side allocations, name lookups and trace collection are included. This is one **Debug** machine snapshot, not a performance promise; no timing threshold fails tests. Percentiles use nearest rank. A low median iteration count/runtime can reflect immediate rejection rather than efficient convergence.

Operational metrics: an increasing step means `after > before + 1e-12 m`; stagnation means an accepted update outside tolerance with `abs(after-before) <= 1e-10 m`; net worse compares terminal error with initial error. Stagnation on rejected proposals is excluded. Individual increases/oscillation are distinguished from sustained divergence; no stress case had a worse terminal error than its start.

## Ranked findings and exact reproducers

All cases below have executable named reproducers. Home is
`[0,0,0,-1.57079,0,1.57079,-0.7853] rad`, gripper `[0.04,0.04] m`, hand position
`[0.55449947803173538,-2.6377536518327582e-33,0.62450242948758916] m`.

| Priority | Classification | Finding and impact | Evidence / next fix (not implemented) |
| --- | --- | --- | --- |
| **P1** | **Implementation defect** | Non-finite targets/state poison all seven arm coordinates and consume 100 updates. Range validation does not prevent NaN commits. | `nan_x/y/z`, `inf_x/y/z`, `neginf_x/y/z`, `nonfinite_state`. Validate target/state/Jacobian/linear-solve output/proposals with finite checks before committing, and return a distinct invalid/numerical status. |
| **P2** | **Constraint-handling issue** | Valid FK-proven positions fail on the first invalid proposal; 57/120 reachable cases terminate this way. Atomicity works, but there is no feasible recovery. | `reachable_joint_limit`, witness below. Joint-limit-aware recovery should be evaluated as a separate change; any step adjustment or constrained solve needs explicit future design/validation. |
| **P2** | **Implementation defect** | Success bypasses model-joint and starting-limit validation when already within tolerance. A malformed arm or out-of-limit joint4 is reported as converged. | `missing_joint_at_target`, `invalid_state_limits`. Validate all seven mappings and starting enabled limits before the convergence return. |
| **P2** | **Implementation defect** | Reaching tolerance on the last allowed update returns `success=false`, `iteration_limit`. | `last_update`, five-update baseline budget. Add terminal convergence reconciliation. The original loop had no final reached check; the new result exposes that control-flow gap. No default-100 boundary occurrence was observed in the stress sample. |
| **P3** | **Numerical/algorithmic limitation** | Fixed-step DLS allows error increases and oscillation; it spends the full budget on rank-zero stagnation. | `displacements` +400 mm and arbitrary stress case 18 below; `singular` fixture. Consider progress/stagnation/divergence diagnostics first; separately assess future damping/step control. No such changes were made here. |
| Informational | **Expected unreachable-target behavior** | Targets excluded by a geometric bound or locked-arm constraints fail safely. | `unreachable`, `constrained_target`. These are the only cases classified unreachable by proof, rather than inferred from solver failure. |

No test infrastructure issue blocked execution. The current fixed-base Panda's gripper columns in the hand-body Jacobian are zero, so the retained full-`nv` DLS matrix is harmless in this model. If additional moving ancestor DOFs are later introduced, computing over them while discarding their updates is a potential integration defect; this suite does not claim that hypothetical variant was tested or broken.

### Non-finite corruption

Start at home and replace one coordinate of the home hand position with NaN, +infinity or -infinity. After execution all seven arm qpos and the reported final error are NaN; gripper qpos remain unchanged. A NaN `joint1` state with the baseline finite target also propagates to every arm joint. The confirmed control-flow cause is that comparisons `new_pos < lower` and `new_pos > upper` are both false for NaN, allowing the staged invalid proposal to commit. Missing early finite checks then permit all 100 updates. There is no infinite retry, but the reported reason is misleadingly `iteration_limit`.

### Reachable target rejected by constraints

`test_panda_ik reachable_joint_limit` starts at home and uses:

```text
target = [-0.062143887675588504, -0.18024202184315868, 0.068261256651855715] m
valid FK witness q = [-0.60040731942799397, 1.1038278355282545, -0.21411064918577694,
                      -2.7778767300428822, -1.3713232638383797, 1.2919959273124113,
                      -1.2970355714879649] rad
final q = [-0.079460916491924397, -0.60534305006200961, -0.079460916491924397,
           -2.705128234770755, -0.015333400015348933, 0.73950710000594877, -0.7853]
updates = 1; error = 0.43939149060182753 m; reason = joint_limit
```

The target is demonstrably reachable within this model's joint ranges. The solver is local and has no completeness guarantee; this strict red test expresses a desired recovery improvement, rather than claiming all valid starts must mathematically converge. The trace proves a proposed step was rejected; it does not establish which alternative path or optimizer would succeed.

### Convergence validation gaps

- `missing_joint_at_target`: rename `joint7` to `Xoint7` in a test-local model; use the home hand position as target. Result is `success=true`, `converged`, zero updates; the same missing joint with a displaced target is detected and rejected atomically.
- `invalid_state_limits`: start with home except `joint4=0 rad`, exceeding its enabled upper limit `-0.0698 rad`; target is its FK position `[0.10700055675580929,-5.9396931817445874e-18,1.1209993230311845] m`. Result is `success=true`, error zero, zero updates.
- `last_update`: home, target `[0.5744994780317354,-2.6377536518327582e-33,0.62450242948758916] m`, explicit budget five. Final error is `0.00077301406450712537 m`, but result is `success=false`, `iteration_limit`. With unchanged default budget 100, it succeeds after the same five updates.

### Error growth, oscillation and exhaustion

`displacements`: home, target `[0.9544994780317354,-2.6377536518327582e-33,0.62450242948758916] m` (+400 mm X). It ends at `0.15677603230875892 m` after 100 updates, with **49 accepted error increases**. Early errors are `0.400000 → 0.232345 → 0.155674 → 0.149482 → 0.154418 → 0.148520 → 0.154764...`. This is observed oscillation; target reachability is not proven either way.

`stress` arbitrary case **18** uses:

```text
initial q = [-0.34430193528378839, 1.2544887473046407, 1.5020308624417011,
             -0.39602545731835948, -0.69574122345297562, 1.9568663273721467,
             0.88344014079703026]
target = [0.94250145310070366, -0.040068713715299964, 0.16260479809716344] m
final q = [-0.16539101505574416, 1.7335776175168522, 1.4945124503119074,
           -0.2828790530359338, -0.0017861250811985196, 2.7934117768738833,
           0.88344014079694833]
error = 0.10659978013379803 m; updates = 100; increasing steps = 47
```

All seven arm states stayed finite and within limits. No reachability conclusion follows. The exact rank-zero fixture instead proves finite damping and bounded stagnation on an intentionally position-immobile chain, not the existence of a singular reachable Panda failure.

## Recommended order for follow-up work

1. Reject non-finite input/state and numerical output before any commit; return explicit reasons.
2. Validate required joint mappings/types and enabled starting limits before a success return.
3. Reconcile convergence after the terminal update; retain the five-update boundary regression.
4. Add informative progress/numerical failure diagnostics; retain raw error traces and atomic commits.
5. Design and benchmark a joint-constraint recovery strategy against the exact FK witness corpus, reporting home/random-start rates separately.

These recommendations were **not implemented**. The existing custom DLS method, constants, iteration budget and model remain the subject of the retained tests.
