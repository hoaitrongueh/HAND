# Architecture refactor and implementation preservation

Validated locally on **2026-10-17, Asia/Saigon**. This refactor restores the original implementation's source structure while retaining its numerical behavior and all existing test expectations. No solver fixes from the [validation report](panda_ik_validation.md) were applied.

## Original implementation and provenance

Inspection covered the local source/build configuration, tests, documentation, Python simulation/controller/physics benchmark, model layout, dependency setup, placeholder directories, working-tree changes, and Git history for `apps/panda.cpp`.

- `dfd6aec` (current HEAD): Panda position/Jacobian demonstration; it does not contain the iterative solver.
- `7ad7a31`: earlier single-step DLS implementation, including `L`, `alpha`, the explicit `I`, named-joint address mapping and staged limit checks.
- `307030e`: a subsequent single-step version with a position-tolerance gate.
- The full **iterative** implementation was uncommitted before the previous extraction. Its saved working-tree source is `build/ik_validation/panda_before_extraction.cpp`. It was recovered from that snapshot rather than falsely attributed to a Git commit.

The complete iterative original is now preserved byte-for-byte in **`docs/panda_ik_original.cpp`**, outside the disposable build directory. Its SHA-256 matches the original snapshot:

```text
7BDC8C81C56798027BCF65872DD05BC2DD7FAC13D79A76FEAA5316569F68E515
```

This file is an authorship/history reference, not an additional compiled solver. The active implementation remains **`src/ik.cpp`**.

### What the previous extraction changed, and what is restored

| Area | Previous extraction | This refactor |
| --- | --- | --- |
| Math | Retained DLS/LDLT but inlined the identity matrix and read constants through options | Restored `L`, `alpha`, explicit `I`, and `A = J * J.transpose() + L*L * I` |
| Position/error calculation | Replaced explicit reads with an error lambda and renamed the magnitude | Restored `position`, `current`, `error_vector`, `error_magnitude`, `updated_pos`, `updated`, `error` and explicit computations |
| Joint mapping/update | Renamed IDs/addresses/proposals and inlined range/limited reads | Restored `joint_name`, `joint_id`, `pos_index`, `dof`, `pos`, `new_pos`, `lower`, `upper`, `limited`, `proposed_angles`, `pos_addresses` |
| Control flow | Returned immediately on convergence/rejection; inverted the commit branch | Restored the pre-update convergence `break`, positive `if (valid_step)` commit block, and rejection `else`/`break` |
| Application | Rewrote setup with local smart pointers and renamed target setup | Restored `path`, `model`, `data`, `hand_id`, `initial`, `ini_pos`, and the original target expression; moved resource ownership/reset to the integration module |
| Numerical behavior | Defaults, formula, full-`nv` Jacobian, seven-joint update and validation order were retained | Preserved again; exact pre/post comparison described below |

The matrix, solve, update, joint lookup, limit-check and commit blocks are relocated from the original, with their naming and expressions restored. The earlier rewrites were unnecessary for numerical extraction, even though the observed numerical behavior was unchanged.

Remaining differences from the original monolithic application are explicit:

1. The solver is a reusable function in namespace `hand`, accepting borrowed model/data and a caller-supplied target. The existing result/trace interface remains around the original decisions; these observation assignments do not choose steps.
2. `L`, `alpha`, and `tolerance` bind the existing `IkOptions` fields, rather than being hard-coded `constexpr` values. The original **0.05, 0.5, 0.001 m and 100** defaults are unchanged. Explicit shortened-budget tests remain supported.
3. Null model/data/body and missing-body guards introduced at the previous callable boundary remain. Missing-joint handling returns a status and leaves cleanup to the owner instead of destroying caller-owned resources and exiting the program.
4. Verbose matrix/joint/position prints remain outside the algorithm; the app prints the existing concise result trace. Restoring unconditional console output inside a reusable solver would affect test logs and benchmark measurements. The complete original diagnostics remain available in the archived source.
5. Model/data lifetime is managed by the integration owner, whose setup errors reach the app as exceptions and become exit code 1. Successful setup, FK and IK numerics are unchanged. The existing demo's zero exit status after solver failure is retained.

No damping/step adaptation, line search, null-space method, constraint recovery, finite checks, terminal-convergence fix or other algorithm improvement was implemented. Direct MuJoCo FK/Jacobian calls remain in the solver; removing that dependency would require a larger backend abstraction without a current engineering need.

## Architecture and dependency direction

```text
HAND/
  include/hand/
    kinematics.hpp          # Existing 2D algorithms
    ik.hpp                  # Existing position IK options/results/interface
    mujoco.hpp              # Model/data owner and keyframe setup
  src/
    kinematics.cpp          # Existing planar FK/reachability/analytical IK
    ik.cpp                  # Original iterative DLS loop, result observations
    mujoco.cpp              # MuJoCo allocation, cleanup, keyframe reset + FK
  apps/
    main.cpp                # Existing planar demonstration
    panda.cpp               # Model choice, target choice, solver call, output
  tests/
    test_kinematics.cpp      # Existing 2D unit checks
    test_panda_ik.cpp        # Integration, numerical regressions, seeded stress
  docs/
    conventions.md
    panda_ik_original.cpp    # Exact historical source, not a build input
    panda_ik_validation.md   # Preserved prior defect/evidence report
    architecture_refactor.md
  third_party/eigen/         # Existing vendored headers, unchanged
  simulation/               # Existing Python simulation/controller + MJCF assets
  benchmarks/               # Existing Python physics benchmark
  build/                    # All generated build/test evidence
  CMakeLists.txt
```

Existing `assets`, `configs`, `data`, `outputs`, `perception`, and `scripts` remain in place; no speculative directory hierarchy was added.

```text
hand_demo       -> hand_core -> Eigen3::Eigen
test_kinematics -> hand_core

hand_panda      -> hand_ik     -> Eigen3::Eigen, mujoco_native
                -> hand_mujoco -> mujoco_native
test_panda_ik   -> hand_ik
```

`hand_ik` no longer links to the unrelated planar `hand_core` merely to inherit include directories, C++17 or Eigen. Both optional libraries now declare their direct requirements. MuJoCo remains behind `HAND_BUILD_MUJOCO`; the existing imported Windows library/DLL conventions and CTest registration remain intact.

The only new runtime abstraction is **`MujocoSimulation`**, a concrete owner of one matching model/data pair. Its reason is resource lifetime and reusable state setup: data must be destroyed before its model, partial initialization must clean up, and the app should not implement those responsibilities. It owns no IK policy and introduces no backend interface, inheritance, joint cache, generic framework or alternative FK implementation. The solver borrows raw handles; tests retain their existing independent fixture ownership and expectations.

Changes in this task: `src/ik.cpp`, `apps/panda.cpp`, `CMakeLists.txt`, and this README/report documentation; new `include/hand/mujoco.hpp`, `src/mujoco.cpp`, and the archived original. `include/hand/ik.hpp`, both test files, 2D source, existing validation report, models, dependencies, Python programs and unrelated uncommitted editor changes were preserved.

## Executed validation

Environment remains Windows/MSYS2 UCRT64, GCC 16.2.0, CMake 4.4.3, Ninja 1.13.2, Debug, Eigen 5.0.0, MuJoCo 3.16.0. A fresh pre-refactor run established the baseline before editing; after the refactor the main project was configured, fully built, and executed:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DHAND_BUILD_MUJOCO=ON -DMUJOCO_DIR="C:/Users/lethu/Downloads/mujoco-3.16.0-windows-x86_64"
cmake --build build
./build/hand_panda.exe
ctest --test-dir build --output-on-failure

cmake -S . -B build/core_only -G Ninja -DCMAKE_BUILD_TYPE=Debug -DHAND_BUILD_MUJOCO=OFF
cmake --build build/core_only
ctest --test-dir build/core_only --output-on-failure
```

| Check | Before | After |
| --- | --- | --- |
| Main build | Existing working build | All targets built successfully, no compiler diagnostics |
| Panda application | Success | Exit 0; identical complete output |
| Main CTest | **31 executed, 17 passed, 14 failed**, exit 8 | **31 executed, 17 passed, same 14 failed**, exit 8 |
| MuJoCo-disabled configure/build/CTest | Previously supported | Executed successfully, original 2D test **1/1 passed** |
| Home +20 mm X | Five updates, final error **0.00077301406450712537 m** | Identical error trace, final qpos and count |
| All scenario CSV results | 247 solver records | All 247 identical excluding runtime |
| All proposal traces | 2,419 rows | All 2,419 identical |
| Seeded stress | 180 cases, seed **1212239428** | All initial/target/witness/final coordinates, errors, counts and statuses identical |
| Test source/expectation hashes | Pre-refactor manifest | Unchanged |

The 120 FK-proven reachable stress targets still produce **63 convergences / 57 joint-limit terminations** (home starts 50/60; random starts 13/60). The 60 arbitrary Cartesian cases still produce **3 convergences / 56 joint-limit terminations / 1 iteration exhaustion**. Runtime is measured but excluded from numerical equivalence; no timing equality or cross-platform bitwise claim is made. Full post-refactor CTest took 9.24 s on this run.

**No new regression or numerical difference was observed in the exercised corpus.** The known failing set remains `missing_joint_at_target`, `last_update`, `nonfinite_state`, `invalid_state_limits`, `reachable_joint_limit`, and NaN/+infinity/-infinity targets on all three axes. Their causes and exact reproducers remain in the prior validation report: non-finite state corruption, pre-convergence validation gaps, final-update status inconsistency, constraint recovery limitations, and error-growth/stagnation limitations. They remain visible without changing expectations or inverting test outcomes.

Evidence is preserved under **`build/architecture_refactor/`**:

- `before/`: original validation evidence, freshly rerun pre-refactor results, source/config snapshots, Git source checkpoints, working-tree diff/status and a SHA-256 manifest.
- `configure.log`, `build.log`, `panda.txt`, `panda_exit_code.txt`, `ctest.log`, `ctest_exit_code.txt`, `LastTest.log`, `after_validation/`: post-refactor build/application/test evidence, including individual failures and traces.
- `core_configure.log`, `core_build.log`, `core_ctest.log`: optional-dependency-off verification.
- `comparison.txt`: exact comparison of all scenario/trace CSVs excluding `solver_us`, identical failing test set, and identical app output.

The original `build/ik_validation/` evidence was copied before test reruns could overwrite per-case files. The historical validation report and original source snapshot were not overwritten. SHA-256 comparison against the starting manifest confirms that unrelated tracked files and pre-existing test/report files remain unchanged. No reset, checkout, commit, model/dependency modification or unrelated cleanup was performed.
