# Project HAND - Human Assisted Neural Dexterity

A work-in-progress robotic manipulation project.

The goal is to build a system that can perceive objects, plan motions, and control a robotic arm in simulation, eventually moving toward autonomous manipulation.

### Current progress
- 2D forward and inverse kinematics in C++
- Basic kinematics tests
- MuJoCo simulation with a 7-DOF robotic arm
- Initial joint control and tracking tests

### Next
- Connect C++ kinematics to MuJoCo
- Move to 3D kinematics and motion planning
- Add perception and object interaction

Still early in development. Mostly working on getting the fundamentals and simulation running.

### Build and test

Run from the `HAND` directory with CMake 3.20+, Ninja, and a C++17 compiler
on your PATH (on this Windows setup, use MSYS2 UCRT64).
Eigen 5.0.0 is vendored in `third_party/eigen` and used as a header-only
`Eigen3::Eigen` target; no separate Eigen install is needed.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DHAND_BUILD_MUJOCO=OFF
cmake --build build
ctest --test-dir build --output-on-failure
./build/hand_demo.exe
```

For the native Panda example, point CMake at an extracted Windows MuJoCo
distribution containing `include`, `lib`, and `bin`:

```powershell
cmake -S . -B build -G Ninja -DHAND_BUILD_MUJOCO=ON -DMUJOCO_DIR="C:/path/to/mujoco"
cmake --build build
./build/hand_panda.exe
```

Run the Panda example from the `HAND` directory so it can find
`simulation/models/franka_emika_panda/scene.xml`. CMake copies `mujoco.dll`
beside the executable. MuJoCo is optional and disabled for a fresh configure.
Generated binaries and CMake files belong in `build/`; remove that folder
when you need a completely clean configure.

With `HAND_BUILD_MUJOCO=ON` and `BUILD_TESTING=ON`, the build also produces
`test_panda_ik`. Run `ctest --test-dir build --output-on-failure` for the
separate 2D and Panda suites, or `./build/test_panda_ik.exe stress` for the
fixed-seed stress/benchmark corpus. CSV records, per-update traces and test-only
fixtures are written to `build/ik_validation/`; model loading/setup is excluded
from solver timing. Panda tests can run from any working directory.

The adversarial suite deliberately retains failing regression checks, labeled
`known_defect`, for discovered solver weaknesses. See
[the validation report](docs/panda_ik_validation.md) for executed results,
exact reproducers, evidence paths and recommended follow-up changes.

The C++ modules separate planar algorithms (`hand_core`), the original iterative
DLS solver (`hand_ik`), and MuJoCo model/data ownership and keyframe setup
(`hand_mujoco`). The Panda app selects the model/target and reports results.
See [architecture and implementation preservation](docs/architecture_refactor.md)
for dependency direction and executed before/after comparisons. The complete
pre-extraction implementation is preserved in
[the original source archive](docs/panda_ik_original.cpp).
