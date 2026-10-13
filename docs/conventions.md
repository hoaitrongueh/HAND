# Initial engineering conventions

- Use the `hand` namespace for reusable C++ code.
- Put public declarations in `include/hand/` and implementations in `src/`.
- Keep executable entry points in `apps/` small; reusable logic belongs in modules.
- Use SI units internally: metres, seconds, and radians. Convert at input/output boundaries.
- Name coordinate frames and state the direction of each transform explicitly.
- Record dependencies and setup steps when introducing them.
- Add correctness checks with numerical modules and reproducible scenarios with system milestones.
- Keep input configurations separate from generated output.
- Document failure cases and measured limitations alongside successful results.

The first planar arm exercise uses a base at the origin, x to the right, y up,
and positive joint rotation counterclockwise. The second joint angle is relative
to the first link. These are exercise conventions; 3D frame conventions will be
documented when the robot model is selected.
