import argparse
import importlib.metadata as metadata
import json
from pathlib import Path
import platform
import sys

import numpy as np


def mink_probe(root):
    import mink
    import mujoco
    spec = mujoco.MjSpec.from_file(str(root / "simulation/models/franka_emika_panda/scene.xml"))
    spec.body("hand").add_site(name="probe_tip", pos=[0, 0, .12], size=[.002])
    model = spec.compile()
    live = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, live, model.key("home").id)
    mujoco.mj_forward(model, live)
    before = live.qpos.copy()
    configuration = mink.Configuration(model, q=before.copy())
    transform = configuration.get_transform_frame_to_world("probe_tip", "site").as_matrix()
    transform[:3, 3] += [.01, .005, 0]
    angle = .02
    rz = np.array([[np.cos(angle), -np.sin(angle), 0],
                   [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    transform[:3, :3] = rz @ transform[:3, :3]
    task = mink.FrameTask("probe_tip", "site", position_cost=1, orientation_cost=1,
                          gain=.5, lm_damping=1e-4)
    task.set_target(mink.SE3.from_matrix(transform))
    freeze = mink.DofFreezingTask(model, [int(model.jnt_dofadr[model.joint(n).id])
                                       for n in ("finger_joint1", "finger_joint2")])
    for i in range(100):
        error = task.compute_error(configuration)
        if np.linalg.norm(error[:3]) < 1e-6 and np.linalg.norm(error[3:]) < 1e-6:
            break
        velocity = mink.solve_ik(configuration, [task], dt=.01, solver="daqp", safety_break=True,
                                 damping=1e-6, constraints=[freeze])
        configuration.integrate_inplace(velocity, .01)
    actual = configuration.get_transform_frame_to_world("probe_tip", "site").as_matrix()
    pos_error = float(np.linalg.norm(actual[:3, 3] - transform[:3, 3]))
    rotation_error = float(np.linalg.norm(mink.SO3.from_matrix(transform[:3, :3].T @ actual[:3, :3]).log()))
    if pos_error >= 1e-6 or rotation_error >= 1e-6 or not np.array_equal(before, live.qpos):
        raise RuntimeError("Pose IK or independent planning state probe failed")
    return {"status": "passed_local_pose_probe", "iterations": i,
            "position_error_m": pos_error, "orientation_error_rad": rotation_error,
            "live_state_unchanged": bool(np.array_equal(before, live.qpos)),
            "max_joint_change_rad": float(np.max(np.abs(configuration.q[:7] - before[:7]))),
            "limitations": "Local probe only; no path feasibility or recovery certification"}


def toppra_probe():
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint
    path = ta.SplineInterpolator([0, .5, 1], [[0, 0], [.1, .05], [.2, 0]])
    timing = algo.TOPPRA([constraint.JointVelocityConstraint([.5, .5]),
                         constraint.JointAccelerationConstraint([1., 1.])], path,
                        solver_wrapper="seidel", parametrizer="ParametrizeConstAccel",
                        gridpoints=np.linspace(0, 1, 1001)).compute_trajectory(0, 0)
    if timing is None or timing.duration <= 0 or not np.isfinite(timing.duration):
        raise RuntimeError("TOPPRA returned no valid trajectory")
    t = np.linspace(0, timing.duration, 10001)
    v, a = float(np.abs(timing(t, 1)).max()), float(np.abs(timing(t, 2)).max())
    if v > .5001 or a > 1.001:
        raise RuntimeError(f"TOPPRA output violates probe limits: v={v}, a={a}")
    return {"status": "passed", "parametrizer": "ParametrizeConstAccel", "duration_s": timing.duration,
            "velocity_max": v, "acceleration_max": a,
            "rejected_default": "ParametrizeSpline exceeded 1 rad/s^2 probe limit (1.1303218251)"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/dependency-probe.json"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    report = {"python": sys.version, "platform": platform.platform(), "packages": {}, "probes": {}}
    for name in ("ezdxf", "mujoco", "numpy", "scipy", "mink", "qpsolvers", "daqp", "toppra"):
        try:
            dist = metadata.distribution(name)
            report["packages"][name] = {"version": dist.version,
                "license": dist.metadata.get("License-Expression") or dist.metadata.get("License"),
                "license_classifiers": [c for c in dist.metadata.get_all("Classifier", []) if "License" in c]}
        except metadata.PackageNotFoundError:
            report["packages"][name] = {"status": "missing"}
    for name, probe in (("mink", lambda: mink_probe(root)), ("toppra", toppra_probe)):
        try:
            report["probes"][name] = probe()
        except Exception as e:
            report["probes"][name] = {"status": "blocked", "error": f"{type(e).__name__}: {e}"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report["probes"], indent=2))
    return 2 if any(v["status"] == "blocked" for v in report["probes"].values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
