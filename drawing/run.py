import argparse
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import subprocess
import time
import mujoco
import mink
import numpy as np
from .geometry import load_dxf
from .planning import ROOT, plan_drawing, path_dict, quaternion
from .timing import time_path, write_trajectory
from .diagnostics import analyse


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="new directory, prevents stale evidence")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/drawing.json")
    parser.add_argument("--executor", type=Path, default=ROOT / "build/hand_draw.exe")
    parser.add_argument("--velocity", type=float, default=.2)
    parser.add_argument("--acceleration", type=float, default=.4)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report = {"schema": "hand.run.v1", "status": "started", "stage": "geometry", "failures": [],
              "input": str(args.input.resolve()), "config": str(args.config.resolve()),
              "packages": {n: metadata.version(n) for n in ("ezdxf", "mink", "toppra", "mujoco", "numpy", "scipy", "qpsolvers", "daqp")}}
    try:
        config = json.loads(args.config.read_text())
        report["configuration"] = config
        report["input_sha256"] = hashlib.sha256(args.input.read_bytes()).hexdigest()
        d = load_dxf(args.input)
        save(args.output / "strokes.json", d.to_dict())
        report["geometry"] = {"strokes": len(d.strokes), "samples": sum(len(s.points) for s in d.strokes),
                              "approximation_bound_m": d.approximation_bound_cad*d.metres_per_unit,
                              "measured_approximation_error_m": max(s.approximation_error_cad for s in d.strokes)*d.metres_per_unit}
        report["stage"] = "pose_ik"
        model, path = plan_drawing(d, config)
        save(args.output / "joint_path.json", path_dict(path))
        q = np.array([p.joints for p in path])
        report["ik"] = {"waypoints": len(path), "position_residual_max_m": max(p.position_residual_m for p in path),
                        "orientation_residual_max_rad": max(p.orientation_residual_rad for p in path),
                        "max_joint_waypoint_step_rad": float(abs(np.diff(q, axis=0)).max()),
                        "recovery": "bounded retries; no skip; halt before execution on failure"}
        report["stage"] = "timing"
        rows, report["timing"] = time_path(path, model, velocity=args.velocity, acceleration=args.acceleration)
        write_trajectory(args.output / "planned.csv", rows)
        mujoco.mj_saveModel(model, str(args.output / "model.mjb"))
        if args.plan_only:
            report["status"] = "planned_only"
        else:
            report["stage"] = "execution"
            command = [str(args.executor.resolve()), str((args.output / "model.mjb").resolve()),
                       str((args.output / "planned.csv").resolve()), str(args.output.resolve()),
                       str(args.velocity), str(args.acceleration)]
            normal = mink.SO3(quaternion(config.get("cad_rotation_wxyz", [1, 0, 0, 0]))).as_matrix()[:, 2]
            command += [str(x) for x in [*config["cad_origin_m"], *normal]]
            report["execution_command"] = command
            execution_start = time.perf_counter()
            process = subprocess.run(command, capture_output=True, text=True, timeout=300)
            report["execution_wall_s"] = time.perf_counter()-execution_start
            (args.output / "executor.log").write_text(process.stdout+process.stderr, encoding="utf-8")
            if (args.output / "execution.json").exists():
                report["execution"] = json.loads((args.output / "execution.json").read_text())
            if (args.output / "executed.csv").exists():
                report["diagnostics"] = analyse(d, config, args.output / "executed.csv", args.output)
            if process.returncode:
                raise ValueError(f"C++ executor exited {process.returncode}: {process.stderr.strip()}")
            if report["diagnostics"]["drawing_coverage"] < .99:
                raise ValueError("Acceptance failed: drawing coverage < 99% at 1 mm tolerance")
            if report["diagnostics"]["executed_joint_motion_rad"] < .01:
                raise ValueError("Acceptance failed: no meaningful actuator movement")
            report["status"] = "completed"
    except Exception as e:
        report["status"] = "failed"
        report["failures"].append({"stage": report["stage"], "error": f"{type(e).__name__}: {e}"})
    report["wall_time_s"] = time.perf_counter()-started
    save(args.output / "report.json", report)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] in ("completed", "planned_only") else 1


if __name__ == "__main__":
    raise SystemExit(main())
