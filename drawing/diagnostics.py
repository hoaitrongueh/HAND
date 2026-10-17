import csv
from pathlib import Path
import mink
import numpy as np
from .geometry import resample
from .planning import quaternion


def analyse(drawing, config, trace_path: Path, output: Path, tolerance=.001):
    with trace_path.open() as f:
        records = list(csv.DictReader(f))
    if not records:
        raise ValueError("Empty executed trace")
    columns = {key: np.array([float(row[key]) for row in records]) for key in records[0]}
    xyz = np.column_stack([columns[c] for c in ("x", "y", "z")])
    rotation = mink.SO3(quaternion(config.get("cad_rotation_wxyz", [1, 0, 0, 0]))).as_matrix()
    translation = np.array(config["cad_origin_m"])
    marking = columns["pen_down"] == 1
    ids = columns["stroke"].astype(int)
    total = covered = 0.
    per_stroke = []
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    for i, stroke in enumerate(drawing.strokes):
        planned = np.asarray(stroke.points)*drawing.metres_per_unit @ rotation.T + translation
        points = resample(planned, tolerance)
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        mids = (points[:-1]+points[1:])/2
        valid = marking[:-1] & marking[1:] & (ids[:-1] == i) & (ids[1:] == i)
        starts, ends = xyz[:-1][valid], xyz[1:][valid]
        distances = []
        if len(starts):
            v = ends-starts
            squared = np.sum(v*v, axis=1)
            for p in mids:
                u = np.clip(np.sum((p-starts)*v, axis=1)/np.maximum(squared, 1e-30), 0, 1)
                distances.append(float(np.linalg.norm(p-starts-u[:, None]*v, axis=1).min()))
        else:
            distances = [float("inf")]*len(mids)
        length = float(lengths.sum())
        done = float(lengths[np.array(distances) <= tolerance].sum())
        total += length; covered += done
        per_stroke.append({"stroke": i, "length_m": length, "covered_length_m": done,
                           "coverage": done/length if length else 0})
        cad = (planned-translation) @ rotation
        ax.plot(cad[:, 0]*1000, cad[:, 1]*1000, "--", color="gray", linewidth=2,
                label="Planned CAD" if i == 0 else None)
        actual = (xyz-translation) @ rotation

        x = np.where(marking & (ids == i), actual[:, 0]*1000, np.nan)
        y = np.where(marking & (ids == i), actual[:, 1]*1000, np.nan)
        ax.plot(x, y, color="#0569b9", linewidth=1, label="Executed pen trace" if i == 0 else None)
    ax.set_aspect("equal"); ax.set_xlabel("CAD plane X (mm)"); ax.set_ylabel("CAD plane Y (mm)")
    ax.set_title(f"Panda actuator drawing — {100*covered/total:.2f}% coverage")
    ax.legend(); ax.grid(alpha=.2); fig.tight_layout()
    fig.savefig(output / "traces.png", dpi=160); fig.savefig(output / "traces.svg"); plt.close(fig)
    actual_joints = np.column_stack([columns[f"q{i}"] for i in range(1, 8)])
    desired = np.column_stack([columns[f"desired_q{i}"] for i in range(1, 8)])
    return {"coverage_tolerance_m": tolerance, "coverage_method": "CAD chord arc-length midpoint samples <= tolerance spacing, nearest actual pen-on segment of same stroke",
            "drawing_coverage": covered/total, "requested_length_m": total, "covered_length_m": covered,
            "strokes": per_stroke, "pen_requested_samples": int(columns["pen_requested"].sum()),
            "pen_marking_samples": int(marking.sum()),
            "joint_tracking_rms_rad": float(np.sqrt(np.mean((actual_joints-desired)**2))),
            "cartesian_tracking_rms_m": float(np.sqrt(np.mean(columns["cartesian_error"]**2))),
            "executed_joint_motion_rad": float(np.max(np.ptp(actual_joints, axis=0))),
            "actuator_force_max_nm": float(max(np.abs(columns[f"force{i}"]).max() for i in range(1, 8)))}
