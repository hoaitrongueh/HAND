import csv
import math
from pathlib import Path
import mink
import mujoco
import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint

HEADER = ["time", *[f"q{i}" for i in range(1, 8)], "x", "y", "z", "qw", "qx", "qy", "qz", "pen_down", "stroke"]


def slerp(a, b, t):
    a, b = np.array(a), np.array(b)
    if a @ b < 0: b = -b
    angle = math.acos(float(np.clip(a @ b, -1, 1)))
    q = (1-t)*a+t*b if angle < 1e-6 else (
        math.sin((1-t)*angle)*a+math.sin(t*angle)*b) / math.sin(angle)
    return q / np.linalg.norm(q)


def time_path(path, model, *, velocity=.2, acceleration=.4, settle_s=1., pen_dwell_s=.05):
    dt = model.opt.timestep
    if not all(math.isfinite(x) and x > 0 for x in (velocity, acceleration, settle_s, pen_dwell_s)):
        raise ValueError("Invalid timing options")

    constraints = [constraint.JointVelocityConstraint(np.full(7, .9*velocity)),
                   constraint.JointAccelerationConstraint(np.full(7, .9*acceleration))]
    if len(path) < 2 or path[0].pen_down or path[-1].pen_down:
        raise ValueError("Path must start and end pen up")
    rows = []
    max_v = max_a = 0.
    stamp = 0

    def append(q, p, orientation, down, stroke):
        nonlocal stamp
        rows.append([stamp*dt, *q, *p, *orientation, int(down), stroke])
        stamp += 1

    a = path[0]
    for _ in range(math.ceil(settle_s/dt)+1):
        append(a.joints, a.pose.position, a.pose.orientation, False, -1)
    for edge, (a, b) in enumerate(zip(path[:-1], path[1:])):
        q0, q1 = np.array(a.joints), np.array(b.joints)
        delta = q1-q0
        if np.max(np.abs(delta)) < 1e-10:
            for _ in range(max(1, math.ceil(pen_dwell_s/dt))):
                append(q1, b.pose.position, b.pose.orientation, b.pen_down, b.stroke)
            continue


        length = float(np.max(np.abs(delta)))
        curve = ta.SplineInterpolator([0, length], [q0, q1], bc_type="natural")
        timed = algo.TOPPRA(constraints, curve, solver_wrapper="seidel",
                           parametrizer="ParametrizeConstAccel", gridpoints=np.linspace(0, length, 101)).compute_trajectory(0, 0)
        if timed is None or not math.isfinite(timed.duration) or timed.duration <= 0:
            raise ValueError(f"TOPPRA failed: no finite trajectory at edge {edge}, joint length={length:.9g}")
        n = max(2, math.ceil(timed.duration/dt))
        stretch = n*dt/timed.duration

        check = np.linspace(0, timed.duration, max(1001, 2*n))
        v, acc = np.abs(timed(check, 1))/stretch, np.abs(timed(check, 2))/stretch**2
        max_v, max_a = max(max_v, float(v.max())), max(max_a, float(acc.max()))
        if not np.isfinite(v).all() or not np.isfinite(acc).all() or v.max() > velocity or acc.max() > acceleration:
            raise ValueError("TOPPRA output exceeds configured limits")
        index = int(np.argmax(np.abs(delta)))
        for k, q in enumerate(timed(np.arange(1, n+1)*dt/stretch), 1):
            u = float(np.clip((q[index]-q0[index])/delta[index], 0, 1))
            position = np.array(a.pose.position) + u*(np.array(b.pose.position)-a.pose.position)
            append(q, position, slerp(a.pose.orientation, b.pose.orientation, u), b.pen_down, b.stroke)
    rows = np.asarray(rows)

    joints = [model.joint(f"joint{i}").id for i in range(1, 8)]
    bounds = model.jnt_range[joints]
    q = rows[:, 1:8]
    if (not np.isfinite(rows).all() or np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]) or
            np.any(np.diff(rows[:, 0]) <= 0)):
        raise ValueError("Timed path has invalid state or joint limits")
    discrete_v = np.diff(q, axis=0)/dt
    discrete_a = np.diff(discrete_v, axis=0)/dt
    if abs(discrete_v).max() > velocity or abs(discrete_a).max() > acceleration:
        raise ValueError("Sampled trajectory violates derivative limits")

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    qadr = [model.jnt_qposadr[j] for j in joints]
    site = model.site("pen_tip").id
    max_position = max_orientation = 0.
    for row in rows:
        data.qpos[qadr] = row[1:8]
        mujoco.mj_forward(model, data)
        pos = float(np.linalg.norm(data.site_xpos[site]-row[8:11]))
        rot = mink.SO3.from_matrix(data.site_xmat[site].reshape(3, 3)).inverse() @ mink.SO3(row[11:15])
        ori = float(np.linalg.norm(rot.log()))
        max_position, max_orientation = max(max_position, pos), max(max_orientation, ori)
        if pos > 1e-4 or ori > 1e-3 or any(c.dist < -1e-5 for c in data.contact):
            raise ValueError("Timed edge fails Cartesian pose or collision validation")
    return rows, {"backend": "TOPPRA 0.6.3 / seidel / ParametrizeConstAccel",
                  "duration_s": float(rows[-1, 0]), "samples": len(rows),
                  "max_velocity_rad_s": max_v, "max_acceleration_rad_s2": max_a,
                  "max_sampled_velocity_rad_s": float(abs(discrete_v).max()),
                  "max_sampled_acceleration_rad_s2": float(abs(discrete_a).max()),
                  "max_joint_increment_rad": float(abs(np.diff(q, axis=0)).max()),
                  "cartesian_interpolation_error_m": max_position,
                  "orientation_interpolation_error_rad": max_orientation}


def write_trajectory(path: Path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        f.write("# hand.trajectory.v1\n")
        writer = csv.writer(f)
        writer.writerow(HEADER)
        writer.writerows(rows)
