from dataclasses import asdict, dataclass
from pathlib import Path
import math

import mink
import mujoco
import numpy as np
from .geometry import resample

ROOT = Path(__file__).resolve().parents[1]


def quaternion(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Invalid wxyz tool quaternion")
    return q / np.linalg.norm(q)


@dataclass
class ToolPose:
    position: list[float]
    orientation: list[float]

    def __post_init__(self):
        p = np.asarray(self.position, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError("Invalid tool position")
        self.position = p.tolist()
        self.orientation = quaternion(self.orientation).tolist()

    def matrix(self):
        return mink.SE3(np.r_[self.orientation, self.position]).as_matrix()


@dataclass
class JointWaypoint:
    joints: list[float]
    pose: ToolPose
    pen_down: bool
    stroke: int
    position_residual_m: float
    orientation_residual_rad: float


def drawing_model():

    spec = mujoco.MjSpec.from_file(str(ROOT / "simulation/models/franka_emika_panda/scene.xml"))
    hand = spec.body("hand")
    hand.add_site(name="pen_tip", pos=[0, 0, .12], size=[.002], rgba=[1, .1, .1, 1])
    hand.add_geom(name="pen_visual", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                  fromto=[0, 0, .06, 0, 0, .12], size=[.002], mass=0,
                  contype=0, conaffinity=0, rgba=[.1, .1, .1, 1])
    return spec.compile()


class PoseIK:
    def __init__(self, model, *, iterations=100, retries=3, position_tolerance=1e-5,
                 orientation_tolerance=1e-4, max_joint_step=.12):
        if iterations < 1 or not 1 <= retries <= 5 or not all(
            math.isfinite(x) and x > 0 for x in (position_tolerance, orientation_tolerance, max_joint_step)):
            raise ValueError("Invalid IK options")
        self.model = model
        self.configuration = mink.Configuration(model)
        self.configuration.update_from_keyframe("home")
        self.check_data = mujoco.MjData(model)
        self.iterations, self.retries = iterations, retries
        self.position_tolerance, self.orientation_tolerance = position_tolerance, orientation_tolerance
        self.max_joint_step = max_joint_step
        self.qadr = np.array([model.jnt_qposadr[model.joint(f"joint{i}").id] for i in range(1, 8)])
        self.jids = np.array([model.joint(f"joint{i}").id for i in range(1, 8)])
        self.freeze = mink.DofFreezingTask(model, [int(model.jnt_dofadr[model.joint(n).id])
                                                 for n in ("finger_joint1", "finger_joint2")])
        self.task = mink.FrameTask("pen_tip", "site", 1., 1., gain=.5, lm_damping=1e-4)

    def current_pose(self):
        frame = self.configuration.get_transform_frame_to_world("pen_tip", "site")
        return ToolPose(frame.translation().tolist(), frame.rotation().wxyz.tolist())

    def valid(self, q):
        if not np.isfinite(q).all():
            return False
        bounds = self.model.jnt_range[self.jids]
        if np.any(q[self.qadr] < bounds[:, 0]) or np.any(q[self.qadr] > bounds[:, 1]):
            return False
        self.check_data.qpos[:] = q
        mujoco.mj_forward(self.model, self.check_data)
        return all(c.dist >= -1e-5 for c in self.check_data.contact)

    def residual(self, target):
        actual = self.current_pose()
        pos = float(np.linalg.norm(np.array(actual.position) - target.position))
        rot = mink.SO3(np.array(actual.orientation)).inverse() @ mink.SO3(np.array(target.orientation))
        return pos, float(np.linalg.norm(rot.log()))

    def solve(self, target, *, pen_down=False, stroke=-1):
        initial = self.configuration.q.copy()
        if not self.valid(initial):
            raise ValueError("Invalid initial IK configuration (limits or collision)")
        self.task.set_target(mink.SE3(np.r_[target.orientation, target.position]))
        failures = []
        for retry in range(self.retries):
            self.configuration.update(initial)
            try:
                for iteration in range(self.iterations + 1):
                    pos, rot = self.residual(target)
                    if pos <= self.position_tolerance and rot <= self.orientation_tolerance:
                        q = self.configuration.q.copy()
                        if not self.valid(q):
                            raise ValueError("IK candidate violates limits or collision")
                        if np.max(np.abs(q[self.qadr] - initial[self.qadr])) > self.max_joint_step:
                            raise ValueError("IK joint discontinuity")
                        return JointWaypoint(q[self.qadr].tolist(), target, pen_down, stroke, pos, rot)
                    if iteration == self.iterations:
                        break
                    v = mink.solve_ik(self.configuration, [self.task], .01, "daqp", safety_break=True,
                                      damping=1e-6 * 10 ** retry, constraints=[self.freeze])
                    if not np.isfinite(v).all():
                        raise ValueError("Nonfinite IK update")
                    self.configuration.integrate_inplace(v, .01)
                failures.append(f"iteration_limit: position={pos:.6g} orientation={rot:.6g}")
            except (mink.exceptions.MinkError, ValueError) as e:
                failures.append(str(e))
        self.configuration.update(initial)
        raise ValueError(f"Pose IK failed after {self.retries} bounded attempts: {failures}")


def plan_drawing(drawing, config, model=None):
    model = drawing_model() if model is None else model
    solver = PoseIK(model, **config.get("ik", {}))
    fixed = quaternion(config["tool_orientation_wxyz"])
    rotation = mink.SO3(quaternion(config.get("cad_rotation_wxyz", [1, 0, 0, 0]))).as_matrix()
    translation = np.asarray(config["cad_origin_m"], dtype=float)
    lift = float(config.get("lift_m", .02))
    step = float(config.get("max_cartesian_step_m", .002))
    if translation.shape != (3,) or not np.isfinite(translation).all() or not all(
            math.isfinite(x) and x > 0 for x in (lift, step)):
        raise ValueError("Invalid placement or transition options")
    normal = rotation[:, 2]
    initial = solver.current_pose()
    path = [JointWaypoint(solver.configuration.q[solver.qadr].tolist(), initial, False, -1, 0., 0.)]

    def move(position, down, stroke):

        start = np.array(solver.current_pose().position)
        points = resample([start, position], step)
        q_start = np.array(solver.current_pose().orientation)
        q_end = fixed.copy()
        if q_start @ q_end < 0: q_end = -q_end
        angle = math.acos(float(np.clip(q_start @ q_end, -1, 1)))
        n = max(len(points) - 1, math.ceil(2 * angle / .02), 1)
        for k in range(1, n + 1):
            t = k / n
            orient = (1 - t) * q_start + t * q_end if angle < 1e-6 else (
                math.sin((1-t)*angle) * q_start + math.sin(t*angle) * q_end) / math.sin(angle)
            target = ToolPose((start + t * (position - start)).tolist(), orient.tolist())
            waypoint = solver.solve(target, pen_down=down, stroke=stroke)

            previous = np.array(path[-1].joints)
            for u in np.linspace(0, 1, 5):
                q = solver.configuration.q.copy()
                q[solver.qadr] = previous + u * (np.array(waypoint.joints) - previous)
                if not solver.valid(q):
                    raise ValueError("Joint edge collision or joint limit")
            path.append(waypoint)

    for i, stroke in enumerate(drawing.strokes):
        points = np.asarray(stroke.points) * drawing.metres_per_unit @ rotation.T + translation

        if np.max(np.abs((points - translation) @ normal)) > 1e-8:
            raise ValueError("Planar drawing requires CAD geometry in its XY plane")
        move(points[0] + lift * normal, False, i)
        move(points[0], False, i)
        path.append(solver.solve(ToolPose(points[0].tolist(), fixed.tolist()), pen_down=True, stroke=i))
        for point in points[1:]:
            move(point, True, i)
        path.append(solver.solve(ToolPose(points[-1].tolist(), fixed.tolist()), pen_down=False, stroke=i))
        move(points[-1] + lift * normal, False, i)
    return model, path


def path_dict(path):
    return {"schema": "hand.joint_path.v1", "waypoints": [asdict(p) for p in path]}
