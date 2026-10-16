from pathlib import Path
from controller import interpolate
import numpy as np
import mujoco
import mujoco.viewer
import time

MODEL_PATH = (
    Path(__file__).resolve().parent / "models" / "franka_emika_panda" / "scene.xml"
)
model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)


home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")

if home_id == -1:
    raise RuntimeError("Panda home config not found")
mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)

hand_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"hand")

print("Joint positions:", data.qpos)
print("Actuator controls:", data.ctrl)
print("Number of actuators:", model.nu)

joint_ids = []
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
for name in joint_names:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    joint_ids.append(joint_id)

actuator_names = [
    "actuator1",
    "actuator2",
    "actuator3",
    "actuator4",
    "actuator5",
    "actuator6",
    "actuator7",
]
actuator_ids = []
for name in actuator_names:
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    actuator_ids.append(actuator_id)

if -1 in joint_ids or -1 in actuator_ids:
    raise RuntimeError("One or more joints or actuators not found")
joint_qpos_indices = []
for joint_id in joint_ids:
    index = model.jnt_qposadr[joint_id]
    joint_qpos_indices.append(index)

home_angles = np.array(data.qpos[joint_qpos_indices], copy=True)

offsets = np.array([0.3, -0.2, 0.2, 0.2, -0.2, 0.2, 0.3])

target_angles = home_angles + offsets


limits = model.actuator_ctrlrange[actuator_ids]

if np.any(target_angles < limits[:, 0]) or np.any(target_angles > limits[:, 1]):
    raise ValueError("One or more targets exceed actuator limits")

last_report_time = -1.0
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        frame_start = time.perf_counter()

        steps_per_frame = max(1, round((1.0 / 60.0) / model.opt.timestep))

        for _ in range(steps_per_frame):
            if data.time < 2.0:
                data.ctrl[actuator_ids] = home_angles
            elif data.time < 5.0:
                progress = (data.time - 2) / 3
                desired_angles = interpolate(home_angles, target_angles, progress)
                data.ctrl[actuator_ids] = desired_angles
            elif data.time < 6:
                data.ctrl[actuator_ids] = target_angles
            elif data.time < 9:
                progress = (data.time - 6) / 3
                data.ctrl[actuator_ids] = interpolate(
                    target_angles, home_angles, progress
                )
            else:
                data.ctrl[actuator_ids] = home_angles

            mujoco.mj_step(model, data)

        viewer.sync()

        actual_angles = np.array(data.qpos[joint_qpos_indices])
        desired_angles = np.array(data.ctrl[actuator_ids])
        errors = np.abs(desired_angles - actual_angles)
        max_error = np.max(errors)
        mean_error = np.mean(errors)
        worst_joint = np.argmax(errors) + 1

        if data.time - last_report_time >= 1:
            hand_position = data.xpos[hand_id].copy()
            print(
                f"Time: {data.time:.2f}s | "
                f"Max error: {max_error:.5f} rad | "
                f"Mean error: {mean_error:.5f} rad | "
                f"Worst joint: {worst_joint}"
            )
            print(
                f"Hand XYZ: "
                f"x={hand_position[0]:.4f}, "
                f"y={hand_position[1]:.4f}, "
                f"z={hand_position[2]:.4f}"
            )
            last_report_time = data.time
            if data.time >= 10.0:
                home_errors = np.abs(data.qpos[joint_qpos_indices] - home_angles)

                print(f"Max home error: {np.max(home_errors):.6f} rad")

        elapsed = time.perf_counter() - frame_start
        remaining = (1.0 / 60.0) - elapsed

        if remaining > 0:
            time.sleep(remaining)
