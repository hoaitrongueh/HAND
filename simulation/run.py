from pathlib import Path
from controller import interpolate

import mujoco
import mujoco.viewer
import time

MODEL_PATH=(Path(__file__).resolve().parent
            /"models"
            /"franka_emika_panda"
            /"scene.xml")
model=mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data=mujoco.MjData(model)



home_id=mujoco.mj_name2id(
    model,mujoco.mjtObj.mjOBJ_KEY,"home"
)

if home_id==-1:
    raise RuntimeError("Panda home config not found")
mujoco.mj_resetDataKeyframe(model,data,home_id)
mujoco.mj_forward(model,data)


print("Joint positions:", data.qpos)
print("Actuator controls:", data.ctrl)
print("Number of actuators:", model.nu)

joint_id=mujoco.mj_name2id(
    model,mujoco.mjtObj.mjOBJ_JOINT,"joint1"
)
actuator_id=mujoco.mj_name2id(
    model,mujoco.mjtObj.mjOBJ_ACTUATOR,"actuator1"
)

if joint_id==-1 or actuator_id==-1:
    raise RuntimeError("Joint 1 or Actuator 1 not found")
joint_qpos_index=model.jnt_qposadr[joint_id]
home_angle=float(data.qpos[joint_qpos_index])
target_angle=home_angle + 0.5
lower,upper=model.actuator_ctrlrange[actuator_id]
if not lower <= target_angle <= upper:
    raise ValueError("Target exceeds actuator limits")
print("Panda loaded successfully")
print(f"Home angle: {home_angle:.3f} rad")
print(f"Target angle: {target_angle:.3f} rad")

wall_start = time.perf_counter()

last_report_time = -1.0
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        frame_start = time.perf_counter()

        steps_per_frame = max(
            1, round((1.0 / 60.0) / model.opt.timestep)
        )

        for _ in range(steps_per_frame):
            if data.time < 2.0:
                data.ctrl[actuator_id] = home_angle
            elif data.time < 5.0:
                progress=(data.time-2)/3
                desired_angle=interpolate(home_angle,target_angle,progress)
                data.ctrl[actuator_id]=desired_angle
            else:
                data.ctrl[actuator_id] = target_angle

            mujoco.mj_step(model, data)

        viewer.sync()

        actual_angle=float(data.qpos[joint_qpos_index])
        desired_angle=float(data.ctrl[actuator_id])
        error=abs(desired_angle-actual_angle)

        if data.time - last_report_time >=1:
            print(
                f"Time: {data.time:.2f}s | "
                f"Target: {desired_angle:.3f} | "
                f"Actual: {actual_angle:.3f} | "
                f"Error: {error:.4f} rad"
            )
            last_report_time=data.time

        elapsed = time.perf_counter() - frame_start
        remaining = (1.0 / 60.0) - elapsed

        if remaining > 0:
            time.sleep(remaining)
