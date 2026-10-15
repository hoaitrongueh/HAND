
from pathlib import Path
import time

import mujoco

MODEL_PATH = (
    Path(__file__).resolve().parent.parent
    / "simulation"
    / "models"
    / "franka_emika_panda"
    / "scene.xml"
)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

home_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_KEY, "home"
)

if home_id == -1:
    raise RuntimeError("Home configuration not found")

mujoco.mj_resetDataKeyframe(model, data, home_id)


simulation_duration = 10.0
steps = round(simulation_duration / model.opt.timestep)

start = time.perf_counter()

for _ in range(steps):
    mujoco.mj_step(model, data)

elapsed = time.perf_counter() - start
speed = data.time / elapsed

print(f"Physics timestep: {model.opt.timestep}")
print(f"Physics steps: {steps}")
print(f"Simulated time: {data.time:.2f} s")
print(f"Real time: {elapsed:.2f} s")
print(f"Simulation speed: {speed:.2f}x real time")
