import argparse
import csv
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description="Replay recorded arm positions from a completed drawing run.")
    parser.add_argument("run_dir", type=Path, help="directory containing model.mjb and executed.csv")
    parser.add_argument("--speed", type=float, default=4.0, help="playback speed multiplier (default: 4)")
    parser.add_argument("--frame-skip", type=int, default=8, help="render every Nth recorded frame (default: 8)")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and positive")
    if args.frame_skip < 1:
        parser.error("--frame-skip must be positive")

    model_path = args.run_dir / "model.mjb"
    trace_path = args.run_dir / "executed.csv"
    for path in (model_path, trace_path):
        if not path.is_file():
            parser.error(f"File not found: {path}")

    with trace_path.open(newline="", encoding="utf-8") as file:
        frames = list(csv.DictReader(file))
    if not frames:
        parser.error("Execution trace is empty")

    model = mujoco.MjModel.from_binary_path(str(model_path))
    data = mujoco.MjData(model)
    joint_addresses = [model.jnt_qposadr[model.joint(f"joint{i}").id] for i in range(1, 8)]
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)

    def display(frame, viewer):
        for joint_index, address in enumerate(joint_addresses, start=1):
            data.qpos[address] = float(frame[f"q{joint_index}"])
        data.time = float(frame["time"])
        mujoco.mj_forward(model, data)
        viewer.sync()

    print(f"Loaded {len(frames)} recorded frames")
    print(f"Playback speed: {args.speed}x; rendering every {args.frame_skip} frames")
    first_time = float(frames[0]["time"])

    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall_start = time.perf_counter()
        indices = list(range(0, len(frames), args.frame_skip))
        if indices[-1] != len(frames) - 1:
            indices.append(len(frames) - 1)
        for index in indices:
            if not viewer.is_running():
                break
            frame = frames[index]
            target_elapsed = (float(frame["time"]) - first_time) / args.speed
            remaining = wall_start + target_elapsed - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            if not viewer.is_running():
                break
            display(frame, viewer)

        if viewer.is_running():
            print(f"Playback finished in {time.perf_counter() - wall_start:.2f} seconds")
            print("Close the MuJoCo window to exit.")
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.05)


if __name__ == "__main__":
    main()
