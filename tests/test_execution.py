import json
import subprocess
import mujoco
import numpy as np
import pytest
from drawing.planning import ROOT, drawing_model
from drawing.timing import write_trajectory

EXE = ROOT / "build/hand_draw.exe"
pytestmark = pytest.mark.skipif(not EXE.exists(), reason="Build HAND_BUILD_MUJOCO first")


@pytest.fixture
def command(tmp_path):
    model = drawing_model()
    mujoco.mj_saveModel(model, str(tmp_path / "model.mjb"))
    data = mujoco.MjData(model); mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    site = model.site("pen_tip").id
    quat = np.zeros(4); mujoco.mju_mat2Quat(quat, data.site_xmat[site])
    rows = np.array([[i*.002, *data.qpos[:7], *data.site_xpos[site], *quat, int(i in (1, 2)), 0]
                     for i in range(4)])
    def invoke(rows):
        write_trajectory(tmp_path / "trajectory.csv", rows)
        return subprocess.run([str(EXE), str(tmp_path / "model.mjb"), str(tmp_path / "trajectory.csv"),
                               str(tmp_path / "trace"), ".2", ".4", *map(str, data.site_xpos[site]), "0", "0", "1"], capture_output=True, text=True)
    return rows, invoke, tmp_path


def test_real_executor_steps_and_records(command):
    rows, invoke, directory = command
    result = invoke(rows)
    assert result.returncode == 0, result.stderr
    report = json.loads((directory / "trace/execution.json").read_text())
    assert report["steps"] == 3
    assert report["sim_time_s"] == .006
    assert (directory / "trace/executed.csv").exists()


@pytest.mark.parametrize("name", ["nan", "joint_limit", "time", "quaternion", "pen", "fk", "timestep"])
def test_bad_trajectory_never_steps(command, name):
    rows, invoke, directory = command
    if name == "nan": rows[1, 1] = np.nan
    elif name == "joint_limit": rows[1, 4] = 0
    elif name == "time": rows[1, 0] = 0
    elif name == "quaternion": rows[1, 11:15] = 0
    elif name == "pen": rows[0, 15] = 1
    elif name == "fk": rows[1, 8] += .02
    else: rows[1:, 0] *= 1.5
    result = invoke(rows)
    assert result.returncode != 0
    assert not (directory / "trace/executed.csv").exists()


def test_parser_rejects_wrong_schema(command):
    _, _, directory = command
    path = directory / "bad.csv"; path.write_text("# unknown\n")
    result = subprocess.run([str(EXE), str(directory / "model.mjb"), str(path), str(directory / "trace"), ".2", ".4", "0", "0", "0", "0", "0", "1"], capture_output=True)
    assert result.returncode != 0 and not (directory / "trace").exists()
