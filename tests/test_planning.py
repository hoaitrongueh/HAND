import json
from pathlib import Path
import mink
import numpy as np
import pytest
from drawing.geometry import load_dxf
from drawing.planning import ROOT, PoseIK, ToolPose, drawing_model, plan_drawing


@pytest.fixture(scope="module")
def model():
    return drawing_model()


def test_pose_orientation_is_solved(model):
    ik = PoseIK(model)
    target = ik.current_pose()
    rotation = mink.SO3.exp(np.array([.02, -.01, .03])) @ mink.SO3(np.array(target.orientation))
    target = ToolPose(target.position, rotation.wxyz.tolist())
    before = ik.configuration.q.copy()
    result = ik.solve(target)
    assert result.position_residual_m < 1e-5
    assert result.orientation_residual_rad < 1e-4
    assert np.max(np.abs(ik.configuration.q-before)) > .005


def test_failed_ik_bounded_atomic(model, monkeypatch):
    ik = PoseIK(model, iterations=3, retries=2)
    before = ik.configuration.q.copy()
    count = 0
    solve = mink.solve_ik
    def wrapped(*args, **kwargs):
        nonlocal count
        count += 1
        return solve(*args, **kwargs)
    monkeypatch.setattr(mink, "solve_ik", wrapped)
    with pytest.raises(ValueError, match="bounded attempts"):
        ik.solve(ToolPose([8, 8, 8], [1, 0, 0, 0]))
    assert count <= 6
    assert np.array_equal(ik.configuration.q, before)


def test_invalid_initial_joint_limit(model):
    ik = PoseIK(model)
    q = ik.configuration.q.copy(); q[3] = 0
    ik.configuration.update(q)
    with pytest.raises(ValueError, match="Invalid initial"):
        ik.solve(ik.current_pose())


def test_near_joint_limit_noop(model):
    ik = PoseIK(model)
    q = ik.configuration.q.copy(); q[0] = model.joint("joint1").range[1] - 1e-5
    ik.configuration.update(q)
    result = ik.solve(ik.current_pose())
    assert result.position_residual_m < 1e-10
    assert np.array_equal(ik.configuration.q, q)


def test_unreachable_dxf_path_halts(model):
    config = json.loads((ROOT / "configs/drawing.json").read_text())
    config["ik"]["iterations"] = 5
    config["ik"]["retries"] = 1
    with pytest.raises(ValueError):
        plan_drawing(load_dxf(ROOT / "tests/fixtures/unreachable.dxf"), config, model)


@pytest.mark.parametrize("q", [[0, 0, 0, 0], [float("nan"), 1, 0, 0], [1, 0, 0]])
def test_invalid_quaternion(q):
    with pytest.raises(ValueError): ToolPose([0, 0, 0], q)


def test_quaternion_normalization():
    assert ToolPose([0, 0, 0], [2, 0, 0, 0]).orientation == [1, 0, 0, 0]
