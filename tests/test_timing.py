import numpy as np
import pytest
from drawing.planning import PoseIK, drawing_model, JointWaypoint, ToolPose
from drawing.timing import time_path


@pytest.fixture
def setup_path():
    model = drawing_model(); ik = PoseIK(model)
    pose = ik.current_pose()
    start = JointWaypoint(ik.configuration.q[:7].tolist(), pose, False, -1, 0., 0.)
    target = ToolPose((np.array(pose.position)+[.001, 0, 0]).tolist(), pose.orientation)
    finish = ik.solve(target)
    return model, [start, finish]


def test_toppra_real_limits(setup_path):
    model, path = setup_path
    rows, report = time_path(path, model)
    assert report["max_velocity_rad_s"] <= .2
    assert report["max_acceleration_rad_s2"] <= .4
    assert np.max(abs(np.diff(rows[:, 1:8], axis=0))) < .01
    assert np.allclose(rows[-1, 1:8], path[-1].joints)


def test_toppra_failure_no_fallback(setup_path, monkeypatch):
    import drawing.timing as timing
    class Failed:
        def __init__(self, *a, **kw): pass
        def compute_trajectory(self, *a): return None
    monkeypatch.setattr(timing.algo, "TOPPRA", Failed)
    with pytest.raises(ValueError, match="TOPPRA failed"):
        time_path(setup_path[1], setup_path[0])


@pytest.mark.parametrize("kw", [{"velocity": 0}, {"acceleration": float("nan")}, {"settle_s": -1}])
def test_invalid_timing_options(setup_path, kw):
    with pytest.raises(ValueError): time_path(setup_path[1], setup_path[0], **kw)
