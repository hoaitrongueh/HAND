import json
import subprocess
import sys
import pytest
from drawing.planning import ROOT


@pytest.mark.acceptance
@pytest.mark.skipif(not (ROOT / "build/hand_draw.exe").exists(), reason="Build C++ executor first")
@pytest.mark.parametrize("fixture", ["house", "curves", "dense"])
def test_dxf_to_actual_actuators(tmp_path, fixture):
    output = tmp_path / fixture
    p = subprocess.run([sys.executable, "-m", "drawing.run", str(ROOT / f"tests/fixtures/{fixture}.dxf"),
                        "--output", str(output)], cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert p.returncode == 0, p.stdout+p.stderr
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "completed"
    assert report["execution"]["steps"] > 1000
    assert report["diagnostics"]["drawing_coverage"] >= .99
    assert report["diagnostics"]["executed_joint_motion_rad"] > .01
    assert report["execution"]["joint_tracking_max_rad"] < .005
    assert report["execution"]["cartesian_tracking_max_m"] < .001
    assert (output / "planned.csv").exists() and (output / "executed.csv").exists()


@pytest.mark.acceptance
@pytest.mark.parametrize("fixture,stage", [("unreachable", "pose_ik"), ("malformed", "geometry")])
def test_bad_input_never_reaches_execution(tmp_path, fixture, stage):
    output = tmp_path / fixture
    p = subprocess.run([sys.executable, "-m", "drawing.run", str(ROOT / f"tests/fixtures/{fixture}.dxf"),
                        "--output", str(output)], cwd=ROOT, capture_output=True, timeout=120)
    assert p.returncode == 1
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "failed" and report["stage"] == stage
    assert not (output / "executed.csv").exists()
