import math
import ezdxf
import numpy as np
import pytest
from drawing.geometry import load_dxf, simplify


def save(tmp_path, build, units=4):
    doc = ezdxf.new(); doc.units = units
    build(doc.modelspace())
    path = tmp_path / "drawing.dxf"; doc.saveas(path)
    return path


def test_order_units_boundaries_duplicates(tmp_path):
    def build(m):
        m.add_line((4, 5), (1, 2)); m.add_line((4, 5), (1, 2))
        m.add_lwpolyline([(0, 0), (1, 0), (1, 1)], close=True)
    d = load_dxf(save(tmp_path, build), max_segment=.2)
    assert d.metres_per_unit == .001 and len(d.strokes) == 3
    assert d.strokes[0].points == d.strokes[1].points
    assert d.strokes[0].points[0] == [4, 5, 0]
    assert d.strokes[0].points[-1] == [1, 2, 0]
    assert d.strokes[2].closed
    for s in d.strokes:
        assert np.max(np.linalg.norm(np.diff(s.points, axis=0), axis=1)) <= .2 + 1e-12


def test_conditional_simplification(tmp_path):
    p = [(x, 0) for x in np.linspace(0, 10, 100)]
    d = load_dxf(save(tmp_path, lambda m: m.add_lwpolyline(p)))
    assert d.strokes[0].source_vertices == 100
    assert d.strokes[0].simplified_vertices == 2
    corner = np.array([[0, 0, 0], [1, 1, 0], [2, 0, 0]])
    assert len(simplify(corner, .1)) == 3


def test_circle_sagitta_and_bulge(tmp_path):
    d = load_dxf(save(tmp_path, lambda m: m.add_circle((0, 0), 10)), deviation=.001)
    p = np.array(d.strokes[0].points)
    assert np.max(10 - np.linalg.norm((p[:-1] + p[1:]) / 2, axis=1)) <= .001 + 1e-12
    assert p[1, 1] > 0
    d = load_dxf(save(tmp_path, lambda m: m.add_lwpolyline([(0, 0, 1), (10, 0, 0)], format="xyb")))
    assert len(d.strokes[0].points) > 2
    assert min(p[1] for p in d.strokes[0].points) < -4


def test_extrusion(tmp_path):
    d = load_dxf(save(tmp_path, lambda m: m.add_lwpolyline([(1, 2), (3, 4)],
                 dxfattribs={"extrusion": (0, 0, -1)})))
    assert d.strokes[0].points[0] == [-1, 2, 0]


@pytest.mark.parametrize("bulge", [-1, 1])
def test_bulge_preserves_traversal(tmp_path, bulge):
    d = load_dxf(save(tmp_path, lambda m: m.add_lwpolyline(
        [(0, 0, bulge), (10, 0, 0), (12, 3, 0)], format="xyb")))
    p = np.array(d.strokes[0].points)
    assert np.allclose(p[0], [0, 0, 0]) and np.allclose(p[-1], [12, 3, 0])
    assert (p[1, 1] > 0) == (bulge < 0)


@pytest.mark.parametrize("build,units", [(lambda m: None, 4),
    (lambda m: m.add_text("unsupported"), 4), (lambda m: m.add_line((0, 0), (0, 0)), 4),
    (lambda m: m.add_line((0, 0), (1, 0)), 0)])
def test_invalid_drawings(tmp_path, build, units):
    with pytest.raises(ValueError): load_dxf(save(tmp_path, build, units))


def test_malformed(tmp_path):
    p = tmp_path / "bad.dxf"; p.write_text("not dxf")
    with pytest.raises(ValueError): load_dxf(p)


@pytest.mark.parametrize("opts", [{"deviation": math.nan}, {"max_segment": 0}, {"simplify_tolerance": -1}])
def test_invalid_options(tmp_path, opts):
    with pytest.raises(ValueError): load_dxf(tmp_path / "missing.dxf", **opts)


def test_pathological_sample_budget(tmp_path):
    p = save(tmp_path, lambda m: m.add_line((0, 0), (1e9, 0)))
    with pytest.raises(ValueError, match="budget"):
        load_dxf(p)
