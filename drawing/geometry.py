from dataclasses import asdict, dataclass
from pathlib import Path
import math

import ezdxf
import numpy as np

MAX_POINTS = 200_000


@dataclass
class Stroke:
    entity: str
    kind: str
    closed: bool
    points: list[list[float]]
    source_vertices: int
    simplified_vertices: int
    approximation_error_cad: float


@dataclass
class Drawing:
    schema: str
    units: int
    metres_per_unit: float
    strokes: list[Stroke]
    approximation_bound_cad: float

    def to_dict(self):
        return asdict(self)


def point_segment_distance(points, a, b):
    points, a, b = np.asarray(points), np.asarray(a), np.asarray(b)
    v = b - a
    t = np.zeros(len(points)) if v @ v == 0 else np.clip((points - a) @ v / (v @ v), 0, 1)
    return np.linalg.norm(points - (a + t[:, None] * v), axis=1)


def simplify(points, tolerance):

    p = np.asarray(points, dtype=float)
    keep = {0, len(p) - 1}
    stack = [(0, len(p) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        distances = point_segment_distance(p[a + 1:b], p[a], p[b])
        i = a + 1 + int(np.argmax(distances))
        if distances[i - a - 1] > tolerance:
            keep.add(i)
            stack.extend(((a, i), (i, b)))
    return p[sorted(keep)]


def resample(points, max_segment):
    result = [np.asarray(points[0], dtype=float)]
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a), np.asarray(b)
        distance = float(np.linalg.norm(b - a))
        if not math.isfinite(distance):
            raise ValueError("Unrepresentable segment length")
        n = max(1, math.ceil(distance / max_segment))
        if len(result) + n > MAX_POINTS:
            raise ValueError("Stroke exceeds sample budget")
        result.extend(a + (b - a) * (i / n) for i in range(1, n + 1))
    return np.asarray(result)


def _arc(entity, max_segment, deviation):
    radius = float(entity.dxf.radius)
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("Invalid arc radius")
    start = 0 if entity.dxftype() == "CIRCLE" else math.radians(entity.dxf.start_angle)
    sweep = 2 * math.pi if entity.dxftype() == "CIRCLE" else math.radians(
        (entity.dxf.end_angle - entity.dxf.start_angle) % 360)

    angle = min(math.pi / 2, 2 * math.acos(max(-1, 1 - deviation / radius)), max_segment / radius)
    if angle <= 0 or sweep <= 0:
        raise ValueError("Degenerate arc or unrepresentable tolerance")
    n = math.ceil(sweep / angle)
    if n + 1 > MAX_POINTS:
        raise ValueError("Arc exceeds sample budget")
    center = np.asarray(entity.dxf.center)
    ocs = entity.ocs()
    return np.asarray([ocs.to_wcs(center + radius * np.array([math.cos(t), math.sin(t), 0]))
                       for t in np.linspace(start, start + sweep, n + 1)])


def _arc_error(entity, points):
    r = float(entity.dxf.radius)
    chord = float(np.linalg.norm(np.diff(points, axis=0), axis=1).max())

    return (chord*chord/4) / (r + math.sqrt(max(0, r*r-chord*chord/4)))


def load_dxf(path: Path, *, max_segment=2.0, deviation=0.05,
             simplify_tolerance=0.02, dense_threshold=32, unit_override=None):


    if (not all(math.isfinite(x) and x > 0 for x in (max_segment, deviation)) or
            not math.isfinite(simplify_tolerance) or simplify_tolerance < 0 or dense_threshold < 3):
        raise ValueError("Invalid geometry tolerances")
    try:
        doc = ezdxf.readfile(path)
        if doc.audit().has_errors:
            raise ValueError("DXF audit failed")
    except (OSError, ezdxf.DXFError) as e:
        raise ValueError(f"Cannot read DXF: {e}") from e
    units = int(doc.units if unit_override is None else unit_override)
    if units == 0:
        raise ValueError("Unitless DXF requires explicit unit_override")
    try:
        scale = ezdxf.units.conversion_factor(units, ezdxf.units.M)
    except (ValueError, TypeError) as e:
        raise ValueError("Unsupported CAD units") from e
    strokes = []
    total_points = 0
    for entity in doc.modelspace():
        kind = entity.dxftype()
        closed = kind == "CIRCLE"
        linear_poly = False
        approximation = 0.
        if kind == "LINE":
            points = np.array([entity.dxf.start, entity.dxf.end])
        elif kind in ("ARC", "CIRCLE"):
            points = _arc(entity, max_segment, deviation)
            approximation = _arc_error(entity, points)
        elif kind in ("LWPOLYLINE", "POLYLINE"):
            if kind == "POLYLINE" and (entity.is_polygon_mesh or entity.is_poly_face_mesh):
                raise ValueError("Mesh POLYLINE is not a drawing stroke")
            closed = bool(entity.closed if kind == "LWPOLYLINE" else entity.is_closed)
            if kind == "POLYLINE" and entity.is_3d_polyline:
                points = np.array([v.dxf.location for v in entity.vertices])
                if closed and len(points):
                    points = np.vstack([points, points[0]])
                linear_poly = True
            else:
                pieces = list(entity.virtual_entities())
                linear_poly = all(p.dxftype() == "LINE" for p in pieces)
                parts = [np.array([p.dxf.start, p.dxf.end]) if p.dxftype() == "LINE"
                         else _arc(p, max_segment, deviation) for p in pieces]
                approximation = max((_arc_error(p, part) for p, part in zip(pieces, parts)
                                     if p.dxftype() != "LINE"), default=0.)


                vertices = list(entity.vertices_in_wcs() if kind == "LWPOLYLINE"
                                else entity.points_in_wcs())
                current = np.asarray(vertices[0]) if vertices else None
                for i, part in enumerate(parts):
                    if np.linalg.norm(part[0] - current) > 1e-8:
                        if np.linalg.norm(part[-1] - current) > 1e-8:
                            raise ValueError("Discontinuous virtual polyline")
                        parts[i] = part[::-1]
                    current = parts[i][-1]
                points = np.vstack([parts[0], *[p[1:] for p in parts[1:]]]) if parts else np.empty((0, 3))
        else:
            raise ValueError(f"Unsupported DXF entity {kind} ({entity.dxf.handle})")
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError(f"Invalid coordinates in {kind}")
        if np.linalg.norm(np.diff(points, axis=0), axis=1).sum() <= 1e-12:
            raise ValueError("Zero-length stroke")
        original = len(points)
        if linear_poly and original >= dense_threshold and simplify_tolerance > 0:
            source = points
            points = simplify(source, simplify_tolerance)
            distances = np.full(len(source), np.inf)
            for a, b in zip(points[:-1], points[1:]):
                distances = np.minimum(distances, point_segment_distance(source, a, b))
            approximation = float(distances.max())
        simplified = len(points)
        points = resample(points, max_segment)
        total_points += len(points)
        if total_points > MAX_POINTS:
            raise ValueError("Drawing exceeds sample budget")
        strokes.append(Stroke(entity.dxf.handle, kind, closed, points.tolist(), original, simplified, approximation))
    if not strokes:
        raise ValueError("DXF contains no drawing strokes")
    return Drawing("hand.strokes.v1", units, float(scale), strokes,
                   max(deviation, simplify_tolerance))
