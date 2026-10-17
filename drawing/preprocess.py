import argparse
import json
from pathlib import Path
from .geometry import load_dxf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-segment", type=float, default=2.0, help="CAD units")
    parser.add_argument("--deviation", type=float, default=.05, help="CAD units")
    parser.add_argument("--simplify", type=float, default=.02, help="CAD units; 0 disables")
    parser.add_argument("--dense-threshold", type=int, default=32)
    parser.add_argument("--units", type=int, help="explicit DXF unit code override")
    args = parser.parse_args()
    try:
        drawing = load_dxf(args.input, max_segment=args.max_segment, deviation=args.deviation,
                           simplify_tolerance=args.simplify, dense_threshold=args.dense_threshold,
                           unit_override=args.units)
    except ValueError as e:
        parser.exit(2, f"Geometry rejected: {e}\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(drawing.to_dict(), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"schema": drawing.schema, "strokes": len(drawing.strokes),
                      "samples": sum(len(s.points) for s in drawing.strokes),
                      "metres_per_unit": drawing.metres_per_unit,
                      "approximation_bound_cad": drawing.approximation_bound_cad}))


if __name__ == "__main__":
    main()
