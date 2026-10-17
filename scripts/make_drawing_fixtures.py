from pathlib import Path
import ezdxf
import numpy as np


def main():
    root = Path(__file__).resolve().parents[1] / "tests" / "fixtures"
    root.mkdir(exist_ok=True)
    for name in ("house", "curves", "dense", "unreachable"):
        d = ezdxf.new("R2010"); d.units = ezdxf.units.MM
        m = d.modelspace()
        if name == "house":
            m.add_lwpolyline([(0, 0), (50, 0), (50, 40), (25, 65), (0, 40)], close=True)
            m.add_lwpolyline([(18, 0), (18, 25), (32, 25), (32, 0)])
        elif name == "curves":
            m.add_circle((25, 25), 20)
            m.add_arc((25, 25), 10, 0, 180)
            m.add_line((0, 0), (50, 0))
        elif name == "dense":
            m.add_lwpolyline([(float(x), float(10 * np.sin(x / 10))) for x in np.linspace(0, 50, 251)])
        else:
            m.add_line((5000, 0), (5010, 10))

        d.saveas(root / f"{name}.dxf")
    (root / "malformed.dxf").write_text("This is deliberately not a DXF file.\n")


if __name__ == "__main__":
    main()
