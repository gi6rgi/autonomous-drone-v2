"""Export placed assembly bodies as base64 STL (JSON) for the web viewer.
Run: .venv/bin/python viewer_data.py <out.json> [module]   (module: build | accessories)
"""
import base64
import json
import sys
import tempfile
from pathlib import Path

import cadquery as cq
from cadquery import exporters

import importlib

COLORS = {
    "camera_mount": "#d9542b", "gps_mount": "#d3a52a",   # must come before "camera": prefix match
    "plate": "#8c9096", "deck": "#3b6ea5", "standoff": "#c9ccd1", "gps_mast": "#d3a52a", "camera": "#1f5c3a", "lens": "#333",
    "arm": "#7a7d80", "plate_A": "#4b4f52", "plate_B": "#5b5f63", "deck_E": "#3b6ea5",
    "tray_H": "#5a6b3a", "leg": "#e8792f", "camera_F": "#c8342b", "mast_G": "#d3a52a",
    "motor": "#1f7a7a", "prop": "#8a8f94", "stack": "#222", "pi5": "#2f8f3a",
    "battery": "#243a7a", "gps": "#dddddd",
}
MASS = {n: 1 for n in COLORS}


def color_for(name):
    for k, v in COLORS.items():
        if name.startswith(k):
            return k, v
    return "other", "#999"


def main(out_path, module="build"):
    build = importlib.import_module(module)
    asm, parts = build.assembly()
    rho = 1.27e-3
    items = []
    tmp = Path(tempfile.mkdtemp())
    for name, shape, loc in build._walk(asm):
        placed = shape.moved(loc)
        kind, col = color_for(name)
        group = "reference" if kind in ("motor", "prop", "stack", "pi5", "battery", "gps", "gps_mast", "camera", "lens") else "printed"
        f = tmp / f"{name}.stl"
        exporters.export(cq.Workplane().add(placed), str(f), tolerance=0.08, angularTolerance=0.25)
        items.append({
            "name": name, "kind": kind, "group": group, "color": col,
            "mass_g": round(shape.Volume() * rho, 1) if group == "printed" else None,
            "stl": base64.b64encode(f.read_bytes()).decode(),
        })
    Path(out_path).write_text(json.dumps(items))
    total = sum(len(i["stl"]) for i in items)
    print(f"{len(items)} bodies, {total/1e6:.2f} MB base64")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "build")
