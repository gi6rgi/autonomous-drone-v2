"""Export v3 parts: assembly-position STL/STEP to out/, print-oriented STL to print/.
Run: .venv/bin/python export.py"""
from pathlib import Path

import cadquery as cq
from cadquery import exporters

from layout import DENSITY_PLA
import topbox
from parts import build_all

HERE = Path(__file__).parent
OUT, PRINT = HERE / "out", HERE / "print"

# name in print/ : (part in the assembly, qty, how it goes on the bed)
PRINT_SET = {
    "arm_a": ("arm_FL", 2, "flat side down (tongue and pad on the bed), use for FL and RR"),
    "arm_b": ("arm_FR", 2, "flat side down, use for FR and RL"),
    "ring_a": ("ring_FL", 2, "upright (bottom rim on the bed), gussets carry the tabs; use for FL and RR"),
    "ring_b": ("ring_FR", 2, "upright, use for FR and RL"),
    "bridge": ("bridge_L", 2, "flat"),
    "spine_bottom": ("spine_bottom", 1, "floor on the bed"),
    "spine_top": ("spine_top", 1, "flat"),
    "skid": ("skid_L", 2, "PETG! on its side, nut slots up; same part left and right (turned 180 deg)"),
    "box_cradle": ("box_cradle", 1, "flat underside on the bed, posts up"),
    "box_deck": ("box_deck", 1, "flat underside on the bed, Pi bosses up"),
    "box_camera": ("box_camera", 1, "flange on the bed"),
    "box_spacer": ("box_spacer", 1, "flat; 4 mm, balance spacer at the stop wall"),
}


def to_bed(wp, flip=False, side=False, lay=False):
    s = wp.val()
    if lay:    # thin in x: lay it flat
        s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90)
    if side:   # y up: the skid profile lies on the bed
        s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90)
    if flip:
        s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def main():
    OUT.mkdir(exist_ok=True)
    PRINT.mkdir(exist_ok=True)
    parts, info = build_all()
    parts.update(topbox.build_box([b for a in info["arms"] for b in a.bolts]))
    for name, wp in topbox.refs().items():   # components, not printed: for the viewer and the simulator
        exporters.export(wp, str(OUT / f"{name}.stl"), tolerance=0.1, angularTolerance=0.3)
    asm = cq.Assembly()
    for name, wp in parts.items():
        exporters.export(wp, str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
        asm.add(wp, name=name)
    asm.save(str(OUT / "frame_v3_assembly.step"))
    total = 0.0
    rows = []
    for pname, (src, qty, how) in PRINT_SET.items():
        s = to_bed(parts[src], flip=False, side=pname == "skid", lay=pname == "box_spacer")
        exporters.export(cq.Workplane("XY").add(s), str(PRINT / f"{pname}.stl"), tolerance=0.05, angularTolerance=0.2)
        m = s.Volume() * DENSITY_PLA
        total += m * qty
        bb = s.BoundingBox()
        rows.append(f"{pname:13s} x{qty}  {bb.xlen:5.1f} x {bb.ylen:5.1f} x {bb.zlen:4.1f}  solid {m:5.1f} g  {how}")
    print("\n".join(rows))
    print(f"total solid PLA {total:.0f} g")


if __name__ == "__main__":
    main()
