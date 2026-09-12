"""Export all parts in print orientation → print/*.stl (one 3MF-compatible STL set).
Run: .venv/bin/python export_print.py
"""
from pathlib import Path

import cadquery as cq
from cadquery import exporters

import accessories as A
import plate_v2 as P

PRINT = Path(__file__).parent / "print"
PRINT.mkdir(exist_ok=True)


def on_bed(shape):
    """Drop part to Z=0 and shift into the positive quadrant."""
    bb = shape.val().BoundingBox()
    return shape.translate((-bb.xmin, -bb.ymin, -bb.zmin))


def export(name, shape, qty, note):
    shape = on_bed(shape)
    bb = shape.val().BoundingBox()
    assert bb.xlen <= 220 and bb.ylen <= 220 and bb.zlen <= 250, f"{name} does not fit"
    exporters.export(shape, str(PRINT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    print(f"{name:22s} x{qty}  {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f}  {shape.val().Volume()*1.27e-3:6.1f} g  {note}")


def test_motor_pad():
    """Test motor pad: same pattern and thickness as the plate. Print first."""
    pad = cq.Workplane("XY").circle(P.PAD_D / 2).extrude(P.PLATE_T)
    pts = [(dx, dy) for dx, dy in ((P.MOTOR_HOLE_R, 0), (-P.MOTOR_HOLE_R, 0), (0, P.MOTOR_HOLE_R), (0, -P.MOTOR_HOLE_R))]
    pad = pad.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(P.HW.m3_through if hasattr(P, "HW") else 3.4)
    pad = pad.faces(">Z").workplane(origin=(0, 0, 0)).hole(P.MOTOR_CENTER_HOLE)
    if P.MOTOR_CBORE[1] > 0:
        for x, y in pts:                                           # head counterbores from below, as on plate
            pad = pad.cut(cq.Workplane("XY").center(x, y).circle(P.MOTOR_CBORE[0] / 2).extrude(P.MOTOR_CBORE[1]))
    return pad


def main():
    export("test_motor_pad", test_motor_pad(), 1, "TEST: print first, test-fit motor (10 min)")
    export("plate_v2", P.plate(), 1, "flat, arm side ribs up; 6 perimeters, 40 % gyroid")
    export("deck", A.deck(), 1, "flat; 4 perimeters, 30 %")
    # legs: flange on bed, blade up → flip 180° about X; no supports
    export("leg", A.leg().rotate((0, 0, 0), (1, 0, 0), 180), 4, "standing on flange, blade up; no supports; 100 % or 6 perimeters")
    export("gps_mount", A.gps_mount(), 1, "standing, flange down; 4 perimeters; no supports needed (45° cone)")
    export("camera_mount", A.camera_mount(), 1, "flange down; 4 perimeters, 100 %; no supports. camera board slides into the groove from the top until it clicks")
    export("standoff_35", A.standoff_for_print(), 4, "LYING FLAT (as exported): hex face on bed; 100 %; no supports. M4x8 cuts its own thread in the ø3.5 pilot")


if __name__ == "__main__":
    main()
