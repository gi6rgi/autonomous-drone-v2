"""Printed plate (recovered from gcode) as a reference CAD body.

Coordinates: center = mean of the four motors, +X = front, Z=0 is the plate bottom.
Data: data/plate_contours.json (top surface contours),
        data/plate_ribs.json (ribs +2.4 mm). See docs/components/plate-asprinted.md
"""
import json
from pathlib import Path

import cadquery as cq

DATA = Path(__file__).parent / "data"
PLATE_T = 6.0
RIB_H = 2.4
FIN = dict(x=43.5, t=2.6, w=29.6, z0=8.64, z1=40.04,
           holes_y=(-10.5, 10.5), holes_z=(19.9, 32.4), hole_d=2.2,
           window=(12.0, 10.0, 26.0))           # width, height, center z

MOTORS = [(sx * 90.0, sy * 70.0) for sx in (-1, 1) for sy in (-1, 1)]
MOTOR_HOLES = [(mx + dx, my + dy) for mx, my in MOTORS for dx in (-8, 8) for dy in (-8, 8)]
STACK = [(sx * 15.25, sy * 15.25) for sx in (-1, 1) for sy in (-1, 1)]
A_PTS = [(sx * 59.1, sy * 44.9) for sx in (-1, 1) for sy in (-1, 1)]     # ø3.2, pocket 7.3
B_PTS = [(sx * 34.0, sy * 18.8) for sx in (-1, 1) for sy in (-1, 1)]     # ø3.2, pocket 7.5
C_SLOTS = [(sx * 71.2, sy * 55.4) for sx in (-1, 1) for sy in (-1, 1)]   # 7 × 8
D_SLOTS = [(sx * 39.8, sy * 23.8) for sx in (-1, 1) for sy in (-1, 1)]   # 4 × 8


def _poly(pts, h):
    return cq.Workplane("XY").polyline([tuple(p) for p in pts]).close().extrude(h)


def plate():
    loops = json.load(open(DATA / "plate_contours.json"))
    outer = loops[0]["pts"]
    body = _poly(outer, PLATE_T)
    for lp in loops[1:]:                      # windows, hexagon, strap slots
        body = body.cut(_poly(lp["pts"], PLATE_T))
    for lp in json.load(open(DATA / "plate_ribs.json")):
        body = body.union(_poly(lp["pts"], RIB_H).translate((0, 0, PLATE_T)))

    def holes(b, pts, d):
        return b.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(d)
    body = holes(body, MOTOR_HOLES, 3.2)
    body = holes(body, MOTORS, 12.0)
    body = holes(body, STACK, 3.4)
    body = holes(body, A_PTS + B_PTS, 3.2)
    for x, y in C_SLOTS:
        body = body.cut(cq.Workplane("XY").center(x, y).rect(7.0, 8.0).extrude(PLATE_T + RIB_H))
    for x, y in D_SLOTS:
        body = body.cut(cq.Workplane("XY").center(x, y).rect(4.0, 8.0).extrude(PLATE_T + RIB_H))

    # camera plate
    f = FIN
    fin = (cq.Workplane("XY").center(f["x"], 0).rect(f["t"], f["w"]).extrude(f["z1"] - f["z0"])
           .translate((0, 0, f["z0"])))
    for y in f["holes_y"]:
        for z in f["holes_z"]:
            fin = fin.cut(cq.Workplane("YZ").center(y, z).circle(f["hole_d"] / 2).extrude(60, both=True))
    ww, wh, wz = f["window"]
    fin = fin.cut(cq.Workplane("YZ").center(0, wz).rect(ww, wh).extrude(60, both=True))
    return body.union(fin)


if __name__ == "__main__":
    from cadquery import exporters
    p = plate()
    out = Path(__file__).parent / "out"; out.mkdir(exist_ok=True)
    exporters.export(p, str(out / "plate_asprinted.step"))
    exporters.export(p, str(out / "plate_asprinted.stl"), tolerance=0.05, angularTolerance=0.2)
    bb = p.val().BoundingBox()
    print(f"plate bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}, volume {p.val().Volume()/1000:.1f} cm3 "
          f"(PLA 1.24 → {p.val().Volume()*1.24e-3:.0f} g solid; gcode: 83.7 g at 40 %)")
