"""Prop and motor shaft clearance to the floor, level and tilted.
The drone can rest on any face of the convex hull of its frame; for every such face that faces
down (tilt up to 60 deg) we measure the gap from the prop blades and the shaft ends to that floor.
Run after export.py: .venv/bin/python ground_check.py"""
import glob
import math

import numpy as np
import trimesh

from layout import MX, MY, PROP_R, PROP_Z

PROP_SWEEP = 3.5        # blades sweep PROP_Z +- 3.5 (7 mm hub, estimate)
SHAFT_END = -33.05      # motor drawing: 33.05 overall with shaft, base at z = 0
NUT_R, NUT_Z = 4.0, -31.0

frame = [trimesh.load(f) for f in glob.glob("out/*.stl") if "coupon" not in f]
hull = trimesh.util.concatenate(frame).convex_hull

motors = [(sx * MX, sy * MY) for sx in (1, -1) for sy in (1, -1)]
pts, kind = [], []
for mx, my in motors:
    for a in np.linspace(0, 2 * math.pi, 180, endpoint=False):
        for r in (PROP_R, PROP_R * 0.7, PROP_R * 0.4):
            pts.append((mx + r * math.cos(a), my + r * math.sin(a), PROP_Z - PROP_SWEEP)); kind.append("prop")
    pts.append((mx, my, SHAFT_END)); kind.append("shaft")
    for a in np.linspace(0, 2 * math.pi, 12, endpoint=False):
        pts.append((mx + NUT_R * math.cos(a), my + NUT_R * math.sin(a), NUT_Z)); kind.append("shaft")
P = np.array(pts)
kind = np.array(kind)

V = hull.vertices
rows = []
for n in hull.face_normals:
    if n[2] > -math.cos(math.radians(60)):
        continue                       # not a floor the drone can rest on
    d = (V @ n).max()                  # floor plane: n.x = d, everything on the side n.x <= d
    gap = d - P @ n
    tilt = math.degrees(math.acos(-n[2]))
    rows.append((tilt, gap[kind == "prop"].min(), gap[kind == "shaft"].min()))
rows = np.array(rows)
level = rows[rows[:, 0] < 0.5]
print(f"level on the skids: blades {level[:, 1].min():.1f} mm, shaft/nut {level[:, 2].min():.1f} mm above the floor")
for lim in (5, 15, 30, 60):
    sel = rows[rows[:, 0] <= lim]
    print(f"any rest position up to {lim:2d} deg tilt: blades >= {sel[:, 1].min():.1f} mm, shaft/nut >= {sel[:, 2].min():.1f} mm")
inside = all(math.hypot(x - mx, y - my) < 67.0 for (x, y, z), k in zip(P, kind) for mx, my in [min(motors, key=lambda m: math.hypot(x - m[0], y - m[1]))])
print("blades and shafts lie inside the ring cylinders:", inside)
