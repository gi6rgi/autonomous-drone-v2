"""Plate v2: clean parametric version of the printed plate (same mounting positions).

Kept 1:1 from the printed plate: motors (±90, ±70), stack 30.5, holes A (±59.1, ±44.9),
B (±34, ±18.8), center hexagon, ribs, camera plate at the front.
Removed: slots C (7×8), D (4×8), strap slots (±28, 0); not needed (battery is strapped
through the large windows). Added: rib breaks around A and B for standoffs, camera slots instead of holes.
Coordinates: center = mean of motors, +X = front, Z=0 is the plate bottom.
"""
import math
from pathlib import Path

import cadquery as cq

from params import CAM, HW

PLATE_T = 6.0
RIB_H, RIB_W = 2.4, 5.0
BARS = False                         # longitudinal bars between motors: not needed with 6×6 rib on arms (see PRINT.md)
ARM_RIB_W, ARM_RIB_H = 6.0, 6.0      # (obsolete) center rib along arm: T-section 15.8×6 + 6×6 → I ≈ 1330 mm⁴
ARM_STIFFENER = "walls"              # "walls": two side ribs along arm edges (wire channel between them), "rib": old rib on axis
WALL_T, WALL_H = 3.0, 6.0            # side rib: thickness × height. two 3×6 side ribs = same area as 6×6 rib → I ≈ 1330 mm⁴ (U-section)
WALL_INSET = 0.3                     # side rib inset from arm edge (so faces do not coincide with outline)
LEG_NUT_CLEAR_R = 5.5                # side rib break around leg bolts: M3 nyloc + 5.5 socket wrench (or printed pin barbs)
STACK_CBORE = (6.6, 2.5)             # counterbore from below for stack bolt head (M3 button ø6×2): head does not scratch battery
FIN = False                          # camera plate is a separate swappable bracket (accessories.camera_mount)

MOTOR_X, MOTOR_Y = 90.0, 83.5        # was 90 × 70 (wheelbase 228): spread along Y to prop tip gap 37.5 mm; plate 213 × 200
MOTORS = [(sx * MOTOR_X, sy * MOTOR_Y) for sx in (-1, 1) for sy in (-1, 1)]
PAD_D = 30.0                         # motor pad (bell ø27.5)
MOTOR_HOLE_R = 8.0                   # NOTE: LN2207: 16 mm BETWEEN OPPOSITE holes (per drawing) → holes on ø16 circle,
                                     #   square side 11.3 mm. not the standard "16×16" (that gives 22.6 on the diagonal); v2 bug before 2026-09-04
MOTOR_CENTER_HOLE = 7.0              # for shaft end/circlip (motor base hole ø10); web to screw counterbores 1.6 mm
MOTOR_CBORE = (6.2, 0.0)             # counterbore from below for stock screw head (11 mm incl. head, thread ≈ 9). depth 0 = no counterbore:
                                     #   2026-09-05 verified: with 2 mm counterbore (5 mm into motor) screw hits the winding. without counterbore
                                     #   9 − 6 = 3 mm goes into motor (≥ 1·d for aluminium), head protrudes 2 mm below; use threadlocker
BAR_W = 12.0
ARM_W = 15.8
ARM_P0 = (30.0, 12.0)                # start of diagonal arm axis (inside center box), axis runs to motor
_L = math.hypot(MOTOR_X - ARM_P0[0], MOTOR_Y - ARM_P0[1])
ARM_DIR = ((MOTOR_X - ARM_P0[0]) / _L, (MOTOR_Y - ARM_P0[1]) / _L)   # unit vector of arm axis (quadrant +,+)
ARM_ANGLE = math.degrees(math.atan2(ARM_DIR[1], ARM_DIR[0]))
BOX = (102.0, 56.0)                  # center box x ±51, y ±28
CAM_MOUNT = [(44.0, -9.0), (44.0, 9.0)]   # camera bracket: 2 × M4×8 from below through counterbore, nuts in bracket
TIE_SLOTS = [(sx * 25.0, sy * 23.5) for sx in (-1, 1) for sy in (-1, 1)]   # battery zip tie slots: in center box, between stack and standoffs
TIE_SLOT_WH = (6.0, 3.2)             # width (along X, for zip tie band up to 4.8) × thickness
STACK = [(sx * 15.25, sy * 15.25) for sx in (-1, 1) for sy in (-1, 1)]
A_DIST = 40.0                        # leg mount A: center on arm axis, 40 mm from motor axis
A_SIDE = 8.0                         # two leg bolts: either side of rib, at ±A_SIDE from arm axis
BOSS_W, BOSS_L = 26.0, 34.0          # leg mount: stadium, across × along arm
ARM_W_ROOT, ARM_W_TIP = 24.0, 30.0   # arm width at center box and at motor axis (blends into pad)
ARM_WAIST = (0.5, 16.0, 0.8, 17.0)   # arm waist: (length fraction, width, length fraction, width). 16 = 2 side ribs × 3 + 9.4 channel for 3 × 20 AWG wires
GUSSET = 12.0                        # gusset at arm/center box corner (triangle side, mm)
BOX_CHAMFER = 14.0                   # center box corner chamfer
OUTLINE_FILLET = 3.0                 # convex outline corners
CONCAVE_R = 8.0                      # concave transitions (arm↔center box, arm↔leg mount, arm↔motor pad)
_A = (MOTOR_X - A_DIST * ARM_DIR[0], MOTOR_Y - A_DIST * ARM_DIR[1])
A_PTS = [(sx * _A[0], sy * _A[1]) for sx in (-1, 1) for sy in (-1, 1)]


def arm_width(t):
    """Arm width at length fraction t (linear interpolation between outline stations)."""
    t1, wa, t2, wb = ARM_WAIST
    st = [(0.0, ARM_W_ROOT), (t1, wa), (t2, wb), (1.0, ARM_W_TIP)]
    for (ta, wA), (tb, wB) in zip(st, st[1:]):
        if ta <= t <= tb:
            return wA + (wB - wA) * (t - ta) / (tb - ta)
    return ARM_W_TIP


def _arm_walls(sx, sy):
    """Two side ribs WALL_T × WALL_H along arm edges from center box exit to motor pad (Z from 0)."""
    p0, p1 = _arm_pts(sx, sy)
    dx, dy = sx * ARM_DIR[0], sy * ARM_DIR[1]
    nx, ny = -dy, dx
    ta, tb = 24.0 / _L, 1.0 - 15.0 / _L
    t1, wa, t2, wb = ARM_WAIST
    ts = [ta] + [t for t in (t1, t2) if ta < t < tb] + [tb]
    walls = None
    for side in (-1, 1):
        outer, inner = [], []
        for t in ts:
            cx_, cy_ = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
            ro = arm_width(t) / 2 - WALL_INSET
            ri = ro - WALL_T
            outer.append((cx_ + side * nx * ro, cy_ + side * ny * ro))
            inner.append((cx_ + side * nx * ri, cy_ + side * ny * ri))
        w = cq.Workplane("XY").polyline(outer + inner[::-1]).close().extrude(WALL_H)
        walls = w if walls is None else walls.union(w)
    return walls


def arm_normal(sx, sy):
    """Unit normal to arm axis in quadrant (sx, sy), "left" of the direction to motor."""
    return (-sy * ARM_DIR[1], sx * ARM_DIR[0])


def leg_hole_pts():
    """Leg holes: pairs on both sides of the rib."""
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            ax, ay = sx * _A[0], sy * _A[1]
            nx, ny = arm_normal(sx, sy)
            pts += [(ax + A_SIDE * nx, ay + A_SIDE * ny), (ax - A_SIDE * nx, ay - A_SIDE * ny)]
    return pts


A2_PTS = []                          # (obsolete: second hole along arm replaced by a pair across)
B_PTS = [(sx * 34.0, sy * 18.8) for sx in (-1, 1) for sy in (-1, 1)]     # deck standoffs, same as old plate
HEX_AF = 16.5                        # center window, across flats

FIN_GEOM = dict(x=43.5, t=3.0, w=32.0, h=38.0, cam_z=17.0, lens_dz=8.0)   # w=32: plate corner 70.4 mm from motor, prop r=64.75
# cam_z: center of vertical slots above rib top; lens_dz: lens window above the slots
# (on Arducam the lens is at the top edge of the board, holes closer to the middle)
FIN_SLOT = dict(y=10.75, w=2.6, len=16.0)   # vertical slots: fit both 21 and 22 mm pitch
FIN_WINDOW = (17.0, 20.0)                   # lens window: width × height


def motor_hole_pts():
    """4 × M3 on circle of radius MOTOR_HOLE_R, on X/Y axes (cross); opposite holes 2·R = 16 mm apart."""
    return [(mx + dx, my + dy) for mx, my in MOTORS
            for dx, dy in ((MOTOR_HOLE_R, 0), (-MOTOR_HOLE_R, 0), (0, MOTOR_HOLE_R), (0, -MOTOR_HOLE_R))]


def _band(p0, p1, w, h):
    """Rectangular band of width w along segment p0→p1, height h."""
    (x0, y0), (x1, y1) = p0, p1
    L = math.hypot(x1 - x0, y1 - y0)
    ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
    return (cq.Workplane("XY").rect(L, w).extrude(h)
            .rotate((0, 0, 0), (0, 0, 1), ang).translate(((x0 + x1) / 2, (y0 + y1) / 2, 0)))


def _arm_pts(sx, sy):
    return (sx * ARM_P0[0], sy * ARM_P0[1]), (sx * MOTOR_X, sy * MOTOR_Y)


def _outline():
    """2D plate outline: chamfered center box + tapered arms + leg mount stadiums + motor pads."""
    bw, bh, c = BOX[0] / 2, BOX[1] / 2, BOX_CHAMFER
    box = [(-bw + c, -bh), (bw - c, -bh), (bw, -bh + c), (bw, bh - c), (bw - c, bh), (-bw + c, bh), (-bw, bh - c), (-bw, -bh + c)]
    sk = cq.Sketch().polygon(box + [box[0]])
    for sx in (-1, 1):
        for sy in (-1, 1):
            p0, p1 = _arm_pts(sx, sy)
            dx, dy = sx * ARM_DIR[0], sy * ARM_DIR[1]
            nx, ny = -dy, dx
            t1, wa, t2, wb = ARM_WAIST
            stations = [(0.0, ARM_W_ROOT), (t1, wa), (t2, wb), (1.0, ARM_W_TIP)]
            left, right = [], []
            for t, w in stations:
                cx_, cy_ = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
                left.append((cx_ + nx * w / 2, cy_ + ny * w / 2))
                right.append((cx_ - nx * w / 2, cy_ - ny * w / 2))
            arm = left + right[::-1]
            sk = sk.polygon(arm + [arm[0]])
            ang = math.degrees(math.atan2(dy, dx))
            ax, ay = sx * _A[0], sy * _A[1]
            sk = sk.push([(ax, ay)]).slot(BOSS_L - BOSS_W, BOSS_W, angle=ang).reset()
    sk = sk.push(MOTORS).circle(PAD_D / 2).reset()
    for g in _gussets(box):
        sk = sk.polygon(g + [g[0]])
    return sk.clean()


def _rounded_outline_wire():
    """Morphological rounding of the outline: +R −R (concave corners R) then −r +r (convex r)."""
    face = cq.Workplane("XY").placeSketch(_outline()).extrude(1.0).faces("<Z")   # outline at Z=0
    wire = max(face.wires().vals(), key=lambda w: abs(cq.Face.makeFromWires(w).Area()))
    wp = cq.Workplane("XY").add(wire).toPending()
    wp = wp.offset2D(CONCAVE_R, "arc").offset2D(-CONCAVE_R, "arc")
    wp = wp.offset2D(-OUTLINE_FILLET, "arc").offset2D(OUTLINE_FILLET, "arc")
    return wp


def _seg_intersect(p, d, a, b):
    """Intersection of ray p + t·d (t>0) with segment ab, or None."""
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = d[0] * ey - d[1] * ex
    if abs(den) < 1e-9:
        return None
    t = ((a[0] - p[0]) * ey - (a[1] - p[1]) * ex) / den
    u = ((a[0] - p[0]) * d[1] - (a[1] - p[1]) * d[0]) / den
    if t > 0 and 0 <= u <= 1:
        return (p[0] + t * d[0], p[1] + t * d[1]), (ex, ey), u
    return None


def _gussets(box):
    """Triangular gussets in the corners between arm edges and center box edge."""
    out = []
    edges = [(box[i], box[(i + 1) % len(box)]) for i in range(len(box))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            p0, p1 = _arm_pts(sx, sy)
            d = (sx * ARM_DIR[0], sy * ARM_DIR[1])
            n = (-d[1], d[0])
            for side in (-1, 1):
                e = (p0[0] + side * n[0] * ARM_W_ROOT / 2, p0[1] + side * n[1] * ARM_W_ROOT / 2)   # arm edge
                for a, b in edges:
                    hit = _seg_intersect(e, d, a, b)
                    if hit is None:
                        continue
                    j, (ex, ey), u = hit
                    el = math.hypot(ex, ey); ex, ey = ex / el, ey / el
                    # direction along box edge: away from arm (where normal side points)
                    if ex * n[0] * side + ey * n[1] * side < 0:
                        ex, ey = -ex, -ey
                        room = u * el                       # to edge start
                    else:
                        room = (1 - u) * el                 # to edge end
                    g = min(GUSSET, room)                   # stay inside box corner
                    if g < 3:
                        break
                    out.append([j, (j[0] + d[0] * g, j[1] + d[1] * g), (j[0] + ex * g, j[1] + ey * g)])
                    break
    return out


def plate():
    body = _rounded_outline_wire().extrude(PLATE_T)
    for y in (-22.0, 22.0):                                      # center box lightening
        body = body.cut(cq.Workplane("XY").center(0, y).polygon(6, 11 / math.cos(math.radians(30))).extrude(PLATE_T))
    for x in (-28.0, 28.0):
        body = body.cut(cq.Workplane("XY").center(x, 0).polygon(6, 10 / math.cos(math.radians(30))).extrude(PLATE_T))

    # arm stiffening on top: side ribs along edges (motor wire channel) or old center rib
    ribs = None
    for sx in (-1, 1):
        for sy in (-1, 1):
            if ARM_STIFFENER == "walls":
                r = _arm_walls(sx, sy)
                try:
                    r = r.edges(">Z").fillet(0.8)
                except Exception:
                    pass
            else:
                p0, p1 = _arm_pts(sx, sy)
                a0 = (p0[0] + sx * 24 * ARM_DIR[0], p0[1] + sy * 24 * ARM_DIR[1])      # from center box exit (past B and zip tie slots)
                a1 = (p1[0] - sx * 15 * ARM_DIR[0], p1[1] - sy * 15 * ARM_DIR[1])      # to motor pad
                r = _band(a0, a1, ARM_RIB_W, ARM_RIB_H)
                try:
                    r = r.edges(">Z").fillet(2.0)
                except Exception:
                    pass
            ribs = r if ribs is None else ribs.union(r)
    if BARS:
        for sy in (-1, 1):
            ribs = ribs.union(cq.Workplane("XY").center(0, sy * MOTOR_Y).rect(2 * MOTOR_X - 38, RIB_W).extrude(RIB_H))
    for x, y in B_PTS:                                          # breaks for M4 standoffs (hex 11 across flats)
        ribs = ribs.cut(cq.Workplane("XY").center(x, y).circle(7.5).extrude(ARM_RIB_H))
    if ARM_STIFFENER == "walls":
        for x, y in leg_hole_pts():                             # breaks for leg nylocs (side ribs pass through mount A)
            ribs = ribs.cut(cq.Workplane("XY").center(x, y).circle(LEG_NUT_CLEAR_R).extrude(WALL_H))
    body = body.union(ribs.translate((0, 0, PLATE_T)))

    # holes
    def holes(b, pts, d):
        return b.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(d)
    body = holes(body, motor_hole_pts(), HW.m3_through)
    body = holes(body, MOTORS, MOTOR_CENTER_HOLE)
    if MOTOR_CBORE[1] > 0:
        for x, y in motor_hole_pts():                             # counterbores for motor screw heads (from below)
            body = body.cut(cq.Workplane("XY").center(x, y).circle(MOTOR_CBORE[0] / 2).extrude(MOTOR_CBORE[1]))
    body = holes(body, STACK, HW.m3_through)
    body = holes(body, leg_hole_pts(), HW.m3_through)             # legs: M3x14 + nyloc, plus a zip tie
    for x, y in STACK:                                            # counterbores from below for stack bolt heads
        body = body.cut(cq.Workplane("XY").center(x, y).circle(STACK_CBORE[0] / 2).extrude(STACK_CBORE[1]))
    body = holes(body, B_PTS, HW.m4_through)                     # deck standoffs: M4×8
    body = holes(body, CAM_MOUNT, HW.m4_through)
    for x, y in B_PTS + CAM_MOUNT:                                # counterbore from below for M4 head: screw reaches the nut
        body = body.cut(cq.Workplane("XY").center(x, y).circle(HW.m4_head_d / 2).extrude(HW.m4_cbore))
    body = body.cut(cq.Workplane("XY").polygon(6, HEX_AF / math.cos(math.radians(30))).extrude(PLATE_T + RIB_H))
    for x, y in TIE_SLOTS:                                        # battery zip tie slots
        body = body.cut(cq.Workplane("XY").center(x, y).rect(*TIE_SLOT_WH).extrude(PLATE_T + RIB_H).edges("|Z").fillet(1.0))

    if FIN:
        f = FIN_GEOM
        z0 = PLATE_T + RIB_H
        fin = (cq.Workplane("XY").center(f["x"], 0).rect(f["t"], f["w"]).extrude(f["h"]).translate((0, 0, z0)))
        foot = cq.Workplane("XY").center(f["x"], 0).rect(f["t"] + 6, f["w"]).extrude(RIB_H).translate((0, 0, PLATE_T))
        cz = z0 + f["cam_z"]
        for sy in (-1, 1):                                      # two vertical slots
            fin = fin.cut(cq.Workplane("YZ").center(sy * FIN_SLOT["y"], cz)
                          .slot2D(FIN_SLOT["len"], FIN_SLOT["w"], angle=90).extrude(80, both=True))
        ww, wh = FIN_WINDOW                                     # lens window above the slots
        fin = fin.cut(cq.Workplane("YZ").center(0, cz + f["lens_dz"]).rect(ww, wh).extrude(80, both=True)
                      .edges("|X").fillet(3.0))
        body = body.union(foot).union(fin)
    return body


if __name__ == "__main__":
    from cadquery import exporters
    p = plate()
    out = Path(__file__).parent / "out"; out.mkdir(exist_ok=True)
    exporters.export(p, str(out / "plate_v2.step"))
    exporters.export(p, str(out / "plate_v2.stl"), tolerance=0.02, angularTolerance=0.1)
    bb = p.val().BoundingBox()
    print(f"plate_v2 {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {p.val().Volume()*1.27e-3:.0f} g PETG solid")
