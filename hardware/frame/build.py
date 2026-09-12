"""Frame v1 generator (CadQuery). Run: .venv/bin/python build.py

Coordinate system: frame center at (0,0); Z up; Z=0 is the bottom face
of the base bottom plate (A). Front is +X. Arms on the diagonals (45°).

Vertical stack-up:
  ground ─ legs ─ battery ─ tray(H) ─ standoffs ─ A ─ arms ─ B ─ stack ─ standoffs ─ E(Pi) ─ GPS mast
"""

import math
from pathlib import Path

import cadquery as cq
from cadquery import exporters

from params import PRINT, HW, MOTOR, PROP, STACK, PI5, CAM, GPS, BAT, FRAME

OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)

R_MOTOR = FRAME.wheelbase / 2                     # 120
DIAG = math.sqrt(0.5)
ARM_ANGLES = (45, 135, 225, 315)

# --- vertical levels (mm from bottom of plate A)
Z_A = 0.0
Z_ARM = Z_A + 6.0                                 # A = 6 mm (heat-set inserts)
Z_B = Z_ARM + FRAME.arm_t                         # 16
Z_STACK = Z_B + FRAME.top_plate_t                 # 20
Z_E = Z_STACK + FRAME.deck_gap                    # 48
Z_TRAY = Z_A - FRAME.tray_gap - FRAME.deck_t      # -17.5
Z_GROUND = Z_ARM - FRAME.leg_h                    # -89

PLATE_A_T = 6.0
PLATE_W = 112.0
ARM_CLAMP_R = (40.0, 54.0, 68.0)                  # arm clamp bolts along arm axis
ARM_R_IN = 30.0
ARM_THIN_FROM = R_MOTOR - 17.0                    # from this radius arm is 5 mm (motor pad)
LEG_R = (86.0, 100.0)                              # heat-set inserts for leg on arm
DECK_MOUNTS = [(0, 40), (0, -40), (40, 0), (-40, 0)]   # deck standoffs (B→E) and tray (A→H)
GRID_PTS = [(0, 20), (0, -20), (20, 0), (-20, 0)]      # free grid (not under arms)
TRAY_MOUNTS = [(sx * 40, sy * 20) for sx in (-1, 1) for sy in (-1, 1)]  # heat-set inserts in A from below
STACK_PTS = [(sx * STACK.pitch / 2, sy * STACK.pitch / 2) for sx in (-1, 1) for sy in (-1, 1)]


def diag_pt(r, ang):
    return (r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang)))


def holes(wp, pts, d):
    return wp.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(d)


# ------------------------------------------------------------------ arm
def arm():
    """Arm along +X (local), bottom at Z=0. Rotated to its angle afterwards."""
    root_w, tip_w = FRAME.arm_root_w, FRAME.arm_w
    x0, x1 = ARM_R_IN, ARM_THIN_FROM
    body = (cq.Workplane("XY")
            .polyline([(x0, -root_w / 2), (x1, -tip_w / 2), (R_MOTOR, -tip_w / 2),
                       (R_MOTOR, tip_w / 2), (x1, tip_w / 2), (x0, root_w / 2)]).close()
            .extrude(FRAME.arm_t))
    pad = cq.Workplane("XY").center(R_MOTOR, 0).circle(MOTOR.pad_d / 2).extrude(FRAME.arm_t)
    a = body.union(pad)
    # motor pad: thinned from below to pad_thickness
    cut_h = FRAME.arm_t - MOTOR.pad_thickness
    thin = (cq.Workplane("XY").center((ARM_THIN_FROM + R_MOTOR + 20) / 2, 0)
            .rect(R_MOTOR + 40 - ARM_THIN_FROM, MOTOR.pad_d + 4).extrude(cut_h))
    a = a.cut(thin)
    # holes
    p = MOTOR.hole_pitch / 2
    motor_pts = [(R_MOTOR + sx * p, sy * p) for sx in (-1, 1) for sy in (-1, 1)]
    a = holes(a, motor_pts, HW.m3_through)
    a = holes(a, [(R_MOTOR, 0)], MOTOR.pad_center_hole)
    a = holes(a, [(r, 0) for r in ARM_CLAMP_R], HW.m3_through)
    a = holes(a, [(r, 0) for r in LEG_R], HW.m3_insert)
    # motor wire groove (beside the bolt axis)
    groove = (cq.Workplane("XY").center((45 + R_MOTOR - 8) / 2, 9.0)
              .rect(R_MOTOR - 8 - 45, 5.0).extrude(3.0)
              .translate((0, 0, FRAME.arm_t - 3.0)))
    a = a.cut(groove)
    a = a.edges("|Z").fillet(1.5)
    return a


# ------------------------------------------------------------------ base plates
def plate_base(t):
    p = cq.Workplane("XY").rect(PLATE_W, PLATE_W).extrude(t).edges("|Z").fillet(8)
    p = holes(p, [(0, 0)], 14.0)
    for ang in ARM_ANGLES:
        p = holes(p, [diag_pt(r, ang) for r in ARM_CLAMP_R], HW.m3_through)
    p = holes(p, STACK_PTS, HW.m3_through)
    p = holes(p, GRID_PTS, HW.m3_through)
    p = holes(p, DECK_MOUNTS, HW.m3_through)
    return p


def plate_A():
    p = plate_base(PLATE_A_T)
    return holes(p, TRAY_MOUNTS, HW.m3_insert)   # tray mounts from below into heat-set inserts


def plate_B():
    return plate_base(FRAME.top_plate_t)


# ------------------------------------------------------------------ Pi 5 deck
DECK_L, DECK_W = 130.0, 90.0
CAM_X = 56.0
MAST_X = -54.0


def deck_E():
    d = cq.Workplane("XY").rect(DECK_L, DECK_W).extrude(FRAME.deck_t).edges("|Z").fillet(8)
    d = holes(d, DECK_MOUNTS, HW.m3_through)
    px, py = PI5.hole_pitch
    d = holes(d, [(sx * px / 2, sy * py / 2) for sx in (-1, 1) for sy in (-1, 1)], HW.m25_through)
    d = holes(d, [(CAM_X, 10), (CAM_X, -10)], HW.m3_through)
    d = holes(d, [(MAST_X + sx * 10, sy * 10) for sx in (-1, 1) for sy in (-1, 1)], HW.m3_through)
    d = holes(d, [(0, 0)], 16.0)                       # wires to stack
    d = holes(d, [(x, y) for x in (-20, 20) for y in (-20, 20)], HW.m3_through)  # spare grid
    return d


# ------------------------------------------------------------------ battery tray
TRAY_L, TRAY_W = 150.0, 60.0


def tray_H():
    t = cq.Workplane("XY").rect(TRAY_L, TRAY_W).extrude(FRAME.deck_t).edges("|Z").fillet(6)
    t = holes(t, TRAY_MOUNTS, HW.m3_through)
    t = holes(t, [(0, 0)], 14.0)
    for x in (-45, 45):
        for y in (-26, 26):
            t = t.cut(cq.Workplane("XY").center(x, y).rect(BAT.strap_w + 2, 4).extrude(FRAME.deck_t))
    return t


# ------------------------------------------------------------------ leg
LEG_T, LEG_W, FOOT_L = 5.0, 24.0, 25.0


def leg_I():
    """Local: flange along X (arm axis), flange bottom at Z=0, leg down."""
    flange_l = 28.0
    fl = cq.Workplane("XY").rect(flange_l, LEG_W).extrude(LEG_T).translate((0, 0, -LEG_T))
    fl = fl.faces("<Z").workplane().pushPoints([(-7, 0), (7, 0)]).hole(HW.m3_through)
    h = FRAME.leg_h
    post = (cq.Workplane("XY").rect(LEG_T, LEG_W).extrude(h)
            .translate((flange_l / 2 - LEG_T / 2, 0, -h)))
    foot = (cq.Workplane("XY").rect(FOOT_L, LEG_W).extrude(LEG_T)
            .translate((flange_l / 2 + FOOT_L / 2 - LEG_T, 0, -h)))
    return fl.union(post).union(foot)


# ------------------------------------------------------------------ camera bracket
def camera_F():
    """Flange on deck + tilted plate. Local: flange center at (0,0), deck Z=0."""
    fl = cq.Workplane("XY").rect(24, 30).extrude(FRAME.deck_t)
    fl = holes(fl, [(0, 10), (0, -10)], HW.m3_through)
    face_h, face_w, face_t = 34.0, 32.0, 3.0
    face = cq.Workplane("XZ").rect(face_t, face_h).extrude(face_w / 2, both=True)
    face = face.translate((0, 0, face_h / 2))
    hx, hy = CAM.hole_pitch
    cam_c_z = 8.0 + hy / 2                              # pattern center above plate bottom
    pts = [(sy * hx / 2, cam_c_z + sz * hy / 2) for sy in (-1, 1) for sz in (-1, 1)]
    face = (face.faces(">X").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0))
            .pushPoints(pts).hole(HW.m2_through))
    face = (face.faces(">X").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0))
            .pushPoints([(0, cam_c_z)]).hole(CAM.lens_d + 2))
    face = face.rotate((0, 0, 0), (0, 1, 0), CAM.tilt_deg).translate((6, 0, FRAME.deck_t))
    gusset = (cq.Workplane("XY").polyline([(-8, 0), (8, 0), (8, 12)]).close()
              .extrude(3, both=True).translate((0, 0, FRAME.deck_t)))
    return fl.union(face).union(gusset)


# ------------------------------------------------------------------ GPS mast
def mast_G():
    base = cq.Workplane("XY").rect(30, 30).extrude(3).edges("|Z").fillet(3)
    base = holes(base, [(sx * 10, sy * 10) for sx in (-1, 1) for sy in (-1, 1)], HW.m3_through)
    tube = cq.Workplane("XY").circle(6).circle(4).extrude(GPS.mast_h).translate((0, 0, 3))
    top = cq.Workplane("XY").rect(GPS.mast_pad, GPS.mast_pad).extrude(3).translate((0, 0, 3 + GPS.mast_h))
    top = holes(top, [(0, 0)], 8.0)
    top = top.cut(cq.Workplane("XY").center(0, 10).rect(4, 3).extrude(50))
    top = top.cut(cq.Workplane("XY").center(0, -10).rect(4, 3).extrude(50))
    return base.union(tube).union(top)


# ------------------------------------------------------------------ assembly
def assembly():
    asm = cq.Assembly(name="frame_v1")
    parts = {}
    parts["arm"] = arm()
    for i, ang in enumerate(ARM_ANGLES):
        asm.add(parts["arm"], name=f"arm{i}",
                loc=cq.Location(cq.Vector(0, 0, Z_ARM), cq.Vector(0, 0, 1), ang), color=cq.Color(0.6, 0.6, 0.6))
    parts["plate_A"] = plate_A()
    asm.add(parts["plate_A"], name="plate_A", loc=cq.Location(cq.Vector(0, 0, Z_A)), color=cq.Color(0.4, 0.4, 0.4))
    parts["plate_B"] = plate_B()
    asm.add(parts["plate_B"], name="plate_B", loc=cq.Location(cq.Vector(0, 0, Z_B)), color=cq.Color(0.4, 0.4, 0.4))
    parts["deck_E"] = deck_E()
    asm.add(parts["deck_E"], name="deck_E", loc=cq.Location(cq.Vector(0, 0, Z_E)), color=cq.Color(0.27, 0.51, 0.71))
    parts["tray_H"] = tray_H()
    asm.add(parts["tray_H"], name="tray_H", loc=cq.Location(cq.Vector(0, 0, Z_TRAY)), color=cq.Color(0.33, 0.42, 0.18))
    parts["leg_I"] = leg_I()
    leg_rc = sum(LEG_R) / 2
    for i, ang in enumerate(ARM_ANGLES):
        x, y = diag_pt(leg_rc, ang)
        asm.add(parts["leg_I"], name=f"leg{i}",
                loc=cq.Location(cq.Vector(x, y, Z_ARM), cq.Vector(0, 0, 1), ang), color=cq.Color(1.0, 0.55, 0.0))
    parts["camera_F"] = camera_F()
    asm.add(parts["camera_F"], name="camera_F", loc=cq.Location(cq.Vector(CAM_X, 0, Z_E + FRAME.deck_t)), color=cq.Color(0.85, 0.1, 0.1))
    parts["mast_G"] = mast_G()
    asm.add(parts["mast_G"], name="mast_G", loc=cq.Location(cq.Vector(MAST_X, 0, Z_E + FRAME.deck_t)), color=cq.Color(0.85, 0.65, 0.1))

    # reference bodies (not printed): motors, props, stack, Pi, battery
    ref = cq.Assembly(name="reference")
    motor = cq.Workplane("XY").circle(MOTOR.bell_d / 2).extrude(MOTOR.body_h)
    prop = cq.Workplane("XY").circle(PROP.diameter / 2).extrude(1.0)
    for i, ang in enumerate(ARM_ANGLES):
        x, y = diag_pt(R_MOTOR, ang)
        ref.add(motor, name=f"motor{i}", loc=cq.Location(cq.Vector(x, y, Z_B)), color=cq.Color(0.0, 0.5, 0.5))
        ref.add(prop, name=f"prop{i}", loc=cq.Location(cq.Vector(x, y, Z_B + MOTOR.body_h + 3)),
                color=cq.Color(0.3, 0.3, 0.3, 0.3))
    stack = cq.Workplane("XY").rect(STACK.esc_lwh[0], STACK.esc_lwh[1]).extrude(STACK.height_assembled)
    ref.add(stack, name="stack", loc=cq.Location(cq.Vector(0, 0, Z_STACK)), color=cq.Color(0.1, 0.1, 0.1))
    pi = cq.Workplane("XY").rect(*PI5.board).extrude(1.5)
    ref.add(pi, name="pi5", loc=cq.Location(cq.Vector(0, 0, Z_E + FRAME.deck_t + PI5.standoff_h)), color=cq.Color(0.1, 0.6, 0.1))
    bl, bw, bh = BAT.lwh
    bat = cq.Workplane("XY").rect(bl, bw).extrude(bh)
    ref.add(bat, name="battery", loc=cq.Location(cq.Vector(0, 0, Z_TRAY - bh)), color=cq.Color(0.0, 0.0, 0.5))
    gps = cq.Workplane("XY").circle(GPS.puck_d / 2).extrude(GPS.puck_h)
    ref.add(gps, name="gps", loc=cq.Location(cq.Vector(MAST_X, 0, Z_E + FRAME.deck_t + 3 + GPS.mast_h + 3)), color=cq.Color(0.95, 0.95, 0.95))
    asm.add(ref, name="reference")
    return asm, parts


def interference(asm):
    """Intersection volumes between all assembly bodies (printed + reference)."""
    solids = []
    for name, shape, loc in _walk(asm):
        solids.append((name, shape.moved(loc)))
    bad = []
    for i in range(len(solids)):
        for j in range(i + 1, len(solids)):
            n1, s1 = solids[i]
            n2, s2 = solids[j]
            if n1.startswith("prop") or n2.startswith("prop"):
                continue  # prop discs: visual only
            try:
                v = s1.intersect(s2).Volume()
            except Exception:
                v = -1
            if v > 1.0:
                bad.append((n1, n2, round(v, 1)))
    return bad


def _walk(asm, parent_loc=None):
    loc = asm.loc if parent_loc is None else parent_loc * asm.loc
    if asm.obj is not None:
        yield asm.name, asm.obj.val() if hasattr(asm.obj, "val") else asm.obj, loc
    for ch in asm.children:
        yield from _walk(ch, loc)


def main():
    asm, parts = assembly()
    bad = interference(asm)
    print("interferences:", bad if bad else "none")
    rho = 1.27e-3  # g/mm³ PETG (100 % infill, upper estimate)
    total = 0.0
    counts = {"arm": 4, "leg_I": 4}
    print(f"{'part':10s} {'bbox XYZ, mm':>28s} {'mass, g':>9s}")
    for name, shape in parts.items():
        bb = shape.val().BoundingBox()
        m = shape.val().Volume() * rho
        n = counts.get(name, 1)
        total += m * n
        print(f"{name:10s} {bb.xlen:8.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f}   {m:7.1f} x{n}")
        exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        exporters.export(shape, str(OUT / f"{name}.step"))
        assert bb.xlen <= PRINT.bed_xy and bb.ylen <= PRINT.bed_xy, f"{name} does not fit on print bed"
    print(f"frame total (solid, no hardware): {total:.0f} g")
    asm.save(str(OUT / "frame_v1_assembly.step"))
    asm.save(str(OUT / "frame_v1_assembly.glb"))
    # SVG views
    printed = cq.Assembly()
    for ch in asm.children:
        if ch.name != "reference":
            printed.add(ch.obj, name=ch.name, loc=ch.loc)
    comp = printed.toCompound()
    for view, d in {"iso": (1, -1.2, 0.8), "top": (0, 0, 1), "front": (1, 0, 0), "side": (0, 1, 0)}.items():
        exporters.export(comp, str(OUT / f"view_{view}.svg"),
                         opt={"projectionDir": d, "width": 1400, "height": 900, "showHidden": False,
                              "strokeWidth": 0.6})
    print("Z levels:", dict(A=Z_A, arm=Z_ARM, B=Z_B, stack=Z_STACK, E=Z_E, tray=Z_TRAY, ground=Z_GROUND))


if __name__ == "__main__":
    main()
