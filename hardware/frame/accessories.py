"""Legs, top deck and GPS mast for the plate (plate_v2.py; compatible with the as-printed plate).
Run: .venv/bin/python accessories.py

Plate coordinates: center = mean of motors, +X = front, Z=0 is the plate bottom (6 mm + ribs 2.4).
"""
import math
from pathlib import Path

import cadquery as cq
from cadquery import exporters

import plate_v2 as P
from params import HW, PI5, GPS, BAT, STACK, MOTOR, PROP, CAM

OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)

# ---------------------------------------------------------------- deck
STACK_LIFT = 4.0                   # stack sits on standoffs/dampers above the plate; battery straps pass underneath
DECK_T = 3.0
DECK_RIB = (3.0, 2.5)              # rib along each rail: width × height
STANDOFF = 35.0                    # standoffs: deck bottom at 41, above the camera plate and the stack (34)
Z_DECK = P.PLATE_T + STANDOFF
DECK_MOUNTS = [(sx * 34.0, sy * 18.8) for sx in (-1, 1) for sy in (-1, 1)]  # all 4 B points outside the prop discs (76 mm from motors)
PI_CENTER = (-10.0, 0.0)           # Pi 5 board: x −52.5…32.5, y ±28; USB/Ethernet ports face rear (−X)
MAST_POS = (-72.0, 0.0)            # GPS mast behind the Pi board (flange 34 -> x −89…−55)
RAIL_W, RAIL_X = 10.0, (-86.0, 40.0)          # longitudinal rails at y = ±18.8 (through the standoff points)
CROSS_W = 12.0                                # cross members at x = ±34
REAR_PAD = (34.0, 40.0)                       # mast pad: x × y, centered at MAST_POS
EAR_D = 9.0                                   # ears for the Pi 5 holes
DECK_TIE_HOLES = [(sx * 34.0, sy * 9.0) for sx in (-1, 1) for sy in (-1, 1)]   # ø3.4 on the cross members: zip ties (BEC under the deck)
DECK_R_CONCAVE, DECK_R_CONVEX = 4.0, 2.0


def _rounded(sk: cq.Sketch, r_in: float, r_out: float, h: float) -> cq.Workplane:
    """Extrude the sketch outline, rounding concave corners r_in and convex r_out (morphologically, like the plate).
    Inner outlines (windows) are kept, their corners rounded r_in."""
    face = cq.Workplane("XY").placeSketch(sk.clean()).extrude(1.0).faces("<Z")
    wires = sorted(face.wires().vals(), key=lambda w: -abs(cq.Face.makeFromWires(w).Area()))
    wp = cq.Workplane("XY").add(wires[0]).toPending()
    wp = wp.offset2D(r_in, "arc").offset2D(-r_in, "arc").offset2D(-r_out, "arc").offset2D(r_out, "arc")
    body = wp.extrude(h)
    for w in wires[1:]:
        try:
            win = cq.Workplane("XY").add(w).toPending().offset2D(-r_in, "arc").offset2D(r_in, "arc").extrude(h)
            body = body.cut(win)
        except Exception:
            body = body.cut(cq.Workplane("XY").add(w).toPending().extrude(h))
    return body


def pi_hole_pts():
    px, py = PI5.hole_pitch
    cx, cy = PI_CENTER
    return [(cx + sx * px / 2, cy + sy * py / 2) for sx in (-1, 1) for sy in (-1, 1)]


def mast_hole_pts():
    mx, my = MAST_POS
    return [(mx + sx * 10, my + sy * 10) for sx in (-1, 1) for sy in (-1, 1)]


def deck():
    """Ladder deck: only what carries the Pi 5 and the GPS mast. ≈ 20 g instead of 54."""
    x0, x1 = RAIL_X
    sk = cq.Sketch()
    for sy in (-1, 1):
        sk = sk.push([((x0 + x1) / 2, sy * 18.8)]).rect(x1 - x0, RAIL_W).reset()
    for bx in (-34.0, 34.0):
        sk = sk.push([(bx, 0)]).rect(CROSS_W, 2 * 18.8 + RAIL_W).reset()
    sk = sk.push([MAST_POS]).rect(*REAR_PAD).reset()
    sk = sk.push(pi_hole_pts()).circle(EAR_D / 2).reset()
    d = _rounded(sk, DECK_R_CONCAVE, DECK_R_CONVEX, DECK_T)

    def holes(b, pts, dia):
        return b.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(dia)
    d = holes(d, DECK_MOUNTS, HW.m4_through)                  # M4×8 from the top into the standoff nut
    d = holes(d, pi_hole_pts(), HW.m25_through)               # Pi 5: M2.5 on 6 mm standoffs
    d = holes(d, mast_hole_pts(), HW.m4_through)              # mast: M4×8 from below
    d = holes(d, [MAST_POS], 10.0)                            # GPS cable down through the mast tube
    d = holes(d, DECK_TIE_HOLES, HW.m3_through)
    rw, rh = DECK_RIB
    for sy in (-1, 1):                                        # rib along the rail, gaps over the standoffs (M4 heads)
        rib = (cq.Workplane("XY").center((x0 + 6 + x1 - 4) / 2, sy * 18.8).rect(x1 - 4 - (x0 + 6), rw).extrude(rh)
               .translate((0, 0, DECK_T)))
        for bx in (-34.0, 34.0):
            rib = rib.cut(cq.Workplane("XY").center(bx, sy * 18.8).circle(5.0).extrude(rh).translate((0, 0, DECK_T)))
        for piece in rib.solids().vals():                     # union one solid at a time, otherwise union loses the base
            d = d.union(cq.Workplane("XY").add(piece))
    return d


# ---------------------------------------------------------------- legs
LEG_H = 72.0                       # plate bottom -> ground (battery 38 + strap + clearance ≥ 30)
LEG_MOUNTS = P.A_PTS               # all 4 A holes (on the arm axis)
LEG_SPLAY = 12.0                   # blade splay outward, deg
LEG_T = 6.0                        # blade thickness (along Y, across the arm)
LEG_W_TOP, LEG_W_BOT = 13.0, 7.0   # blade width along the arm at top / bottom
FOOT = (14.0, 10.0, 6.0)           # foot: pad length × width, flare height (slope ≤ 30° from vertical)


def leg(mirror=False):
    """Local: X along the arm toward the motor, mount A at (0,0), Z=0 is the plate bottom.
    Print flange on the bed (blade up), no supports: no lips, blade tilted ≤ 20°, foot flares ≤ 30°."""
    fl_t = 3.5
    L, W = P.BOSS_L + 1.0, P.BOSS_W + 1.0             # flange repeats the mount oval on the plate (35 × 27)
    fl = cq.Workplane("XY").slot2D(L, W).extrude(fl_t).translate((0, 0, -fl_t))
    for hy in (-P.A_SIDE, P.A_SIDE):
        fl = fl.cut(cq.Workplane("XY").center(0, hy).circle(HW.m3_through / 2).extrude(20, both=True))   # M3 or printed snap pin
    # blade: smooth outward curve (spline), taper 13 -> 7, thickness LEG_T along Y
    xo = L / 2                                        # outer end of the flange: the blade outer face is flush with it
    fw, fd, fh = FOOT
    h = LEG_H - fl_t - fh
    dx = h * math.tan(math.radians(LEG_SPLAY))
    zb = -LEG_H + fh
    z0 = -fl_t
    outer = [(xo, z0), (xo + 0.55 * dx, z0 - 0.5 * h), (xo + dx, zb)]
    inner = [(xo + dx - LEG_W_BOT, zb), (xo + 0.45 * dx - LEG_W_TOP * 0.8, z0 - 0.5 * h), (xo - LEG_W_TOP, z0)]
    prof = (cq.Workplane("XZ").moveTo(*outer[0]).spline(outer[1:], includeCurrent=True)
            .lineTo(*inner[0]).spline(inner[1:], includeCurrent=True).close())
    blade = prof.extrude(LEG_T / 2, both=True)      # profile corners not rounded: hidden in the flange and the foot (else overhang under the foot)
    # foot: flare from the blade section (7 × 6) to the pad fw × fd over height fh (slope ≤ 30°, prints without supports)
    cx = xo + dx - LEG_W_BOT / 2
    foot = (cq.Workplane("XY").workplane(offset=-LEG_H).center(cx, 0).rect(fw, fd)
            .workplane(offset=fh).rect(LEG_W_BOT, LEG_T).loft(combine=True))
    try:
        foot = foot.edges("<Z").fillet(1.5)
    except Exception:
        pass
    return fl.union(blade).union(foot)


def gps_mount():
    """Mast for the round ø50 GPS puck: flange 34×34×6 with M4 nuts in side pockets (M4×8 bolts from below
    through the deck) -> tube -> cup with a lip. Print standing, flange down."""
    FL_T = GPS_FL_T
    base = cq.Workplane("XY").rect(34, 34).extrude(FL_T).edges("|Z").fillet(4)
    base = base.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(
        [(sx * 10, sy * 10) for sx in (-1, 1) for sy in (-1, 1)]).hole(HW.m4_through)
    for sx in (-1, 1):
        for sy in (-1, 1):                                       # nut pocket: entry from the nearest side face (along Y)
            base = base.cut(_nut_pocket(1.2, angle=90 if sy > 0 else -90).translate((sx * 10, sy * 10, 0)))
    tube = cq.Workplane("XY").circle(6.0).circle(4.3).extrude(GPS.mast_h).translate((0, 0, FL_T))   # wall 1.7 mm
    cone = (cq.Workplane("XY").workplane(offset=FL_T + GPS.mast_h - 8).circle(6.0)
            .workplane(offset=8).circle(14).loft(combine=True))
    cone = cone.cut(cq.Workplane("XY").workplane(offset=FL_T + GPS.mast_h - 8).circle(4.3)                # hollow cone
                    .workplane(offset=8).circle(11.5).loft(combine=True))
    top_z = FL_T + GPS.mast_h
    cup = cq.Workplane("XY").circle(GPS.puck_d / 2 + 2.5).circle(GPS.puck_d / 2 - 7).extrude(2.0)   # ring 9.5 mm
    for a in (0, 90):                                                                        # two spokes crosswise
        cup = cup.union(cq.Workplane("XY").rect(GPS.puck_d, 8).extrude(2.0).rotate((0, 0, 0), (0, 0, 1), a))
    cup = cup.translate((0, 0, top_z))
    lip = (cq.Workplane("XY").circle(GPS.puck_d / 2 + 2.5).circle(GPS.puck_d / 2 + 0.5).extrude(2.5)
           .translate((0, 0, top_z + 2.0)))
    m = base.union(tube).union(cone).union(cup).union(lip)
    # cable notch in the lip, zip tie slots, wire through the center
    m = m.cut(cq.Workplane("XY").center(GPS.puck_d / 2 + 1, 0).rect(6, 10).extrude(10).translate((0, 0, top_z + 1)))
    for a in (45, 135, 225, 315):
        r = GPS.puck_d / 2 - 7
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        m = m.cut(cq.Workplane("XY").rect(4, 8).extrude(6).rotate((0, 0, 0), (0, 0, 1), a)
                  .translate((x, y, top_z - 1)))
    m = m.faces(">Z").workplane(origin=(0, 0, 0)).hole(8.0)
    return m


# ---------------------------------------------------------------- printed standoff
GPS_FL_T = 6.0                     # GPS mast flange (M4 nuts in pockets)
STANDOFF_NUTS = False              # False: M4 threads straight into a pilot hole (simple, light). True: nut pockets at both ends
STANDOFF_AF = 9.0 if not STANDOFF_NUTS else 11.0   # hex across flats: 9 mm leaves 2.7 mm wall around the ø3.5 pilot
STANDOFF_PILOT = 3.5               # M4 thread-forming pilot in PETG (minor dia 3.24)
NUT_AF, NUT_H = HW.m4_nut_af, HW.m4_nut_h   # M4 nut pocket with side entry (flanges: pocket height along Z, prints true)
STANDOFF_NUT_AF, STANDOFF_NUT_H = 7.5, 3.9  # standoff prints lying down: pocket height is an XY slot, FDM shrinks it ~0.4


def _nut_pocket(z0, angle=0.0, af=NUT_AF, h=NUT_H):
    """M4 nut pocket with side entry, axis Z, entry along +X (rotated by angle)."""
    pocket = cq.Workplane("XY").polygon(6, af / math.cos(math.radians(30))).extrude(h)
    slot = cq.Workplane("XY").center(10, 0).rect(20, af).extrude(h)
    return pocket.union(slot).rotate((0, 0, 0), (0, 0, 1), angle).translate((0, 0, z0))


def standoff():
    """Hex standoff 35 mm for M4x8 from both ends.
    Default: ø3.5 pilot through, the M4 cuts its own thread (4.5 mm from the plate side, 5 mm from the deck side).
    STANDOFF_NUTS=True: ø4.4 bore + M4 nuts in side-entry pockets.
    A hex face looks at +X. Print LYING on the opposite face (export_print lays it down), layers along the standoff."""
    d = STANDOFF_AF / math.cos(math.radians(30))
    st = cq.Workplane("XY").polygon(6, d).extrude(STANDOFF).rotate((0, 0, 0), (0, 0, 1), 30)   # faces at ±X
    if STANDOFF_NUTS:
        st = st.faces(">Z").workplane(origin=(0, 0, 0)).hole(HW.m4_through)
        for z0 in (1.5, STANDOFF - 1.5 - STANDOFF_NUT_H):        # pockets 1.5 mm from each end
            st = st.cut(_nut_pocket(z0, af=STANDOFF_NUT_AF, h=STANDOFF_NUT_H))
    else:
        st = st.faces(">Z").workplane(origin=(0, 0, 0)).hole(STANDOFF_PILOT)
        st = st.edges("|Z").chamfer(0.6)                          # take the sharp hex corners off
    return st


def standoff_for_print():
    """Standoff on its side: axis along X, pocket entries up (+Z), bottom hex face on the bed."""
    return standoff().rotate((0, 0, 0), (0, 1, 0), -90)


# ---------------------------------------------------------------- camera bracket (swappable)
CAM_FL = dict(x0=38.0, x1=58.5, w=25.0, t=2.0, w_front=30.0, x_wide=40.0)   # flat 2 mm flange: x 38…58.5 (plate edge is 51), width 25 at the rear past standoffs B, 30 for x >= 40
CAM_NUT_CLEAR = 6.0                                                           # radius around each M4 nut kept free of anything tall (10 mm socket)
# camera cradle for Arducam B0392 (board 25 × 24, thickness ≈ 1.0, M12 lens in a ~14 × 14 block at the top edge, ribbon exits upward,
# FPC connector on the back side at the top edge). Board slides in from the TOP into a groove between the front and back walls and is held
# by its side edges; lens exits through a U-shaped window in front, connector through a rear relief. Board holes are not used.
CAM_PCB = dict(w=25.0, h=24.0, t=1.0)
CAM_CRADLE = dict(
    tilt_deg=15.0,        # board tilt forward-down (looking below the horizon)
    groove_t=1.5,         # groove width for the board (PCB 1.0 + FDM tolerance)
    groove_d=1.5,         # groove depth in the side pillars (edge grip)
    wall=1.6,             # back wall (board rests on it) and front groove lips
    pillar=2.4,           # side pillar
    h=20.0,               # pillar height (top 4 mm of the board free for ribbon/connector)
    front_band=0.0,       # front fully open from the groove bottom (lens, M2 screw heads)
    back_band=16.0,       # back wall from the bottom up to the FPC connector relief (connector in the top ~6 mm of the board)
    back_relief_w=22.0,   # rear relief width for the connector
    holes=(21.0, 12.5),   # board hole pattern: same as Pi Camera v2 (Arducam is a drop-in replacement); measured 22 × 11 ± error
    hole_z0=2.0,          # bottom hole row: 2 mm from the board bottom edge
    boss_d=6.0,           # bosses on the back wall for M2 screws (ø6: margin for re-drilling ±0.7 mm)
    boss_h=3.0,           # boss height behind the wall -> thread in plastic 1.6 + 3.0 = 4.6 mm
    pilot=1.7,            # pilot hole for M2 (screw taps its own thread)
)
CAM_X0 = 53.0                         # bottom rear edge of the cassette: bosses start at 50, clear of the 10 mm socket on the nuts at x=44
_CAM_T = CAM_CRADLE["wall"] * 2 + CAM_CRADLE["groove_t"]
CAM_Z0 = CAM_FL["t"] + _CAM_T * math.sin(math.radians(CAM_CRADLE["tilt_deg"]))   # lift: groove bottom not below the flange top when tilted


def camera_mount():
    """Camera cradle: flat 2 mm flange on the plate (2 × M4×8 from below through the plate counterbores, plain nuts on top, socket access) +
    tilted cassette with a board groove. Local: plate coordinates (X forward), Z=0 is the plate top.
    Print flange down, no supports (tilt 15°, windows open upward). Flange bottom is flat."""
    f, c, pcb = CAM_FL, CAM_CRADLE, CAM_PCB
    fl = (cq.Workplane("XY").center((f["x0"] + f["x1"]) / 2, 0).rect(f["x1"] - f["x0"], f["w"]).extrude(f["t"])
          .edges("|Z").fillet(2.0))
    wide = (cq.Workplane("XY").center((f["x_wide"] + f["x1"]) / 2, 0).rect(f["x1"] - f["x_wide"], f["w_front"]).extrude(f["t"])
            .edges("|Z").edges(">X").fillet(2.0))                                       # round only the front corners: the rear ones carry the cassette
    fl = fl.union(wide)
    for x, y in P.CAM_MOUNT:
        fl = fl.cut(cq.Workplane("XY").center(x, y).circle(HW.m4_through / 2).extrude(f["t"]))

    # cassette in local axes: X forward (toward the drone nose), Y width, Z along the board (up).
    # x ∈ [0, wall] is the BACK wall (board rests on it; carries the M2 bosses and the FPC connector relief);
    # x ∈ [wall, wall+groove_t] is the board; x ∈ [wall+groove_t, T] is the front: open, only the groove lips in the pillars remain.
    W = pcb["w"] + 2 * c["pillar"]
    T = _CAM_T
    H = c["h"]
    ext = 8.0                                               # margin downward so that after tilting it cuts into the flange
    box = cq.Workplane("XY").center(T / 2, 0).rect(T, W).extrude(H + ext).translate((0, 0, -ext))
    slot = (cq.Workplane("XY").center(c["wall"] + c["groove_t"] / 2, 0).rect(c["groove_t"], pcb["w"] + 0.4)
            .extrude(H + 1))                                                            # groove, bottom = board stop (z = 0)
    box = box.cut(slot)
    front_open = (cq.Workplane("XY").center(T - c["wall"] / 2, 0).rect(c["wall"] + 0.2, pcb["w"] + 0.4)
                  .extrude(H + 1).translate((0.1, 0, c["front_band"])))                # front wall removed over the board width
    box = box.cut(front_open)
    relief = (cq.Workplane("XY").center(c["wall"] / 2, 0).rect(c["wall"] + 0.2, c["back_relief_w"])
              .extrude(H + 1).translate((-0.1, 0, c["back_band"])))                    # rear relief for the FPC connector
    box = box.cut(relief)
    hx, hz = c["holes"]
    hole_pts = [(sy * hx / 2, c["hole_z0"] + k * hz) for sy in (-1, 1) for k in (0, 1)]
    for hy, hzz in hole_pts:                                                           # bosses BEHIND the back wall (x < 0) + pilot holes
        boss = cq.Workplane("YZ").center(hy, hzz).circle(c["boss_d"] / 2).extrude(-c["boss_h"])
        box = box.union(boss)
        box = box.cut(cq.Workplane("YZ").center(hy, hzz).circle(c["pilot"] / 2)
                      .extrude(c["boss_h"] + c["wall"] + 0.2).translate((-c["boss_h"] - 0.1, 0, 0)))   # pilot through the boss and the wall
    # tilt forward and seat on the flange: bottom rear edge of the cassette at (CAM_X0, 0, CAM_Z0)
    box = box.rotate((0, 0, 0), (0, 1, 0), c["tilt_deg"]).translate((CAM_X0, 0, CAM_Z0))
    box = box.cut(cq.Workplane("XY").rect(200, 200).extrude(f["t"] + 60).translate((0, 0, -60)))       # remove everything below the flange top
    body = fl.union(box)
    return body


def camera_board_pose():
    """Camera board center and optical axis direction in plate coordinates (for the assembly and the simulator)."""
    c, pcb = CAM_CRADLE, CAM_PCB
    t = math.radians(c["tilt_deg"])
    lx, lz = c["wall"] + c["groove_t"] / 2, pcb["h"] / 2
    x = CAM_X0 + lx * math.cos(t) + lz * math.sin(t)
    z = CAM_Z0 - lx * math.sin(t) + lz * math.cos(t)
    return (x, 0.0, z), (math.cos(t), 0.0, -math.sin(t))


# ---------------------------------------------------------------- assembly
def assembly():
    asm = cq.Assembly(name="drone_v2")
    asm.add(P.plate(), name="plate", color=cq.Color(0.55, 0.57, 0.6))
    dk = deck()
    asm.add(dk, name="deck", loc=cq.Location(cq.Vector(0, 0, Z_DECK)), color=cq.Color(0.27, 0.51, 0.71))
    lg = leg()
    for i, (x, y) in enumerate(LEG_MOUNTS):
        sx, sy = (1 if x > 0 else -1), (1 if y > 0 else -1)
        ang = math.degrees(math.atan2(sy * P.ARM_DIR[1], sx * P.ARM_DIR[0]))
        asm.add(lg, name=f"leg{i}", loc=cq.Location(cq.Vector(x, y, 0), cq.Vector(0, 0, 1), ang),
                color=cq.Color(1.0, 0.55, 0.0))
    cm = camera_mount()
    asm.add(cm, name="camera_mount", loc=cq.Location(cq.Vector(0, 0, P.PLATE_T)), color=cq.Color(0.8, 0.25, 0.2))
    gm = gps_mount()
    asm.add(gm, name="gps_mount", loc=cq.Location(cq.Vector(MAST_POS[0], MAST_POS[1], Z_DECK + DECK_T)),
            color=cq.Color(0.85, 0.65, 0.1))
    so = standoff()
    for i, (x, y) in enumerate(DECK_MOUNTS):
        ang = 180 if x < 0 else 0                                # pocket side entry faces outward, toward the deck edge
        asm.add(so, name=f"standoff{i}", loc=cq.Location(cq.Vector(x, y, P.PLATE_T), cq.Vector(0, 0, 1), ang),
                color=cq.Color(0.75, 0.75, 0.78))

    ref = cq.Assembly(name="reference")
    stack = cq.Workplane("XY").rect(STACK.esc_lwh[0], STACK.esc_lwh[1]).extrude(STACK.height_assembled)
    ref.add(stack, name="stack", loc=cq.Location(cq.Vector(0, 0, P.PLATE_T + STACK_LIFT)), color=cq.Color(0.1, 0.1, 0.1))
    pi = cq.Workplane("XY").rect(*PI5.board).extrude(1.5)
    ref.add(pi, name="pi5", loc=cq.Location(cq.Vector(PI_CENTER[0], PI_CENTER[1], Z_DECK + DECK_T + PI5.standoff_h)),
            color=cq.Color(0.1, 0.6, 0.1))
    bl, bw, bh = BAT.lwh
    ref.add(cq.Workplane("XY").rect(bl, bw).extrude(bh), name="battery",
            loc=cq.Location(cq.Vector(0, 0, -bh - 2)), color=cq.Color(0, 0, 0.5))
    motor = cq.Workplane("XY").circle(MOTOR.bell_d / 2).extrude(MOTOR.body_h)
    prop = cq.Workplane("XY").circle(PROP.diameter / 2).extrude(1.0)
    for i, (x, y) in enumerate(P.MOTORS):
        ref.add(motor, name=f"motor{i}", loc=cq.Location(cq.Vector(x, y, P.PLATE_T)), color=cq.Color(0, 0.5, 0.5))
        ref.add(prop, name=f"prop{i}", loc=cq.Location(cq.Vector(x, y, P.PLATE_T + MOTOR.body_h + 3)),
                color=cq.Color(0.3, 0.3, 0.3, 0.3))
    gps = cq.Workplane("XY").circle(GPS.puck_d / 2).extrude(GPS.puck_h)
    ref.add(gps, name="gps", loc=cq.Location(cq.Vector(MAST_POS[0], MAST_POS[1], Z_DECK + DECK_T + GPS_FL_T + GPS.mast_h + 2)),
            color=cq.Color(0.95, 0.95, 0.95))
    (cx, cy, cz), _ = camera_board_pose()
    tilt = CAM_CRADLE["tilt_deg"]
    lens_off = 7.0                                                                     # lens center above the board center
    camb = (cq.Workplane("YZ").rect(CAM.board[0], CAM.board[1]).extrude(CAM_PCB["t"] / 2, both=True)
            .union(cq.Workplane("YZ").center(0, lens_off).circle(CAM.lens_d / 2).extrude(CAM.lens_h)
                   .translate((CAM_PCB["t"] / 2, 0, 0)))                               # lens from the board front face
            .rotate((0, 0, 0), (0, 1, 0), tilt))
    ref.add(camb, name="camera", loc=cq.Location(cq.Vector(cx, cy, P.PLATE_T + cz)), color=cq.Color(0.78, 0.2, 0.17))
    asm.add(ref, name="reference")
    return asm, dict(deck=dk, leg=lg, gps_mount=gm, standoff=so, camera_mount=cm)


def _walk(asm, parent_loc=None):
    loc = asm.loc if parent_loc is None else parent_loc * asm.loc
    if asm.obj is not None:
        yield asm.name, asm.obj.val() if hasattr(asm.obj, "val") else asm.obj, loc
    for ch in asm.children:
        yield from _walk(ch, loc)


def interference(asm):
    solids = [(n, s.moved(l)) for n, s, l in _walk(asm)]
    bad = []
    for i in range(len(solids)):
        for j in range(i + 1, len(solids)):
            n1, s1 = solids[i]; n2, s2 = solids[j]
            if n1.startswith("prop") or n2.startswith("prop"):
                continue
            try:
                v = s1.intersect(s2).Volume()
            except Exception:
                v = -1
            if v > 1.0:
                bad.append((n1, n2, round(v, 1)))
    return bad


def main():
    asm, parts = assembly()
    print("interference:", interference(asm) or "none")
    rho = 1.27e-3
    counts = {"leg": 4, "standoff": 4}
    for name, shape in parts.items():
        bb = shape.val().BoundingBox()
        print(f"{name:10s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f}  {shape.val().Volume()*rho:6.1f} g x{counts.get(name, 1)}")
        exporters.export(shape, str(OUT / f"acc_{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        exporters.export(shape, str(OUT / f"acc_{name}.step"))
    asm.save(str(OUT / "drone_v2.step"))
    print("Z: plate 0–6 (+ribs 12), stack 6–34, camera cradle up to", round(P.PLATE_T + CAM_FL["t"] + CAM_CRADLE["h"], 1),
          "deck", Z_DECK, "–", Z_DECK + DECK_T, "; ground", -LEG_H, "; battery bottom", -BAT.lwh[2] - 2)


if __name__ == "__main__":
    main()
