"""Frame v3 layout: all dimensions and the 2D footprints (shapely) the parts are built from.

Units mm. Axes: X forward, Y left, Z up. z = 0 is the motor base plane:
motors hang below the arm pads (inverted), props are under the motors.

Arm frame: a point on an arm is M - t*u + c*n, where M is the motor centre,
u the unit vector from the arm root towards the motor, n = (-u_y, u_x).
t grows from the motor towards the spine.
"""
import math

from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

# ---- layout ---------------------------------------------------------------
MX, MY = 80.0, 96.0          # motor centres (+-MX, +-MY): 160 x 192, wheelbase 250
ARM_DEG = 70.0               # arm direction from +X for the front-left arm

# ---- prop and duct ring (prop 5.1" = 129.5 mm, datasheet) -------------------
PROP_R = 64.75
PROP_Z = -24.0               # prop disc plane (estimate: bell 19.2 + hub)
RI = 67.0                    # ring inner radius, 2.25 mm tip gap
RW = 1.2                     # ring wall (3 perimeters at 0.4)
LIP = 4.0                    # 45 deg inlet chamfer at the top
ZT, ZB = 6.0, -36.0          # ring top (= spine cavity top) / bottom rim (= landing gear)
R_OUT = RI + LIP + RW        # 72.2, outer radius at the lip
RING_GAP = 2.0               # lip to spine clearance

# ---- spine sandwich ---------------------------------------------------------
SP_X = 100.0                 # spine from -100 to +100
SP_HW_MAX, SP_HW_MIN = 40.0, 22.0
SP_WAIST_HW = 28.0           # narrower waist in the middle (|x| < 18), nothing there needs the width
FLOOR = 3.0                  # bottom plate floor, z -3..0
CAV = 6.0                    # cavity = tongue thickness, z 0..6 (stack 3+6+3 = 12: M3x16 + nylock fits)
TOP = 3.0                    # top plate, z 6..9
WALL = 3.0                   # pocket walls
WALL_RIM = 2.4               # outer rim of the cavity (6 perimeters)
CLR = 0.2                    # pocket clearance per side
CRUSH = 0.2                  # cavity walls are 0.2 lower than the tongue (5.8 vs 6): the top plate clamps the tongue, not the walls

# ---- arm ----------------------------------------------------------------------
TONGUE_HW = 17.0             # tongue half width at the head
NECK_HW = 12.5               # half width of the neck where the tongue leaves the spine
TONGUE_IN = 40.0             # tongue length inside the loft root
ROOT_W, ROOT_H = 28.0, 18.0  # loft root section (flare)
GUSSET = 10.0                # ramp from the loft root (18) down onto the tongue (6), about 50 deg
PAD_R, PAD_T = 15.5, 6.0     # motor pad ø31 (motor flange ø28); 6 mm keeps 3 mm of the stock 11 mm screws in the motor
PAD_HOLE_D = 12.0            # shaft / circlip ø10 + air
MOTOR_BC = 16.0              # motor screws on a ø16 circle (datasheet drawing, opposite holes 16 apart)
MOTOR_SCREW_D = 3.3
LOFT_END_T = 11.0            # loft ends 11 mm from the motor axis: inside the pad, clear of the screw heads
ARM_STYLE = "bone"           # bone | blade | tee | fork, see parts.free_arm
TIE_Q = (0.35, 0.72)         # zip-tie notches on top of the arm (loft parameter), motor wires run underneath
TIE_W, TIE_D = 4.0, 1.2

# ---- fasteners ------------------------------------------------------------------
M3_CLEAR, M3_PILOT = 3.4, 2.8   # 2.8 pilot tested on the printed deck (2026-10-03)
BOLT_EDGE = 4.5                 # min distance from a bolt axis to a part edge
NUT_AF, NUT_SEAT = 5.8, 1.2     # M3 nylock (5.5 AF) seat under the floor: stops it turning, one hex key from the top
RING_TAB_T = 5.0                # ring tab under the floor (1 mm in a recess), z -7..-2; M3x16 from below into a captive nylock in the spine
RING_RECESS = 1.0               # locating recess for the ring tab in the floor underside

# ---- landing skids (PETG) ---------------------------------------------------------
# Two identical skids under the spine. Each hangs on two arm clamp bolts (front and rear arm,
# the bolt at |x| 38, |y| 16.6), M3x30 instead of M3x16, nylock in a side slot of the skid.
SKID_T = 8.0                 # skid thickness (y); printed on its side, layers in the profile plane
SKID_Z = -42.0               # runner bottom: 6 mm below the ring rims, 12 mm below the FC stack
SKID_BLOCK_Z = -23.0         # mount block bottom
SKID_NUT_Z = (-19.4, -15.0)  # nylock slot (4.2 tall) -> M3x30 ends 1 mm past the nut

# ---- belly / box --------------------------------------------------------------
STACK = 30.5 / 2
WIRE_SLOTS = [(86.5, -6.0, 96.0, 6.0), (-96.0, -6.0, -86.5, 6.0)]   # front: Pi / camera / HW-636 wires; rear: battery lead (XT60 behind the box)

DENSITY_PLA = 1.24e-3           # g / mm^3


def motors():
    """[(sx, sy, M, u, n)] for FL, FR, RL, RR."""
    out = []
    a = math.radians(ARM_DEG)
    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        M = (sx * MX, sy * MY)
        u = (sx * math.cos(a), sy * math.sin(a))
        n = (-u[1], u[0])
        out.append((sx, sy, M, u, n))
    return out


def arm_pt(M, u, n, t, c):
    return (M[0] - t * u[0] + c * n[0], M[1] - t * u[1] + c * n[1])


def arm_rect(M, u, n, t0, t1, c0, c1):
    return Polygon([arm_pt(M, u, n, t, c) for t, c in ((t0, c0), (t1, c0), (t1, c1), (t0, c1))])


def _ss(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def outline():
    """Spine footprint: 2 mm clear of every ring lip, never narrower than 44."""
    wide = box(-SP_X, -SP_HW_MAX, SP_X, SP_HW_MAX)
    rings = unary_union([Point(M).buffer(R_OUT + RING_GAP, 256) for _, _, M, _, _ in motors()])
    core = box(-SP_X, -SP_HW_MIN, SP_X, SP_HW_MIN)
    shape = unary_union([wide.difference(rings), core])
    # waist: smooth from 2 x 28 in the middle to 2 x 40 at |x| = 34
    xs = [-SP_X + i * 2.0 for i in range(int(SP_X))] + [SP_X]
    hw = lambda x: SP_WAIST_HW + (SP_HW_MAX - SP_WAIST_HW) * _ss(18.0, 34.0, abs(x))
    waist = Polygon([(x, hw(x)) for x in xs] + [(x, -hw(x)) for x in reversed(xs)])
    shape = shape.intersection(waist)
    # soften the four outer corners
    return shape.buffer(-4, join_style=1).buffer(4, join_style=1)


def _first_hit(M, u, n, half_w, shape, t_from=20.0, t_to=140.0, step=0.1):
    """Smallest t at which the section segment (c in +-half_w) touches `shape`."""
    t = t_from
    while t < t_to:
        seg = Polygon([arm_pt(M, u, n, t, -half_w), arm_pt(M, u, n, t, half_w),
                       arm_pt(M, u, n, t + 0.01, half_w), arm_pt(M, u, n, t + 0.01, -half_w)])
        if seg.intersects(shape):
            return t
        t += step
    raise ValueError("arm never reaches the spine")


class ArmLayout:
    """Everything 2D about one arm: loft root, tongue, pocket, bolts."""

    def __init__(self, sx, sy, M, u, n, ol):
        self.sx, self.sy, self.M, self.u, self.n = sx, sy, M, u, n
        # loft root: the whole root section (plus the gusset ramp) must stay outside the spine
        self.t_axis = _first_hit(M, u, n, 0.01, ol)              # where the arm axis enters the spine
        self.t0 = _first_hit(M, u, n, ROOT_W / 2, ol.buffer(0.5)) - 0.05
        self.t_in = self.t0 + TONGUE_IN
        half = box(-500, 1.6, 500, 500) if sy > 0 else box(-500, -500, 500, -1.6)
        # hammer-head tongue: narrow neck where it leaves the spine, wide head inside.
        # The arm drops in from above, so the head can be wider than the neck: the pocket
        # shoulders then hold the arm against pull-out, the bolts only clamp.
        self.t_neck = self.t0 + GUSSET + 2.0
        rect = unary_union([arm_rect(M, u, n, self.t0 - 4, self.t_neck, -NECK_HW, NECK_HW),
                            arm_rect(M, u, n, self.t_neck, self.t_in, -TONGUE_HW, TONGUE_HW)]).intersection(half)
        rect = rect.buffer(-1.0).buffer(1.0)            # R1 corners: no sharp notch in the pocket
        inside = rect.intersection(ol)
        # keep the part outside the spine only next to the root (between loft root and spine edge)
        root_strip = arm_rect(M, u, n, self.t0 - 4, self.t0 + GUSSET + 12, -TONGUE_HW, TONGUE_HW)
        outside = rect.difference(ol).intersection(root_strip)
        self.tongue = unary_union([inside, outside]).buffer(0.01).buffer(-0.01)
        self.tongue_in = inside
        self.pocket = inside.buffer(CLR, join_style=2).intersection(ol)
        # gusset footprint (needs a notch in the top plate)
        self.gusset = arm_rect(M, u, n, self.t0, self.t0 + GUSSET, -ROOT_W / 2, ROOT_W / 2)
        # three clamp bolts: two across the neck right behind the gusset (they hold the top plate
        # down where the thrust lever pushes up), one deep in the head (holds the tail end down)
        safe = self.tongue_in.difference(arm_rect(M, u, n, 0, self.t0 + GUSSET + 1.0, -50, 50)).buffer(-BOLT_EDGE)
        if safe.geom_type != "Polygon":
            safe = max(safe.geoms, key=lambda g: g.area)
        x0, y0, x1, y1 = safe.bounds
        grid = [(x0 + i * 0.5, y0 + j * 0.5) for i in range(int((x1 - x0) / 0.5) + 1)
                for j in range(int((y1 - y0) / 0.5) + 1)]
        grid = [q for q in grid if safe.contains(Point(q))]
        tt = lambda q: -((q[0] - M[0]) * u[0] + (q[1] - M[1]) * u[1])     # t of a point
        cc = lambda q: (q[0] - M[0]) * n[0] + (q[1] - M[1]) * n[1]        # c of a point
        t_min = min(tt(q) for q in grid)
        front = [q for q in grid if tt(q) <= t_min + 2.0]
        self.bolts = [min(front, key=cc), max(front, key=cc)]
        self.bolts.append(max(grid, key=lambda q: tt(q) - 0.15 * abs(cc(q))))

    def check(self):
        errs = []
        for b in self.bolts:
            if self.tongue_in.exterior.distance(Point(b)) < BOLT_EDGE - 0.01:
                errs.append(f"bolt {b[0]:.1f},{b[1]:.1f} closer than {BOLT_EDGE} to the tongue edge")
        for i in range(len(self.bolts)):
            for j in range(i + 1, len(self.bolts)):
                if Point(self.bolts[i]).distance(Point(self.bolts[j])) < 9:
                    errs.append("clamp bolts closer than 9 mm (washers ø7 overlap)")
        return errs


class RingLayout:
    """Spine tab (bolted under the floor, ring comes off downwards) and bridge tab (to the neighbour ring)."""

    def __init__(self, sx, sy, M, u, n, ol):
        self.sx, self.sy, self.M = sx, sy, M
        x0, x1 = sorted((sx * 75.0, sx * 93.0))
        y0, y1 = sorted((sy * 11.5, sy * (MY - RI + 3.0)))  # into the ring wall
        disc_in = Point(M).buffer(RI + 0.3, 256)
        self.tab = box(x0, y0, x1, y1).buffer(-2).buffer(2).difference(disc_in)
        self.tab_in = self.tab.intersection(ol)
        self.recess = self.tab_in.buffer(CLR, join_style=2)   # 1 mm locating recess under the floor
        self.y_wall = sy * (MY - RI - RW)                                      # inner face of the gusset
        self.bolts = [(sx * 80.0, sy * 17.0), (sx * 88.0, sy * 17.0)]
        bx0, bx1 = sorted((sx * 1.0, sx * (MX - RI + 2.0)))
        self.bridge_tab = box(bx0, MY * sy - 7.0, bx1, MY * sy + 7.0).difference(disc_in)
        self.bridge_bolt = (sx * 6.0, sy * MY)


def build_layout():
    ol = outline()
    arms = [ArmLayout(*m, ol) for m in motors()]
    for a in arms[1:]:   # exact mirror images of the front-left arm
        a.bolts = [(a.sx * x, a.sy * y) for x, y in arms[0].bolts]
    rings = [RingLayout(*m, ol) for m in motors()]
    errs = []
    for a in arms:
        errs += a.check()
    pockets = [a.pocket for a in arms] + [r.recess for r in rings]
    for i in range(len(pockets)):
        for j in range(i + 1, len(pockets)):
            d = pockets[i].distance(pockets[j])
            if d < 2.0:
                errs.append(f"pockets {i} and {j} only {d:.2f} mm apart")
    return ol, arms, rings, errs


if __name__ == "__main__":
    ol, arms, rings, errs = build_layout()
    print(f"spine outline area {ol.area:.0f} mm2, bounds {[round(v, 1) for v in ol.bounds]}")
    for name, a in zip(("FL", "FR", "RL", "RR"), arms):
        print(f"{name}: t_axis {a.t_axis:.1f}, loft root t0 {a.t0:.1f}, free loft {a.t0 - LOFT_END_T:.1f} mm, "
              f"tongue {a.tongue.area:.0f} mm2, bolts {[(round(x, 1), round(y, 1)) for x, y in a.bolts]}")
    print("errors:", errs or "none")
