"""Box on top of the spine: battery cradle with deck posts, deck for the Pi 5, camera cassette.

The battery lies directly on the spine (over the FC stack), the deck with the Pi sits above it on 4 posts
and the front stop wall. The battery slides out backwards: unplug the XT60, open the two straps,
lift the tail over the 3 mm rear lip, pull. The HW-636 (Pi supply) hangs under the spine in front of the stack.

Same axes as layout.py; the spine top surface is at z = ZS.
"""
import math

import cadquery as cq
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from layout import CAV, M3_CLEAR, M3_PILOT, TOP
from parts import hex_windows, holes, prism

ZS = CAV + TOP                         # 9.0, spine top surface (top plate rests on the tongues)

# ---- battery (measured 2026-09-03: 140 x 43 x 38, XT60) ---------------------------------
BAT = (140.0, 43.0, 38.0)
BAT_X = 0.0                            # battery centre with no spacer (front at x 70); a spacer of t moves it back by t
PAD = 1.0                              # rubber / anti-slip pad under the battery
SEAT_Z = 16.0                          # cradle top: arm ramps reach z 14.5 inside |y| 21.5, battery bottom at 17
BAT_Z = SEAT_Z + PAD
LIFT = 3.0                             # rear lip: the tail is lifted this much to slide the pack out
DECK_Z = BAT_Z + BAT[2] + LIFT + 2.0   # deck underside: pack + lift + 2 mm
DECK_T = 2.5

# ---- cradle ---------------------------------------------------------------------------------
RIB = 2.4                              # rib / rim width (6 perimeters)
MID_X, MID_HW = 36.0, 25.5             # wide middle part |x| < 36; the arm ramps start at x 41
END_X, END_HW = 84.5, 15.5             # narrow ends between the arm ramps (they reach |y| 17.2 at z 9.5)
BOX_SCREWS = [(-80.0, 0.0), (-20.0, 0.0), (20.0, 0.0), (80.0, 0.0)]   # into the spine pilots, M3x16 self-tapping
SCREW_SEAT = 13.0                      # head seat: M3x16 ends at z -3, flush with the floor underside
STRAPS = [(-16.0, 21.0), (16.0, 21.0)] # (x centre, width): 20 mm straps loop under the rim of the middle part
STRAP_GROOVE = 1.8
POSTS = [(sx * 30.0, sy * 27.0) for sx in (1, -1) for sy in (1, -1)]
POST_R = 3.75
FRONT_X = BAT_X + BAT[0] / 2 + 0.5     # stop wall rear face
WALL_T = 3.0
REAR_X = FRONT_X - BAT[0] - 6.0        # rear lip front face: 146 mm of room, a front spacer (0..6) sets the balance
SPACER_HOLES = [(sy * 12.0, BAT_Z + 19.0) for sy in (1, -1)]   # (y, z) in the stop wall, M3x16 + nylock
SPACER_T = 4.0                          # printed default; measure the CG and pick 0..6
GUIDE_HW = BAT[1] / 2 + 0.6            # side guides

# ---- deck -------------------------------------------------------------------------------------
DECK_X = (-40.0, 100.0)
PI_C = (20.0, 0.0)                     # Pi 5 board centre; USB / Ethernet end forward, GPIO end back
PI_BOARD = (85.0, 56.0)
PI_HOLES = [(PI_C[0] - 10.0 + sx * 29.0, sy * 24.5) for sx in (1, -1) for sy in (1, -1)]   # 58 x 49, pattern 10 mm toward the GPIO end
PI_BOSS = dict(d=6.0, h=7.0, pilot=2.2)                    # M2.5 cuts its own thread (tested on deck v2)
WALL_BOSSES = [(FRONT_X + 3.6, sy * 13.0) for sy in (1, -1)]   # in front of the wall, clear of the pack
CAM_HOLES = [(84.0, sy * 6.0) for sy in (1, -1)]           # camera cassette, 2 x M3x16 + nylock
CAM_FLANGE = (80.0, 100.0, 16.0)                           # x0, x1, half width
DECK_SLOT = (74.5, -7.0, 79.5, 7.0)                        # wires from the front slot of the spine, in front of the stop wall

# ---- camera cassette (Arducam B0392: board 25 x 24, lens block 14 high) -----------------------
CAM_TILT = 15.0                        # looking down from the horizon
CAM_PCB = (25.0, 24.0, 1.0)
CAM_X0 = 88.0                          # rear bottom edge of the cassette


def _ss(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def cradle_outline():
    mid = box(-MID_X, -MID_HW, MID_X, MID_HW)
    ends = box(-END_X, -END_HW, END_X, END_HW)
    posts = unary_union([Point(p).buffer(POST_R + 0.01, 48) for p in POSTS])
    return unary_union([mid, ends]).buffer(3, join_style=1).buffer(-3, join_style=1).union(posts)


def cradle(arm_bolts):
    """One part: open frame on the spine (battery seat), side guides, front stop wall, rear lip, 4 deck posts.
    Prints upright on its flat underside, no supports (strap grooves under the rim are 21 mm bridges)."""
    ol = cradle_outline()
    inner = ol.buffer(-RIB, join_style=2)
    rim = ol.difference(inner)
    ribs = [box(-END_X, -RIB / 2, END_X, RIB / 2)]                       # keel
    for x in (-80.0, -55.0, -20.0, 0.0, 20.0, 55.0, 80.0):
        ribs.append(box(x - RIB / 2, -MID_HW, x + RIB / 2, MID_HW))
    bosses = [Point(p).buffer(4.5, 32) for p in BOX_SCREWS]
    frame = unary_union([rim] + ribs + bosses).intersection(ol)
    # clear the arm clamp bolt heads (button head + washer) and keep hex-key access to them through the frame
    heads = unary_union([Point(b).buffer(4.5, 32) for b in arm_bolts])
    frame = frame.difference(heads).union(unary_union([Point(b).buffer(4.5 + 1.6, 32).difference(Point(b).buffer(4.5, 32))
                                                       for b in arm_bolts]).intersection(ol))
    body = prism(frame, ZS, SEAT_Z)
    body = body.cut(holes(BOX_SCREWS, M3_CLEAR, ZS - 1, SEAT_Z + 1))
    body = body.cut(holes(BOX_SCREWS, 6.6, SCREW_SEAT, SEAT_Z + 1))
    # strap grooves under the rim of the middle part (the strap loops: over the pack, under the rim, back up)
    for x, w in STRAPS:
        body = body.cut(prism(box(x - w / 2, -MID_HW - 1, x + w / 2, MID_HW + 1), ZS - 1, ZS + STRAP_GROOVE))
    # side guides along the middle, open where the straps run
    g = box(-MID_X + 2, GUIDE_HW, MID_X - 2, GUIDE_HW + 1.8)
    for x, w in STRAPS:
        g = g.difference(box(x - w / 2 - 1, -99, x + w / 2 + 1, 99))
    guides = unary_union([g, Polygon([(px, -py) for px, py in g.exterior.coords]) if g.geom_type == "Polygon" else
                          unary_union([Polygon([(px, -py) for px, py in q.exterior.coords]) for q in g.geoms])])
    body = body.union(prism(guides, SEAT_Z - 0.5, BAT_Z + 12))
    # posts
    for p in POSTS:
        post = cq.Workplane("XY").workplane(offset=ZS).center(*p).circle(POST_R).extrude(DECK_Z - ZS)
        body = body.union(post)
    body = body.cut(holes(POSTS, M3_PILOT, DECK_Z - 14, DECK_Z + 1))
    # front stop wall: narrow below z 19 (arm ramps), full width above; two bosses for the deck on top
    low = box(FRONT_X, -END_HW, FRONT_X + WALL_T, END_HW)
    high = box(FRONT_X, -GUIDE_HW - 1.8, FRONT_X + WALL_T, GUIDE_HW + 1.8)
    wall = prism(low, ZS, 19.5).union(prism(high, 19.0, DECK_Z))
    win = box(FRONT_X - 1, -9, FRONT_X + WALL_T + 1, 9).buffer(-1).buffer(1)
    wall = wall.cut(prism(win, BAT_Z + 6, DECK_Z - 9))
    for p in WALL_BOSSES:
        wall = wall.union(cq.Workplane("XY").workplane(offset=DECK_Z - 12).center(*p).circle(3.6).extrude(12))
    wall = wall.cut(holes(WALL_BOSSES, M3_PILOT, DECK_Z - 13, DECK_Z + 1))
    # gusset in front of the wall, along the keel
    pl = cq.Plane(origin=(0, 0, 0), xDir=(1, 0, 0), normal=(0, -1, 0))
    gus = cq.Workplane(pl).polyline([(FRONT_X + WALL_T - 0.5, ZS), (END_X - 1, ZS), (END_X - 1, SEAT_Z),
                                     (FRONT_X + WALL_T - 0.5, DECK_Z - 14)]).close().extrude(RIB / 2, both=True)
    body = body.union(wall).union(gus)
    body = body.cut(holes([BOX_SCREWS[3]], 6.6, SCREW_SEAT, DECK_Z))     # keep the front screw reachable
    body = body.cut(holes([BOX_SCREWS[3]], M3_CLEAR, ZS - 1, DECK_Z))
    # rear lip
    lip = box(REAR_X - WALL_T, -END_HW, REAR_X, END_HW).difference(Point(BOX_SCREWS[0]).buffer(5.0))
    body = body.union(prism(lip, ZS, BAT_Z + LIFT))
    # spacer holes in the stop wall
    for y, z in SPACER_HOLES:
        body = body.cut(cq.Workplane("YZ").workplane(offset=FRONT_X - 1).center(y, z).circle(M3_CLEAR / 2).extrude(WALL_T + 2))
    return body


def deck_outline():
    x0, x1 = DECK_X
    xs = [x0 + i * (x1 - x0) / 80 for i in range(81)]
    hw = [22.0 + 9.5 * _ss(-40, -30, x) * (1 - _ss(64, 78, x)) for x in xs]
    pts = [(x, h) for x, h in zip(xs, hw)] + [(x, -h) for x, h in reversed(list(zip(xs, hw)))]
    return Polygon(pts).buffer(-4, join_style=1).buffer(4, join_style=1)


def deck():
    """Flat deck on the posts and the stop wall. Prints on its flat underside, the Pi bosses and the edge ribs
    stand up, no supports. Pi on printed bosses (M2.5 self-tapping), camera cassette on the nose."""
    ol = deck_outline()
    screws = POSTS + WALL_BOSSES
    keep = unary_union([Point(p).buffer(6.0) for p in screws + PI_HOLES + CAM_HOLES]
                       + [box(CAM_FLANGE[0], -CAM_FLANGE[2], CAM_FLANGE[1], CAM_FLANGE[2])])
    win_region = ol.buffer(-4).difference(keep)
    windows = hex_windows(win_region, r=8.0, pitch=18.5)
    slot = box(*DECK_SLOT).buffer(-1.5).buffer(1.5)
    plate = ol.difference(windows).difference(slot)
    body = prism(plate, DECK_Z, DECK_Z + DECK_T)
    # edge ribs above the deck (stiffness of the overhangs), away from the Pi board
    rim = ol.difference(ol.buffer(-2.0)).difference(box(PI_C[0] - 45, -30, PI_C[0] + 45, 30)) \
        .difference(box(CAM_FLANGE[0] - 1, -CAM_FLANGE[2] - 1, 200, CAM_FLANGE[2] + 1))
    body = body.union(prism(rim, DECK_Z + DECK_T, DECK_Z + DECK_T + 4))
    for p in PI_HOLES:
        body = body.union(cq.Workplane("XY").workplane(offset=DECK_Z + DECK_T).center(*p).circle(PI_BOSS["d"] / 2)
                          .extrude(PI_BOSS["h"]))
    body = body.cut(holes(PI_HOLES, PI_BOSS["pilot"], DECK_Z + 0.8, DECK_Z + DECK_T + PI_BOSS["h"] + 1))
    body = body.cut(holes(screws + CAM_HOLES, M3_CLEAR, DECK_Z - 1, DECK_Z + DECK_T + 5))
    return body


def camera():
    """Cassette for the Arducam: flange on the deck nose (2 x M3x16 + nylock), the board slides in from the top
    into a groove, tilted CAM_TILT down. Prints flange down."""
    zf = DECK_Z + DECK_T
    w, h, t = CAM_PCB
    fl = prism(box(CAM_FLANGE[0], -CAM_FLANGE[2], CAM_FLANGE[1], CAM_FLANGE[2]).buffer(-3).buffer(3), zf, zf + 3.0)
    fl = fl.cut(holes(CAM_HOLES, M3_CLEAR, zf - 1, zf + 4))
    wall, gt, pil = 1.6, 1.5, 2.4
    T, W, H = 2 * wall + gt, w + 2 * pil, 22.0
    cas = cq.Workplane("XY").center(T / 2, 0).rect(T, W).extrude(H + 8).translate((0, 0, -8))
    cas = cas.cut(cq.Workplane("XY").center(wall + gt / 2, 0).rect(gt, w + 0.4).extrude(H + 1))           # board groove
    cas = cas.cut(cq.Workplane("XY").center(T - wall / 2 + 0.1, 0).rect(wall + 0.2, w - 3.0).extrude(H + 1).translate((0, 0, 1.0)))  # lens side open
    cas = cas.cut(cq.Workplane("XY").center(wall / 2 - 0.1, 0).rect(wall + 0.2, 22.0).extrude(H).translate((0, 0, 16.0)))          # FPC connector
    cas = cas.rotate((0, 0, 0), (0, 1, 0), CAM_TILT).translate((CAM_X0, 0, zf + 3.0 + T * math.sin(math.radians(CAM_TILT))))
    cas = cas.cut(cq.Workplane("XY").rect(400, 400).extrude(60).translate((0, 0, zf + 3.0 - 60)))
    # two side gussets behind the cassette
    pl = cq.Plane(origin=(0, 0, 0), xDir=(1, 0, 0), normal=(0, -1, 0))
    for sy in (1, -1):
        g = cq.Workplane(pl).polyline([(CAM_X0 - 9, zf + 3), (CAM_X0 + 1, zf + 3), (CAM_X0 + 1 + 14 * math.sin(math.radians(CAM_TILT)), zf + 3 + 14)]) \
            .close().extrude(1.2, both=True).translate((0, sy * (W / 2 - 1.2), 0))
        cas = cas.union(g)
    return fl.union(cas)


def spacer(t=SPACER_T):
    """Plate between the stop wall and the pack: moves the pack back by t for balance (or holds a shorter pack).
    Prints flat."""
    z0, z1 = BAT_Z + 6.0, BAT_Z + 32.0
    body = cq.Workplane("YZ").workplane(offset=FRONT_X - t).center(0, (z0 + z1) / 2).rect(32.0, z1 - z0).extrude(t) \
        .edges("|X").fillet(3)
    for y, z in SPACER_HOLES:
        body = body.cut(cq.Workplane("YZ").workplane(offset=FRONT_X - t - 1).center(y, z).circle(M3_CLEAR / 2).extrude(t + 2))
    return body


# ---- reference blocks (not printed): for the checks and the viewer --------------------------------
def refs():
    L, W, H = BAT
    bat = cq.Workplane("XY").workplane(offset=BAT_Z).center(FRONT_X - SPACER_T - 0.5 - BAT[0] / 2, 0).rect(L, W).extrude(H).edges("|X").fillet(3)
    pi = cq.Workplane("XY").workplane(offset=DECK_Z + DECK_T + PI_BOSS["h"]).center(*PI_C).rect(*PI_BOARD).extrude(1.6)
    pi = pi.union(cq.Workplane("XY").workplane(offset=DECK_Z + DECK_T + PI_BOSS["h"] + 1.6).center(PI_C[0] + 2, 0)
                  .rect(60, 50).extrude(15.0))                                           # active cooler + ports, rough
    hw = cq.Workplane("XY").workplane(offset=-8.0 - 21.0).center(63.0, 0).rect(76.0, 22.2).extrude(21.0)
    w, h, t = CAM_PCB
    zf = DECK_Z + DECK_T + 3.0
    cam = cq.Workplane("XY").center(1.6 + 0.75, 0).rect(t, w).extrude(h).union(
        cq.Workplane("YZ").workplane(offset=1.6 + 1.5).center(0, h - 9).rect(14, 14).extrude(14))
    cam = cam.rotate((0, 0, 0), (0, 1, 0), CAM_TILT).translate((CAM_X0, 0, zf + 3.2 * math.sin(math.radians(CAM_TILT)) + 0.6))
    return {"ref_battery": bat, "ref_pi5": pi, "ref_hw636": hw, "ref_camera": cam}


def build_box(arm_bolts):
    return {"box_cradle": cradle(arm_bolts), "box_deck": deck(), "box_camera": camera(), "box_spacer": spacer()}


if __name__ == "__main__":
    from layout import DENSITY_PLA, build_layout
    ol, arms, rings, errs = build_layout()
    bolts = [b for a in arms for b in a.bolts]
    for n, wp in {**build_box(bolts), **refs()}.items():
        s = wp.val()
        bb = s.BoundingBox()
        print(f"{n:12s} valid={s.isValid()}  x {bb.xmin:6.1f}..{bb.xmax:6.1f}  y {bb.ymin:6.1f}..{bb.ymax:6.1f}  z {bb.zmin:5.1f}..{bb.zmax:5.1f}  {s.Volume() * DENSITY_PLA:5.1f} g")
    print(f"battery z {BAT_Z}..{BAT_Z + BAT[2]}, deck z {DECK_Z}..{DECK_Z + DECK_T}")
