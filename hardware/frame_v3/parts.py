"""Frame v3 solids (CadQuery), built from the 2D layout in layout.py.

Parts: bottom plate, top plate, 4 arms (two mirror pairs), 4 rings (two mirror pairs), 2 bridges.
Sandwich: the arm tongue and the ring tab sit in pockets of the bottom plate (cavity z 0..8)
and are clamped by the top plate with M3 bolts and nylock nuts.
"""
import math

import cadquery as cq
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

from layout import (CRUSH, RING_RECESS, RING_TAB_T, SKID_BLOCK_Z, SKID_NUT_Z, SKID_T, SKID_Z, WALL_RIM, NUT_AF, NUT_SEAT, ARM_STYLE, TIE_D, TIE_Q, TIE_W, CAV, CLR, FLOOR, GUSSET, LIP, LOFT_END_T, M3_CLEAR, M3_PILOT, MOTOR_BC, MOTOR_SCREW_D,
                    MY, PAD_HOLE_D, PAD_R, PAD_T, RI, ROOT_H, ROOT_W, RW, STACK, TOP, WALL, WIRE_SLOTS,
                    ZB, ZT, arm_pt, arm_rect, build_layout)


# ---- helpers ----------------------------------------------------------------------
def _polys(g):
    if g.is_empty:
        return []
    if isinstance(g, Polygon):
        return [g]
    if isinstance(g, MultiPolygon):
        return list(g.geoms)
    return [p for p in getattr(g, "geoms", []) if isinstance(p, Polygon)]


def prism(geom, z0, z1):
    """Extrude a shapely (multi)polygon with holes between z0 and z1."""
    solids = []
    for p in _polys(geom):
        p = p.simplify(0.02)
        wp = cq.Workplane("XY").workplane(offset=z0).polyline(list(p.exterior.coords)[:-1]).close().extrude(z1 - z0)
        for hole in p.interiors:
            cut = cq.Workplane("XY").workplane(offset=z0 - 1).polyline(list(hole.coords)[:-1]).close().extrude(z1 - z0 + 2)
            wp = wp.cut(cut)
        solids.append(wp)
    out = solids[0]
    for s in solids[1:]:
        out = out.union(s)
    return out


def holes(points, d, z0, z1):
    return prism(unary_union([Point(p).buffer(d / 2, 32) for p in points]), z0, z1)


def hexagon(c, af):
    r = af / math.sqrt(3)
    return Polygon([(c[0] + r * math.cos(math.radians(60 * k)), c[1] + r * math.sin(math.radians(60 * k))) for k in range(6)])


def ss(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


# ---- arm ----------------------------------------------------------------------------
def arm_w(q):
    """Loft width: 28 at the root (flare), 15 in the middle, 21 at the pad (tulip, stays inside the pad circle)."""
    return 15 + 13 * (1 - ss(0, 0.16, q)) + 6 * ss(0.55, 1, q) ** 1.4


def arm_h(q):
    """Loft height: 18 at the root, 6 at the pad; drops slowly first, where the bending is."""
    return 6 + (ROOT_H - 6) * (1 - q ** 2.2)


def section_pts(w, h, n_arc=8):
    """Rounded rectangle with a flat bottom (prints on the bed): bottom r 1, top r 0.35*min(w,h)."""
    rb, rt = 1.0, 0.35 * min(w, h)
    pts = []
    corners = [((w / 2 - rb, rb), rb, -90, 0), ((w / 2 - rt, h - rt), rt, 0, 90),
               ((-w / 2 + rt, h - rt), rt, 90, 180), ((-w / 2 + rb, rb), rb, 180, 270)]
    for (cx, cz), r, a0, a1 in corners:
        for i in range(n_arc + 1):
            a = math.radians(a0 + (a1 - a0) * i / n_arc)
            pts.append((cx + r * math.cos(a), cz + r * math.sin(a)))
    return pts


def arm_footprint(al, n=40):
    M, u, nn = al.M, al.u, al.n
    quads = []
    prev = None
    for i in range(n + 1):
        q = i / n
        t = al.t0 - q * (al.t0 - LOFT_END_T)
        w = (fork_w if ARM_STYLE == "fork" else arm_w)(q)
        seg = (arm_pt(M, u, nn, t, -w / 2), arm_pt(M, u, nn, t, w / 2))
        if prev:
            quads.append(Polygon([prev[0], prev[1], seg[1], seg[0]]))
        prev = seg
    return unary_union(quads + [Point(M).buffer(PAD_R, 64), al.tongue])


ARM_STYLES = ("bone", "blade", "tee", "fork")


def fork_w(q):
    """Fork outline: wider, so two 5 mm rails leave a window between them."""
    return 22 + 10 * (1 - ss(0, 0.16, q)) - 1 * ss(0.55, 1, q) ** 1.4


def _outline(al, wf, q0=0.0, q1=1.0, n=48, extra_root=0.5):
    """Top-view outline of the free arm between q0 and q1 for a width law wf."""
    M, u, nn = al.M, al.u, al.n
    left, right = [], []
    for i in range(n + 1):
        q = q0 + (q1 - q0) * i / n
        t = al.t0 - q * (al.t0 - LOFT_END_T) + (extra_root if i == 0 and q0 == 0 else 0)
        left.append(arm_pt(M, u, nn, t, wf(q) / 2))
        right.append(arm_pt(M, u, nn, t, -wf(q) / 2))
    return Polygon(left + right[::-1])


def _side_profile(al, n=48):
    """Side view (t, z): flat bottom, top follows arm_h; extruded across the arm."""
    M, u, nn = al.M, al.u, al.n
    pts = [(al.t0 + 0.5, 0.0), (al.t0 + 0.5, arm_h(0))]
    for i in range(1, n + 1):
        q = i / n
        pts.append((al.t0 - q * (al.t0 - LOFT_END_T), arm_h(q)))
    pts += [(LOFT_END_T - 0.5, arm_h(1)), (LOFT_END_T - 0.5, 0.0)]
    o = M
    pl = cq.Plane(origin=(o[0], o[1], 0), xDir=(-u[0], -u[1], 0), normal=(nn[0], nn[1], 0))
    return cq.Workplane(pl).polyline(pts).close().extrude(40, both=True)


def free_arm(al, style):
    """The free part of the arm between the spine and the pad, in one of four styles."""
    M, u, n = al.M, al.u, al.n
    if style == "bone":      # lofted rounded section
        wires = []
        for q in (0.0, 0.08, 0.16, 0.3, 0.45, 0.6, 0.72, 0.84, 0.93, 1.0):
            t = al.t0 - q * (al.t0 - LOFT_END_T)
            cx, cy = arm_pt(M, u, n, t, 0)
            pts = [cq.Vector(cx + c * n[0], cy + c * n[1], z) for c, z in section_pts(arm_w(q), arm_h(q))]
            wires.append(cq.Wire.makePolygon(pts, close=True))
        return cq.Workplane("XY").add(cq.Solid.makeLoft(wires, ruled=False))
    side = _side_profile(al)
    if style == "blade":     # top outline x side profile, crisp edges
        return prism(_outline(al, arm_w), 0, ROOT_H + 1).intersect(side)
    if style == "tee":       # 3 mm flange + 4 mm rib on top
        flange = prism(_outline(al, arm_w), 0, 3.0)
        rib = prism(_outline(al, lambda q: 4.0), 0, ROOT_H + 1).intersect(side)
        return flange.union(rib)
    if style == "fork":      # two 5 mm rails with a window between them
        outer = _outline(al, fork_w)
        window = _outline(al, lambda q: max(fork_w(q) - 10, 0.2), 0.14, 0.78).buffer(-1.5).buffer(1.5)
        return prism(outer.difference(window), 0, ROOT_H + 1).intersect(side)
    raise ValueError(style)


def arm(al, style="bone"):
    M, u, n = al.M, al.u, al.n
    loft = free_arm(al, style)
    body = loft.union(prism(al.tongue, 0, CAV))
    # 45 deg ramp from the loft root down onto the tongue
    o = arm_pt(M, u, n, al.t0, 0)
    pl = cq.Plane(origin=(o[0], o[1], 0), xDir=(-u[0], -u[1], 0), normal=(n[0], n[1], 0))
    ramp = cq.Workplane(pl).polyline([(-1, CAV), (-1, ROOT_H - 0.5), (0, ROOT_H - 0.5), (GUSSET, CAV)]).close() \
        .extrude(ROOT_W / 2 - 1.5, both=True)
    body = body.union(ramp)
    # motor pad
    pad = cq.Workplane("XY").center(*M).circle(PAD_R).extrude(PAD_T)
    body = body.union(pad)
    au = math.atan2(u[1], u[0])
    screws = [(M[0] + MOTOR_BC / 2 * math.cos(au + math.pi / 4 + k * math.pi / 2),
               M[1] + MOTOR_BC / 2 * math.sin(au + math.pi / 4 + k * math.pi / 2)) for k in range(4)]
    body = body.cut(holes(screws, MOTOR_SCREW_D, -1, PAD_T + 20))
    body = body.cut(holes([M], PAD_HOLE_D, -1, PAD_T + 20))
    # air channels under the pad, between the screws: less contact with the hot motor flange
    for k in range(4):
        a = au + k * math.pi / 2
        p0 = (M[0] + 4 * math.cos(a), M[1] + 4 * math.sin(a))
        p1 = (M[0] + (PAD_R + 3) * math.cos(a), M[1] + (PAD_R + 3) * math.sin(a))
        ch = Polygon([(p0[0] - 2 * math.sin(a), p0[1] + 2 * math.cos(a)), (p1[0] - 2 * math.sin(a), p1[1] + 2 * math.cos(a)),
                      (p1[0] + 2 * math.sin(a), p1[1] - 2 * math.cos(a)), (p0[0] + 2 * math.sin(a), p0[1] - 2 * math.cos(a))])
        body = body.cut(prism(ch.intersection(Point(M).buffer(PAD_R + 0.5)), -1, 1.0))
    body = body.cut(holes(al.bolts, M3_CLEAR, -1, CAV + 1))
    # zip-tie notches across the top: the arm tapers, so a tie around it would slide towards the motor
    for q in (TIE_Q if style != "fork" else (0.9,)):
        t = al.t0 - q * (al.t0 - LOFT_END_T)
        notch = arm_rect(M, u, n, t - TIE_W / 2, t + TIE_W / 2, -20, 20)
        body = body.cut(prism(notch, arm_h(q) - TIE_D, ROOT_H + 5))
    return body, screws


# ---- ring -------------------------------------------------------------------------------
def ring_profile():
    """(r, z) closed profile: 1.2 wall, 45 deg inlet lip on top, landing flange at the bottom."""
    return [(RI, ZB), (RI, ZT - LIP), (RI + LIP, ZT), (RI + LIP + RW, ZT), (RI + LIP + RW, ZT - 1.0),
            (RI + RW, ZT - 1.0 - LIP), (RI + RW, ZB + 3), (RI + RW + 1.8, ZB + 3), (RI + RW + 1.8, ZB)]


def ring(rl, arm_fp):
    M = rl.M
    body = cq.Workplane("XZ").polyline(ring_profile()).close().revolve(360, (0, 0, 0), (0, 1, 0))
    body = body.translate((M[0], M[1], 0))
    feats = [math.atan2(-rl.sy, 0), math.atan2(0, -rl.sx)]
    # outer ribs, skipping the tabs and the arm notch
    au = math.atan2(M[1] - arm_fp.centroid.y, M[0] - arm_fp.centroid.x) + math.pi
    feats.append(au)
    for k in range(6):
        a = math.radians(30 + 60 * k)
        if min(abs(math.remainder(a - f, 2 * math.pi)) for f in feats) < math.radians(22):
            continue
        r0, r1 = RI + RW - 0.3, RI + RW + 2.2
        c, s = math.cos(a), math.sin(a)
        rib = Polygon([(M[0] + r0 * c - 0.8 * s, M[1] + r0 * s + 0.8 * c), (M[0] + r1 * c - 0.8 * s, M[1] + r1 * s + 0.8 * c),
                       (M[0] + r1 * c + 0.8 * s, M[1] + r1 * s - 0.8 * c), (M[0] + r0 * c + 0.8 * s, M[1] + r0 * s - 0.8 * c)])
        body = body.union(prism(rib, ZB + 3, ZT - LIP - 1.5))
    # spine tab under the floor, with a 45 deg gusset down to the ring wall (prints upright without support)
    zt0, zt1 = -FLOOR - RING_TAB_T + RING_RECESS, -FLOOR + RING_RECESS
    body = body.union(prism(rl.tab, zt0, zt1))
    x0, _, x1, _ = rl.tab.bounds
    yin = min(abs(p[1]) for p in rl.tab.exterior.coords) * rl.sy
    span = abs(rl.y_wall - yin)
    pl = cq.Plane(origin=(x0 + 2, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))
    gusset = cq.Workplane(pl).polyline([(yin, zt0 + 0.01), (rl.y_wall + rl.sy * 1.5, zt0 + 0.01),
                                        (rl.y_wall + rl.sy * 1.5, zt0 - span)]).close().extrude(x1 - x0 - 4)
    body = body.union(gusset.cut(cq.Workplane("XY").add(cq.Solid.makeCylinder(RI, 200, cq.Vector(M[0], M[1], -100)))))
    body = body.cut(holes(rl.bolts, M3_CLEAR, zt0 - 1, zt1 + 1))
    # wells through the gusset for the bolt head, a ø7 washer and the hex key
    body = body.cut(holes(rl.bolts, 7.6, zt0 - span - 1, zt0))
    # bridge tab on top, gusset below it (the gap between the front and rear ring is free)
    body = body.union(prism(rl.bridge_tab, ZT - 4, ZT))
    bx0, by0, bx1, by1 = rl.bridge_tab.bounds
    xw = rl.sx * (abs(bx1 if rl.sx > 0 else bx0) )
    xin = rl.sx * min(abs(bx0), abs(bx1))
    xwall = rl.sx * (RI and (abs(M[0]) - RI - RW))
    plb = cq.Plane(origin=(0, by0 + 2, 0), xDir=(1, 0, 0), normal=(0, -1, 0))
    gb = cq.Workplane(plb).polyline([(xin, ZT - 4 + 0.01), (xwall + rl.sx * 1.5, ZT - 4 + 0.01),
                                     (xwall + rl.sx * 1.5, ZT - 4 - abs(xwall - xin))]).close().extrude(-(by1 - by0 - 4))
    body = body.union(gb.cut(cq.Workplane("XY").add(cq.Solid.makeCylinder(RI, 200, cq.Vector(M[0], M[1], -100)))))
    body = body.cut(holes([rl.bridge_bolt], M3_CLEAR, ZT - 30, ZT + 1))
    # notch where the arm crosses the ring
    body = body.cut(prism(arm_fp.buffer(0.8), 0, ZT + 20))
    return body


def bridge(sy):
    g = box(-11, sy * MY - 7, 11, sy * MY + 7).buffer(-3).buffer(3)
    b = prism(g, ZT, ZT + 3)
    return b.cut(holes([(-6, sy * MY), (6, sy * MY)], M3_CLEAR, ZT - 1, ZT + 4))


# ---- spine plates -------------------------------------------------------------------
def hex_windows(region, r=6.0, pitch=15.0):
    out = []
    x0, y0, x1, y1 = region.bounds
    row = 0
    y = y0
    while y < y1 + pitch:
        x = x0 + (pitch / 2 if row % 2 else 0)
        while x < x1 + pitch:
            h = Point(x, y).buffer(r, 6)  # 24-gon approx is fine; resolution 6 per quarter
            hexa = Polygon([(x + r * math.cos(math.radians(30 + 60 * k)), y + r * math.sin(math.radians(30 + 60 * k))) for k in range(6)])
            if hexa.within(region):
                out.append(hexa)
            x += pitch
        y += pitch * math.sqrt(3) / 2
        row += 1
    return unary_union(out)


def spine(ol, arms, rings):
    pockets = unary_union([a.pocket for a in arms])
    ring_bolts = [b for r in rings for b in r.bolts]
    recesses = unary_union([r.recess for r in rings])
    clamp_bolts = [b for a in arms for b in a.bolts] + ring_bolts
    stack = [(sx * STACK, sy * STACK) for sx in (1, -1) for sy in (1, -1)]
    slots = unary_union([box(*b).buffer(-2).buffer(2) for b in WIRE_SLOTS])
    # box mounting holes: 20 mm grid where nothing else is
    keepout = unary_union([pockets.buffer(6), slots.buffer(5)] + [Point(p).buffer(7) for p in clamp_bolts + stack])
    box_holes = [(x, 0.0) for x in (-80, -20, 20, 80)
                 if Point(x, 0).within(ol.buffer(-7)) and not Point(x, 0).within(keepout)]
    # cavity walls
    rim = ol.difference(ol.buffer(-WALL_RIM))
    pocket_walls = pockets.buffer(WALL, join_style=2).intersection(ol)
    bosses = unary_union([Point(p).buffer(4.5, 32) for p in box_holes] + [Point(p).buffer(5.5, 32) for p in ring_bolts])
    walls = unary_union([rim, pocket_walls, bosses]).difference(pockets)
    # windows where nothing sits on the floor or under the top plate
    busy = unary_union([walls.buffer(1.6), pockets.buffer(2.5), slots.buffer(2.0)]
                       + [Point(p).buffer(5.0) for p in clamp_bolts + stack + box_holes])
    free = ol.buffer(-WALL_RIM - 1.6).difference(busy)
    centre = box(-9.75, -9.75, 9.75, 9.75).buffer(-3).buffer(3)   # over the FC: light and lets heat out
    big = hex_windows(free.difference(centre.buffer(2.4)), r=7.5, pitch=18.0)
    small = hex_windows(free.difference(centre.buffer(2.4)).difference(big.buffer(2.4)), r=4.5, pitch=11.0)
    windows = unary_union([centre.intersection(free), big, small])

    bottom = prism(ol.difference(windows), -FLOOR, 0).union(prism(walls, 0, CAV - CRUSH))
    bottom = bottom.cut(holes(clamp_bolts + stack, M3_CLEAR, -FLOOR - 1, CAV + 1))
    bottom = bottom.cut(holes(box_holes, M3_PILOT, -FLOOR - 1, CAV + 1))
    # hex seats for the nylocks on the underside: the nut cannot turn, tighten with one key from the top
    seats = unary_union([hexagon(p, NUT_AF) for p in clamp_bolts if p not in ring_bolts])
    bottom = bottom.cut(prism(seats, -FLOOR - 1, -FLOOR + NUT_SEAT))
    bottom = bottom.cut(prism(slots, -FLOOR - 1, CAV + 1))
    # ring tabs: locating recess under the floor, captive nylocks in the bosses (dropped in before the top plate)
    bottom = bottom.cut(prism(recesses, -FLOOR - 1, -FLOOR + RING_RECESS))
    bottom = bottom.cut(prism(unary_union([hexagon(p, NUT_AF) for p in ring_bolts]), CAV - 4.3, CAV + 1))

    notches = unary_union([a.gusset.buffer(0.6, join_style=2) for a in arms]).buffer(1.5).buffer(-1.5)
    top = prism(ol.difference(windows).difference(notches).difference(slots), CAV, CAV + TOP)
    top = top.cut(holes(clamp_bolts + box_holes, M3_CLEAR, CAV - 1, CAV + TOP + 1))
    top = top.cut(holes(stack, 7.0, CAV - 1, CAV + TOP + 1))
    info = dict(box_holes=box_holes, windows=len(_polys(windows)), stack=stack)
    return bottom, top, info


def skid_profile(xm):
    """Side profile (x, z) of a skid hanging from bolts at x = +-xm: mount blocks, splayed legs,
    flat runner with upturned tips. Strokes are unioned and the inner corners filleted R5."""
    zb = SKID_Z
    run_h = 7.0
    shapes = [
        LineString([(-50, zb + run_h / 2), (50, zb + run_h / 2)]).buffer(run_h / 2, cap_style=1),
    ]
    for s in (1, -1):
        # upturned tip, thinner
        shapes.append(LineString([(s * 50, zb + run_h / 2), (s * 58, zb + 5.5), (s * 63, zb + 10.0)]).buffer(2.6, cap_style=1))
        # leg: 14 wide at the block, 10 at the runner, splayed out by 10 mm
        top, bot = (s * xm, SKID_BLOCK_Z + 2), (s * (xm + 10), zb + run_h / 2)
        leg = Polygon([(top[0] - 7, top[1]), (top[0] + 7, top[1]), (bot[0] + 5, bot[1]), (bot[0] - 5, bot[1])])
        shapes.append(leg)
        # mount block under the floor
        shapes.append(box(s * xm - 9, SKID_BLOCK_Z, s * xm + 9, -3.0))
    prof = unary_union(shapes).buffer(5, join_style=1).buffer(-5, join_style=1)
    # keep the top dead flat against the floor
    return prof.intersection(box(-100, -100, 100, -3.0))


def skid(sy, bolt_pts):
    """One skid at y = sy * |bolt y|; bolt_pts are the two arm bolts it hangs on."""
    xm = abs(bolt_pts[0][0])
    yc = bolt_pts[0][1]
    prof = skid_profile(xm)
    # plane with normal -y: local x = global x, local y = global z; extrude towards +y
    pl = cq.Plane(origin=(0, yc - SKID_T / 2, 0), xDir=(1, 0, 0), normal=(0, -1, 0))
    pts = [(x, z) for x, z in list(prof.exterior.coords)[:-1]]
    body = cq.Workplane(pl).polyline(pts).close().extrude(-SKID_T)
    for bx, by in bolt_pts:
        body = body.cut(holes([(bx, by)], M3_CLEAR, SKID_BLOCK_Z - 1, 0))
        # nylock slot, open on the outer face
        z0, z1 = SKID_NUT_Z
        y_out = yc + sy * SKID_T / 2
        slot = box(bx - 3.0, min(yc, y_out + sy * 1), bx + 3.0, max(yc, y_out + sy * 1))
        body = body.cut(prism(slot, z0, z1))
    return body


def build_all():
    ol, arms, rings, errs = build_layout()
    parts = {}
    arm_fps = []
    for name, al in zip(("FL", "FR", "RL", "RR"), arms):
        body, screws = arm(al, ARM_STYLE)
        parts[f"arm_{name}"] = body
        al.motor_screws = screws
        arm_fps.append(arm_footprint(al))
    for name, rl, fp in zip(("FL", "FR", "RL", "RR"), rings, arm_fps):
        parts[f"ring_{name}"] = ring(rl, fp)
    parts["bridge_L"] = bridge(1)
    parts["bridge_R"] = bridge(-1)
    bottom, top, info = spine(ol, arms, rings)
    parts["spine_bottom"] = bottom
    parts["spine_top"] = top
    # skids hang on the |x| 38 arm bolts of the front and rear arm on each side
    for name, sy in (("L", 1), ("R", -1)):
        pts = [b for a in arms if a.sy == sy for b in a.bolts if abs(abs(b[0]) - 38.0) < 0.6]
        parts[f"skid_{name}"] = skid(sy, sorted(pts))
    info_sk = [b for a in arms for b in a.bolts if abs(abs(b[0]) - 38.0) < 0.6]
    info["skid_bolts"] = info_sk
    return parts, dict(outline=ol, arms=arms, rings=rings, errors=errs, **info)
