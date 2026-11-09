"""Joint test coupon: three arm joints side by side on one spine block.

  S  sandwich: hammer-head tongue in a pocket, top plate, 3 x M3 + nylock in hex seats (same as the frame)
  A  slip dovetail: two dovetails slide up from below to a stop, 1 x M3 from below
  B  wedge dovetail: same, but the dovetails taper 2 mm over 18 mm, the bolt pulls the arm into the wedge

Every test arm has the same 16 x 26 stub, 60 mm long, and an end block with a ø6 hole
(hang a weight) and a flat top (tape a laser pointer). Clamp the flange to a table edge,
the block and the arms hang over the edge.

Coupon frame: block face (arm side) at y = 0, arms point to +y, bed at z = -3.
Run: .venv/bin/python coupon.py   ->  print/coupon_*.stl
"""
from pathlib import Path

import cadquery as cq
from cadquery import exporters
from shapely.geometry import box

from layout import CAV, CLR, CRUSH, DENSITY_PLA, M3_CLEAR, M3_PILOT, NECK_HW, NUT_AF, NUT_SEAT, TONGUE_HW, TOP
from parts import hexagon, holes, prism
from shapely.ops import unary_union

Z0 = -3.0                 # bed
BLOCK_TOP = 18.0
GROOVE = dict(a=6.0, b=9.0, d=3.0, top=15.0)   # dovetail: width at the face, at depth d, blind end
TAPER = 2.0               # wedge: width loss over the groove length
C_SLIP, C_WEDGE = 0.15, 0.10
STUB_W, STUB_H, STUB_L = 16.0, 26.0, 60.0
STATIONS = {"S": 25.0, "A": 75.0, "B": 125.0}  # x centres

HERE = Path(__file__).parent


def trapezoid(gx, a, b, d, z, extra=0.3):
    return [(gx - a / 2, extra, z), (gx + a / 2, extra, z), (gx + a / 2, 0, z), (gx + b / 2, -d, z),
            (gx - b / 2, -d, z), (gx - a / 2, 0, z)]


def dovetail(gx, z_lo, z_hi, a_lo, b_lo, a_hi, b_hi, d, extra):
    w0 = cq.Wire.makePolygon([cq.Vector(*p) for p in trapezoid(gx, a_lo, b_lo, d, z_lo, extra)], close=True)
    w1 = cq.Wire.makePolygon([cq.Vector(*p) for p in trapezoid(gx, a_hi, b_hi, d, z_hi, extra)], close=True)
    return cq.Workplane("XY").add(cq.Solid.makeLoft([w0, w1], ruled=True))


def female_w(z, taper):
    k = taper * (z - Z0) / (GROOVE["top"] - Z0)
    return GROOVE["a"] - k, GROOVE["b"] - k


def stub(xc, z_bed, y0):
    """Test stub and end block, standing on the bed at z_bed."""
    s = cq.Workplane("XY").workplane(offset=z_bed).center(xc, y0 + STUB_L / 2).rect(STUB_W, STUB_L).extrude(STUB_H) \
        .edges("|Y and >Z").fillet(4)
    end = cq.Workplane("XY").workplane(offset=z_bed).center(xc, y0 + STUB_L + 7.5).rect(22, 15).extrude(STUB_H) \
        .faces(">Z").workplane().center(0, 0).hole(6.0)
    return s.union(end)


def base():
    flange = prism(box(0, -70, 150, -30).buffer(-4).buffer(4), Z0, 0)
    flange = flange.cut(prism(box(20, -60, 40, -50).buffer(-2).buffer(2), Z0 - 1, 1))
    flange = flange.cut(prism(box(110, -60, 130, -50).buffer(-2).buffer(2), Z0 - 1, 1))
    xs = STATIONS["S"]
    s_block = prism(box(xs - 25, -35, xs + 25, 0), Z0, CAV - CRUSH)   # 0.2 under the tongue
    s_block = s_block.cut(prism(s_tongue().buffer(CLR, join_style=2).union(box(xs - NECK_HW - CLR, -1, xs + NECK_HW + CLR, 1)), 0, CAV + 1))
    s_block = s_block.cut(holes(s_bolts(), M3_CLEAR, Z0 - 1, CAV + 1))
    s_block = s_block.cut(prism(unary_union([hexagon(b, NUT_AF) for b in s_bolts()]), Z0 - 1, Z0 + NUT_SEAT))
    body = flange.union(s_block)
    for key, taper in (("A", 0.0), ("B", TAPER)):
        xc = STATIONS[key]
        blk = prism(box(xc - 25, -30, xc + 25, 0), Z0, BLOCK_TOP)
        for gx in (xc - 11, xc + 11):
            a0, b0 = female_w(Z0 - 0.5, taper)
            a1, b1 = female_w(GROOVE["top"], taper)
            blk = blk.cut(dovetail(gx, Z0 - 0.5, GROOVE["top"], a0, b0, a1, b1, GROOVE["d"], 0.5))
        blk = blk.cut(holes([(xc, -12)], M3_PILOT, Z0 - 1, Z0 + 14))
        body = body.union(blk)
    return body


def s_tongue():
    """Hammer head like the frame: neck 25 wide at the edge, head 34 wide inside, R1 corners."""
    xs = STATIONS["S"]
    return unary_union([box(xs - NECK_HW, -12, xs + NECK_HW, 0.5),
                        box(xs - TONGUE_HW, -31, xs + TONGUE_HW, -12)]).buffer(-1).buffer(1)


def s_bolts():
    """Same L pattern as the frame: a pair across the arm 15 mm in, one 10 mm deeper."""
    xs = STATIONS["S"]
    return [(xs - 12.3, -16.5), (xs + 12.3, -16.5), (xs + 12.3, -26.0)]


def s_top():
    xs = STATIONS["S"]
    p = prism(box(xs - 25, -35, xs + 25, 0), CAV, CAV + TOP)   # rests on the tongue
    return p.cut(holes(s_bolts(), M3_CLEAR, CAV - 1, CAV + TOP + 1))


def arm_s():
    xs = STATIONS["S"]
    tongue = prism(s_tongue(), 0, CAV)
    body = tongue.union(stub(xs, 0, 0.5))
    return body.cut(holes(s_bolts(), M3_CLEAR, -1, CAV + 1))


def arm_dovetail(key):
    xc = STATIONS[key]
    wedge = key == "B"
    c = C_WEDGE if wedge else C_SLIP
    foot_gap = 6.5 if wedge else 0.5
    foot_top = Z0 - foot_gap
    foot_bot = foot_top - 4
    male_top = (GROOVE["top"] - 6.0) if wedge else GROOVE["top"]
    root = prism(box(xc - 16, 0, xc + 16, 5), foot_bot, male_top + (BLOCK_TOP - GROOVE["top"]))
    foot = prism(box(xc - 10, -20, xc + 10, 5), foot_bot, foot_top)
    body = root.union(foot)
    taper = TAPER if wedge else 0.0
    for gx in (xc - 11, xc + 11):
        a0, b0 = female_w(foot_top, taper)
        a1, b1 = female_w(male_top, taper)
        body = body.union(dovetail(gx, foot_top, male_top, a0 - 2 * c, b0 - 2 * c, a1 - 2 * c, b1 - 2 * c,
                                   GROOVE["d"] - c, 0.5))
    body = body.union(stub(xc, foot_bot, 5))
    return body.cut(holes([(xc, -12)], M3_CLEAR, foot_bot - 1, foot_top + 1))


def main():
    out = HERE / "print"
    out.mkdir(exist_ok=True)
    parts = {"coupon_base": base(), "coupon_top_S": s_top(), "coupon_arm_S": arm_s(),
             "coupon_arm_A": arm_dovetail("A"), "coupon_arm_B": arm_dovetail("B")}
    # assembly-position check: nothing may overlap
    names = list(parts)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            try:
                v = parts[names[i]].intersect(parts[names[j]]).val().Volume()
            except Exception:
                v = 0.0
            if v > 0.3:
                print(f"CLASH {names[i]} x {names[j]}: {v:.2f} mm3")
    for n, wp in parts.items():
        s = wp.val()
        bb = s.BoundingBox()
        s = s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))
        exporters.export(cq.Workplane("XY").add(s), str(out / f"{n}.stl"), tolerance=0.05, angularTolerance=0.2)
        print(f"{n:14s} valid={s.isValid()} {bb.xlen:5.1f} x {bb.ylen:5.1f} x {bb.zlen:5.1f}  {s.Volume() * DENSITY_PLA:5.1f} g")
    asm = HERE / "out"
    asm.mkdir(exist_ok=True)
    for n, wp in parts.items():
        exporters.export(wp, str(asm / f"{n}.stl"), tolerance=0.05, angularTolerance=0.2)


if __name__ == "__main__":
    main()
