"""Interference and clearance checks for the v3 assembly. Run: .venv/bin/python check.py"""
import itertools
import math

import cadquery as cq

from layout import CAV, MY, PROP_R, PROP_Z, RI, ZB
from parts import build_all


def motor(M):
    flange = cq.Workplane("XY").center(*M).circle(14.0).extrude(-2.3)
    bell = cq.Workplane("XY").workplane(offset=-2.3).center(*M).circle(13.75).extrude(-16.9)
    return flange.union(bell)


def prop(M):
    # swept disc of the blades plus hub, z PROP_Z +-3.5
    return cq.Workplane("XY").workplane(offset=PROP_Z - 3.5).center(*M).circle(PROP_R).extrude(7.0)


def vol(a, b):
    try:
        return a.intersect(b).val().Volume()
    except Exception:
        return 0.0


def run(tol=0.5):
    parts, info = build_all()
    ok = True
    print("layout errors:", info["errors"] or "none")
    names = list(parts)
    for a, b in itertools.combinations(names, 2):
        A, B = parts[a].val().BoundingBox(), parts[b].val().BoundingBox()
        if A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin:
            continue
        v = vol(parts[a], parts[b])
        if v > tol:
            ok = False
            print(f"CLASH {a} x {b}: {v:.2f} mm3")
    for al in info["arms"]:
        m, p = motor(al.M), prop(al.M)
        for n, s in parts.items():
            for what, solid in (("motor", m), ("prop", p)):
                v = vol(solid, s)
                if v > tol:
                    ok = False
                    print(f"CLASH {what}@{al.M} x {n}: {v:.2f} mm3")
    # belly: FC/ESC stack (46 x 44 x 27 under the floor) and the nylocks that stay under the floor
    stack = cq.Workplane("XY").workplane(offset=-30).center(0, 0).rect(46, 44).extrude(27)
    skid_bolts = {(round(x, 1), round(y, 1)) for x, y in info.get("skid_bolts", [])}
    nuts = [b for a in info["arms"] for b in a.bolts]   # ring nylocks are captive inside the spine
    nuts = [b for b in nuts if (round(b[0], 1), round(b[1], 1)) not in skid_bolts]
    for n, s in parts.items():
        v = vol(stack, s)
        if v > tol:
            ok = False
            print(f"CLASH FC stack x {n}: {v:.2f} mm3")
        if n.startswith("skid"):
            for b in nuts:
                nut = cq.Workplane("XY").workplane(offset=-5.8).center(*b).polygon(6, 6.4).extrude(4.0)
                v = vol(nut, s)
                if v > tol:
                    ok = False
                    print(f"CLASH nylock at {b} x {n}: {v:.2f} mm3")
    ok = check_box(parts, info, tol) and ok
    low = min(s.val().BoundingBox().zmin for n, s in parts.items())
    print(f"lowest point z = {low:.1f} (ring rims at {ZB}, stack bottom at -30)")
    print(f"prop tip to ring wall: {RI - PROP_R:.2f} mm, prop plane {PROP_Z - ZB:.1f} mm above the ring bottom")
    for n, s in parts.items():
        bb = s.val().BoundingBox()
        big = max(bb.xlen, bb.ylen)
        flag = "" if big <= 210 else "  <-- does not fit 220 bed with margin"
        print(f"  {n:13s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f}{flag}")
    print("ALL CLEAR" if ok else "PROBLEMS FOUND")
    return ok


def check_box(parts, info, tol):
    """Box parts vs the frame, the clamp bolt heads, the components, and the battery slide-out path."""
    import topbox as tb
    ok = True
    bolts = [b for a in info["arms"] for b in a.bolts]
    box_parts = tb.build_box(bolts)
    refs = tb.refs()
    heads = cq.Workplane("XY")
    for b in bolts:
        heads = heads.union(cq.Workplane("XY").workplane(offset=tb.ZS).center(*b).circle(3.5).extrude(2.2))
    frame = dict(parts, bolt_heads=heads)
    for bn, bs in box_parts.items():
        for n, s in frame.items():
            v = vol(bs, s)
            if v > tol:
                ok = False
                print(f"CLASH {bn} x {n}: {v:.2f} mm3")
    for a, b in itertools.combinations(box_parts, 2):
        v = vol(box_parts[a], box_parts[b])
        if v > tol:
            ok = False
            print(f"CLASH {a} x {b}: {v:.2f} mm3")
    # components: zero tolerance against everything printed
    for rn, rs in refs.items():
        for n, s in {**frame, **box_parts}.items():
            v = vol(rs, s)
            if v > 0.05:
                ok = False
                print(f"CLASH {rn} x {n}: {v:.2f} mm3")
    # slide-out: the pack lifted by LIFT and moved back past the spine end
    L, W, H = tb.BAT
    bx = tb.FRONT_X - tb.SPACER_T - 0.5 - L / 2
    path = cq.Workplane("XY").workplane(offset=tb.BAT_Z + tb.LIFT).center(bx - 120, 0).rect(L + 100, W).extrude(H)
    for n, s in {**frame, **box_parts}.items():
        v = vol(path, s)
        if v > 0.05:
            ok = False
            print(f"CLASH battery slide-out path x {n}: {v:.2f} mm3")
    # clearance of the battery over the arm ramps
    for k, a in enumerate(info["arms"]):
        name = ("arm_FL", "arm_FR", "arm_RL", "arm_RR")[k]
        for dz in (0.5, 1.0, 1.5):
            grown = cq.Workplane("XY").workplane(offset=tb.BAT_Z - dz).center(bx, 0).rect(L + 12, W + 2 * dz).extrude(H)
            if vol(grown, parts[name]) > 0.01:
                print(f"  battery to {name}: less than {dz} mm")
                break
    print(f"box: battery z {tb.BAT_Z}..{tb.BAT_Z + H}, deck z {tb.DECK_Z}, slide-out path checked")
    return ok


if __name__ == "__main__":
    run()
