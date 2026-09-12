"""Assembly checks: hole alignment, standoff seating, clearances. Run: .venv/bin/python check.py"""
import math
import cadquery as cq
import accessories as A
import plate_v2 as P
from params import PI5, GPS, PROP, MOTOR, CAM

asm, _ = A.assembly()
bodies = {n: s.moved(l) for n, s, l in A._walk(asm)}


def probe(name, x, y, z0, z1, d=2.6):
    """Volume of material of body name inside a vertical cylinder ø d at (x,y) from z0 to z1."""
    cyl = cq.Workplane("XY").center(x, y).circle(d / 2).extrude(z1 - z0).translate((0, 0, z0)).val()
    try:
        return bodies[name].intersect(cyl).Volume()
    except Exception:
        return -1


fails = 0


def expect_free(name, x, y, z0, z1, label, d=2.6):
    global fails
    v = probe(name, x, y, z0, z1, d=d)
    ok = abs(v) < 0.5
    fails += (not ok)
    print(f"{'ok ' if ok else 'FAIL'} {label:55s} {name:10s} ({x:7.1f},{y:7.1f})  material in hole: {v:.1f} mm³")


def expect_material(name, x, y, z0, z1, label, d=5.5):
    global fails
    cyl = cq.Workplane("XY").center(x, y).circle(d / 2).extrude(z1 - z0).translate((0, 0, z0)).val()
    full = cyl.Volume()
    v = bodies[name].intersect(cyl).Volume()
    ok = v > 0.98 * full - 0.5
    fails += (not ok)
    print(f"{'ok ' if ok else 'FAIL'} {label:55s} {name:10s} ({x:7.1f},{y:7.1f})  support: {v/full*100:.0f} %")


print("== legs: hole pairs on both sides of the rib through flange and plate")
pts = P.leg_hole_pts()
for i, (x, y) in enumerate(A.LEG_MOUNTS):
    for j, (hx, hy) in enumerate(pts[2 * i:2 * i + 2]):
        expect_free(f"leg{i}", hx, hy, -6, 4, f"leg leg{i}: hole {j}")
        expect_free("plate", hx, hy, -1, 14, f"plate: leg hole {j} (rib not hit)")
    # channel between side ribs above the leg mount is free (motor wires pass); with the old rib the rib is continuous
    if P.ARM_STIFFENER == "walls":
        expect_free("plate", x, y, P.PLATE_T + 0.05, P.PLATE_T + P.WALL_H, "channel between side ribs above leg mount is free", d=6.0)
    else:
        expect_material("plate", x, y, P.PLATE_T + 0.5, P.PLATE_T + P.ARM_RIB_H - 0.5, "arm rib above leg mount is continuous", d=3.0)

print("== deck standoffs: hole in plate and deck, flat support without rib")
for i, (x, y) in enumerate(A.DECK_MOUNTS):
    expect_free("plate", x, y, -1, 7, f"plate: standoff hole {i}")
    expect_free("deck", x, y, A.Z_DECK - 1, A.Z_DECK + 5, f"deck: standoff hole {i}")
    # support: cylinder ø5.5 from plate top (6) to rib top (8.4) must be EMPTY (rib does not enter)
    v = probe("plate", x, y, P.PLATE_T + 0.05, P.PLATE_T + P.RIB_H, d=6.0)
    ok = v < 0.5; fails += (not ok)
    print(f"{'ok ' if ok else 'FAIL'} {'rib does not obstruct standoff '+str(i):55s} plate      ({x:7.1f},{y:7.1f})  rib under standoff: {v:.1f} mm³")

print("== battery strap slots: free, strap on top passes clear of standoffs and stack")
for x, y in P.TIE_SLOTS:
    expect_free("plate", x, y, -1, 14, "plate: strap slot", )
    # strap path over the plate top: 5 mm wide band along Y from −y to +y at height 6…9 mm must not hit standoffs/stack
    band = cq.Workplane("XY").center(x, 0).rect(5, 2 * abs(y)).extrude(2).translate((0, 0, P.PLATE_T)).val()
    for n in [k for k in bodies if k.startswith("standoff") or k == "stack"]:
        v = bodies[n].intersect(band).Volume()
        if v > 0.5:
            fails += 1; print(f"FAIL strap at x={x:.0f} hits {n}: {v:.0f} mm³")

print("== standoffs: through hole along the axis")
for i, (x, y) in enumerate(A.DECK_MOUNTS):
    expect_free(f"standoff{i}", x, y, P.PLATE_T + 0.5, A.Z_DECK - 0.5, f"standoff {i}: M3 hole along the full length")

print("== GPS mast ↔ deck, Pi 5 ↔ deck")
for sx in (-1, 1):
    for sy in (-1, 1):
        x, y = A.MAST_POS[0] + sx * 10, A.MAST_POS[1] + sy * 10
        expect_free("gps_mount", x, y, A.Z_DECK + DECK_T - 1 if False else A.Z_DECK + A.DECK_T - 1, A.Z_DECK + A.DECK_T + 4, "mast: flange hole")
        expect_free("deck", x, y, A.Z_DECK - 1, A.Z_DECK + 5, "deck: mast hole")
        px, py = PI5.hole_pitch
        expect_free("deck", A.PI_CENTER[0] + sx * px / 2, A.PI_CENTER[1] + sy * py / 2, A.Z_DECK - 1, A.Z_DECK + 5, "deck: Pi 5 hole")

print("== motors: 4 holes on ø16 (opposite ones 16 mm apart), walls closed")
for mx, my in P.MOTORS:
    pts = [(x, y) for x, y in P.motor_hole_pts() if abs(x - mx) < 10 and abs(y - my) < 10]
    d_opp = math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
    ok = abs(d_opp - 16.0) < 0.05; fails += (not ok)
    print(f"{'ok ' if ok else 'FAIL'} {'motor: distance between opposite holes':55s} plate      ({mx:7.1f},{my:7.1f})  {d_opp:.2f} mm (drawing: 16±0.1)")
    for x, y in pts:
        expect_free("plate", x, y, -1, 7, "plate: motor hole")
        ang = math.atan2(y - my, x - mx)
        r = math.hypot(x - mx, y - my) + 1.7
        expect_material("plate", mx + (r + 2.0) * math.cos(ang), my + (r + 2.0) * math.sin(ang), 0.5, P.PLATE_T - 0.5,
                        "plate: wall outside motor hole", d=1.5)
        expect_material("plate", mx + (P.MOTOR_CENTER_HOLE / 2 + 0.8) * math.cos(ang), my + (P.MOTOR_CENTER_HOLE / 2 + 0.8) * math.sin(ang),
                        0.3, P.PLATE_T - 0.5, "plate: bridge between center and screw counterbore", d=1.2)

print("== stack: counterbores from below for bolt heads are free")
for x, y in P.STACK:
    expect_free("plate", x, y, 0.2, P.STACK_CBORE[1] - 0.2, "plate: stack bolt counterbore", d=P.STACK_CBORE[0] - 0.4)
    expect_material("plate", x + 4.5, y, P.STACK_CBORE[1] + 0.3, P.PLATE_T - 0.3, "plate: wall next to stack counterbore", d=1.5)

print("== motor screws: engagement in the motor")
eng = (MOTOR.stock_screw_len - 2.0) - (P.PLATE_T - P.MOTOR_CBORE[1])
ok = 3.0 <= eng <= 4.0; fails += (not ok)
print(f"{'ok ' if ok else 'FAIL'} stock screw {MOTOR.stock_screw_len} mm: pad {P.PLATE_T - P.MOTOR_CBORE[1]:.1f} mm under the head -> into motor ≈ {eng:.1f} mm (need 3…4: with 5 mm the screw hits the winding, verified 2026-09-05)")
for x, y in (P.motor_hole_pts() if P.MOTOR_CBORE[1] > 0 else []):
    v = probe("plate", x, y, 0.2, P.MOTOR_CBORE[1] - 0.2, d=6.0)
    ok = v < 0.5; fails += (not ok)
    if not ok:
        print(f"FAIL motor counterbore not free ({x:.0f},{y:.0f}): {v:.0f} mm³")

print("== camera: board groove free, lens window and connector relief open; flange bolts")
c, pcb = A.CAM_CRADLE, A.CAM_PCB
t = math.radians(c["tilt_deg"])
def local_to_world(lx, ly, lz):
    return (A.CAM_X0 + lx * math.cos(t) + lz * math.sin(t), ly, P.PLATE_T + A.CAM_Z0 - lx * math.sin(t) + lz * math.cos(t))
def probe_box(lx, ly, lz, sx, sy, sz):
    b = (cq.Workplane("XY").center(sx / 2, 0).rect(sx, sy).extrude(sz).translate((lx, ly, lz))
         .rotate((0, 0, 0), (0, 1, 0), c["tilt_deg"]).translate((A.CAM_X0, 0, P.PLATE_T + A.CAM_Z0)).val())
    return bodies["camera_mount"].intersect(b).Volume()
v = probe_box(c["wall"] + 0.1, 0, 0.5, c["groove_t"] - 0.2, pcb["w"] + 0.2, c["h"] - 3.5)     # board in the groove (up to the detents)
ok = v < 0.5; fails += (not ok); print(f"{'ok ' if ok else 'FAIL'} {'cradle: board groove 25 × 1.5 is free':55s} {v:.1f} mm³")
v = probe_box(c["wall"] + c["groove_t"] + 0.2, 0, 3.0, 5.0, pcb["w"] - 0.4, c["h"] - 3.5)   # open in front of the board (above the flange)
ok = v < 0.5; fails += (not ok); print(f"{'ok ' if ok else 'FAIL'} {'cradle: lens window open':55s} {v:.1f} mm³")
zr = max(c["back_band"], c["hole_z0"] + c["holes"][1] + c["boss_d"] / 2) + 0.5          # above the bosses
v = probe_box(-6.0 - c["boss_h"], 0, zr, 6.0 + c["boss_h"], c["back_relief_w"] - 0.4, c["h"] - zr)
ok = v < 0.5; fails += (not ok); print(f"{'ok ' if ok else 'FAIL'} {'cradle: FPC connector relief open':55s} {v:.1f} mm³")
hx, hz = c["holes"]
for hy, hzz in [(sy * hx / 2, c["hole_z0"] + k * hz) for sy in (-1, 1) for k in (0, 1)]:
    b = (cq.Workplane("YZ").center(hy, hzz).circle(0.7).extrude(c["boss_h"] + A._CAM_T + 6).translate((-c["boss_h"], 0, 0))
         .rotate((0, 0, 0), (0, 1, 0), c["tilt_deg"]).translate((A.CAM_X0, 0, P.PLATE_T + A.CAM_Z0)).val())
    v = bodies["camera_mount"].intersect(b).Volume(); ok = v < 0.5; fails += (not ok)
    print(f"{'ok ' if ok else 'FAIL'} {'cradle: M2 screw path from the front through the pilot is free':55s} (y={hy:6.1f}, z={hzz:5.1f})  {v:.1f} mm³")
for x, y in P.CAM_MOUNT:                                                      # socket clearance over the nuts
    sock = cq.Workplane("XY").center(x, y).circle(A.CAM_NUT_CLEAR - 1.0).extrude(20).translate((0, 0, P.PLATE_T + A.CAM_FL["t"] + 0.1)).val()
    v = bodies["camera_mount"].intersect(sock).Volume(); ok = v < 0.5; fails += (not ok)
    print(f"{'ok ' if ok else 'FAIL'} {'cradle: 10 mm socket fits over the nut':55s} ({x:5.1f},{y:5.1f})  {v:.1f} mm³")
for x, y in P.CAM_MOUNT:
    expect_free("plate", x, y, -1, 7, "plate: camera bracket hole")
    expect_free("camera_mount", x, y, P.PLATE_T - 1, P.PLATE_T + A.CAM_FL["t"] - 0.5, "cradle: M4 hole (inside the flange)")

print("== prop clearances and printer bed")
gx, gy = 2 * P.MOTOR_X - PROP.diameter, 2 * P.MOTOR_Y - PROP.diameter
print(f"{'ok ' if min(gx, gy) >= 15 else 'FAIL'} prop tip clearance: along X {gx:.1f} mm, along Y {gy:.1f} mm (need ≥ 15)")
bb = bodies["plate"].BoundingBox()
print(f"{'ok ' if bb.xlen <= 220 and bb.ylen <= 220 else 'FAIL'} plate {bb.xlen:.0f} × {bb.ylen:.0f} on a 220 × 220 bed")
print("== prop zone: no part enters the cylinder r = R_prop + 5 mm, z = prop plane ± 6 mm")
prop_z = P.PLATE_T + MOTOR.body_h + 3
for mx, my in P.MOTORS:
    sweep = (cq.Workplane("XY").center(mx, my).circle(PROP.diameter / 2 + 5).extrude(12)
             .translate((0, 0, prop_z - 6)).val())
    for name, body in bodies.items():
        if name.startswith(("prop", "motor")):
            continue
        try:
            v = body.intersect(sweep).Volume()
        except Exception:
            v = -1
        if v > 0.5:
            fails += 1
            print(f"FAIL {name} enters prop zone ({mx:.0f},{my:.0f}): {v:.0f} mm³")
print(f"bodies checked: {len(bodies)}; prop plane z = {prop_z:.1f}, deck from {A.Z_DECK:.1f}")
print(f"\nTOTAL errors: {fails}")
