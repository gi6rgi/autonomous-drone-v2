# Frame v3: inverted motors, ducted props, sandwich arms

Started 2026-11-08. Motors hang under the arms, props spin under the motors inside 1.2 mm rings.
A narrow spine runs along X, the box on top of it holds the battery, the Pi 5 and the camera.
Material: **PLA** on the Kobra 2 Neo. All sizes in mm.

```
uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python cadquery shapely trimesh
.venv/bin/python layout.py    # 2D layout + checks (bolt edge distance, pocket walls)
.venv/bin/python check.py     # 3D interference: all parts, motors, prop discs; bed fit
.venv/bin/python export.py    # out/*.stl + out/frame_v3_assembly.step, print/*.stl (print orientation)
.venv/bin/python coupon.py    # joint test coupon -> print/coupon_*.stl
.venv/bin/python ground_check.py  # prop and shaft gap to the floor, level and tilted (after export.py)
```

- `layout.py`: every dimension and the 2D footprints (spine outline, tongues, pockets, bolts).
- `parts.py`: CadQuery solids built from the layout.
- `topbox.py`: the box (battery cradle, deck, camera cassette, balance spacer) and reference blocks of the components.

## Layout
| | Value | Source |
|---|---|---|
| Motors | (+-80, +-96), 160 x 192, wheelbase 250 | chosen so the rings clear the spine |
| Prop | 5.1" = ø129.5, disc plane z = -24 | datasheet / estimate |
| Ring | inner r 67 (tip gap 2.25), wall 1.2, z -36..+6, 45 deg inlet lip | chosen |
| Spine | 200 long, 44..80 wide (56 waist in the middle), follows the rings 2 mm off the lip | chosen |
| Sandwich | floor 3 (z -3..0) + tongue 6 (z 0..6, cavity walls 5.8) + top plate 3 (z 6..9) = 12, fits M3x16 + nylock | chosen |
| Arm | 70 deg to X; loft 28 x 18 at the root -> 15 wide -> 21 x 6 at the pad; tongue 6 thick; two zip-tie notches on top | chosen |
| Motor pad | ø31 x 6, ø12 centre hole, 4 x ø3.3 on ø16, 4 air channels 4 x 1 underneath | motor drawing; 6 mm pad keeps 3 mm of the stock 11 mm screws in the motor (tested on plate v2) |

## Parts (print/)
| File | Qty | Bed | Solid PLA |
|---|---|---|---|
| arm_a | 2 | tongue and pad down, FL and RR | 29 g |
| arm_b | 2 | same, FR and RL | 29 g |
| ring_a | 2 | upright (bottom rim on the bed), FL and RR | 37 g |
| ring_b | 2 | upright, FR and RL | 37 g |
| bridge | 2 | flat | 1 g |
| spine_bottom | 1 | floor down | 55 g |
| spine_top | 1 | flat | 33 g |
| skid | 2 | **PETG**, on its side, nut slots up | 19 g |
| box_cradle | 1 | flat underside down, posts up | 38 g |
| box_deck | 1 | flat underside down, Pi bosses up | 23 g |
| box_camera | 1 | flange down | 4 g |
| box_spacer | 1 | flat (4 mm, balance) | 4 g |

Solid total 461 g. Slicer, first arm (2026-11-08): 16 g, 59 min. Printed with 4 perimeters and 25 % gyroid expect about 300 g (estimate, check the slicer).
Arms: 4 perimeters, 25 % gyroid, 5 top/bottom layers (the loft prints as a thin-wall tube, about 20 g per arm; 4 perimeters around the bolt holes carry the clamp load). Rings: 3 perimeters at 0.4 = solid wall.

## Arm joint
- Hammer-head tongue: neck 25 wide where it leaves the spine, head 34 wide inside. The arm drops in from above,
  the pocket shoulders hold it against pull-out (crash), the bolts only clamp. R1 corners in the pocket.
- The tongue is 6 thick, the cavity walls around it only 5.8: the top plate rests on the tongues, so the bolts
  clamp the arm and not the walls. A tongue that comes out even 0.1 thinner than the walls would sit loose.
  Print arms, spine and coupon at 0.2 mm layers so 3 / 5.8 / 6 mm come out exact. (Until 2026-11-09 the
  0.2 was on the tongue, 6.2; moved to the walls so arms printed at 6 stay valid.)
- 3 bolts per arm in an L: two across the arm right behind the root ramp (they hold the top plate down where
  the thrust lever pushes up), one 10 mm deeper (holds the tail of the tongue down).
- Root moment at full thrust ~1.1 N*m. The clamp needs ~60 N of couple; 3 x M3 in PLA give ~900 N of preload,
  margin x15, about x7 after the PLA creeps. Ring tabs have the same hammer head and 2 bolts.

## Landing skids (PETG)
Two identical skids under the spine, printed in **PETG** (tougher than PLA, it bends instead of cracking).
- Each hangs on two arm clamp bolts (|x| 38, |y| 16.6) of the front and rear arm: those 4 bolts are M3x30,
  the nylock sits in a side slot of the skid. Landing pushes the skid up into the floor, the bolts carry no landing load.
- Runner bottom at z -42: 6 mm below the ring rims, 12 mm below the FC stack. The rings no longer take the landing.
- Profile: mount block 18 x 20, leg 14 -> 10 splayed out 10 mm, runner 7 deep, upturned tips, R5 fillets. 8 thick.
- Print on its side (profile on the bed, nut slots up): layers follow the profile, so landing bends the part along the layers. 5 walls, 30 % gyroid.
- Rough check, hard landing: 30 N in the middle of a runner -> ~6 MPa; 30 N sideways at the runner -> ~8 MPa (PETG ~50).
- Floor clearance (ground_check.py): level on the skids blades 14.5 mm, shaft end/nut 9.0 mm; in any rest position up to 60 deg tilt (skid + ring rim) blades >= 8.7 mm, shaft >= 5.7 mm. Blades sit inside the ring cylinders, so a flat floor can never reach them. Prop height is an estimate (z -24 +- 3.5), measure it.
- Stance is narrow in y (33 mm): on a tilt over ~2 deg the ring rims touch the floor and act as outriggers.

## Rings
- Each ring hangs under the spine: a 5 mm tab sits 1 mm deep in a recess in the floor underside, 2 x M3x16 from below
  into nylocks that are trapped in bosses inside the spine (dropped in before the top plate goes on).
- A 45 deg gusset carries the tab, so the ring prints upright without supports; two ø7.6 wells in the gusset give
  room for the bolt head, a washer and the hex key.
- Bridge tab on top of each ring, also with a gusset; one bridge plate per side joins the front and rear ring.
- To change a ring: 2 bolts from below, 2 bridge bolts, then lower the ring. The arm notch opens upwards, so
  the arm and the spine stay as they are.

## Box (topbox.py)
- The battery lies on the spine, over the FC stack: the spine, the stack and the skids are between it and the floor.
  Cradle top z 16, 1 mm rubber pad, battery 140 x 43 x 38 at z 17..55, 1..1.5 mm over the arm ramps.
- Battery out: unplug the XT60 (rear), open two 20 mm straps, lift the tail 3 mm over the rear lip, slide back.
  The straps loop under the rim of the cradle at x +-16 (21 mm grooves on the underside).
- Loads: forward into the stop wall (with a gusset, screwed to the spine), back against the lip + straps,
  sideways the guides in the middle and the arm roots at the ends. A strap alone gives ~30 N of friction, a crash ~130-200 N.
- 146 mm between the stop wall and the lip: a spacer (0..6 mm, 2 x M3 through the wall) sets the balance of this pack
  (estimate: CG 3 mm forward -> 4 mm spacer), a thicker one holds a shorter pack. Packs up to 41 mm high fit under the deck.
- Deck z 60..62.5 on 4 posts (+-30, +-27) and two bosses in front of the stop wall. Pi 5 on printed bosses
  (M2.5 self-tapping in 2.2 pilots), USB/Ethernet forward, GPIO back. Camera cassette on the nose, 15 deg down.
- HW-636 hangs under the spine in front of the stack (x 25..101, z -29..-8, 13 mm over the floor on the skids):
  on the deck it would sit on a long rear overhang (~30 Hz).
- Wires: front slot in the spine nose (Pi, camera, HW-636) and a window in the deck in front of the stop wall;
  rear slot for the battery lead to the XT60 behind the cradle.
- Rough mass: printed ~330 g, all-up ~1.3 kg with a 650 g pack, CG ~44 mm above the prop plane (estimate).

## Fasteners
| Where | Qty | Stack | Bolt |
|---|---|---|---|
| Arm tongue clamp | 12 (3 per arm) | top 3 + tongue 6 + floor 3 | M3x16 button head, washer, nylock in a hex seat; the 4 bolts at abs(x) 38, abs(y) 16.6 are M3x30 and also hold the skids |
| Ring tab | 8 | tab 5 (1 in a recess) + floor + boss | M3x16 from below through the well in the gusset, washer, captive nylock inside the spine |
| Bridges | 4 | bridge 3 + ring tab 4 | M3x16, nylock |
| Cradle to spine | 4 | cradle 3.8 under the head + top 3 + boss 6 + floor 3 | M3x16 self-tapping into ø2.8 pilots, ends flush with the floor |
| Deck | 6 | deck 2.5 + post / wall boss | M3x16 self-tapping |
| Camera, spacer | 4 | flange + deck, spacer + wall | M3x16 + nylock |
| Pi 5 | 4 | printed boss 7 | M2.5 x 8-10 self-tapping |
| Motors | 16 | pad 6 | stock 11 mm screws |
| FC/ESC stack | 4 | floor 3 | from the top, through the ø7 access holes in the top plate |

## Assembly
1. Drop the ring nylocks into their hex pockets in the bottom plate bosses, lay the arm tongues into their pockets.
2. Nylocks into the hex seats under the floor, top plate on, arm clamp bolts M3x16 from the top (one hex key).
   The 4 bolts at abs(x) 38, abs(y) 16.6 are M3x30 and go on through the skids.
3. Rings from below: tab into its recess, 2 x M3x16 up through the wells. Bridge plates on top, 2 x M3x16 each.
4. Retighten all clamp bolts after the first 5 flights (PLA creeps under the bolt heads).

Changing an arm means taking the box and the top plate off. That is the price of the sandwich.

## Joint test coupon (coupon.py)
Three joints on one block: S sandwich (same hammer head, L of 3 bolts and nut seats as the frame), A slip dovetail, B wedge dovetail
(2 mm taper over 18 mm, the bolt pulls the arm up into the wedge). Same 16 x 26 x 60 stub on each arm.
Clamp the flange to a table edge so the block and the arms overhang it.

Tests per arm:
1. Hang 1.5 kg (water bottle) from the end hole: measure the drop, check it comes back to zero.
2. Tape a laser pointer to the end block, wall 3 m away: push the stub lightly up/down and sideways. A dot that jumps = play.
3. Tap the end block with a pencil, record with the phone, look at the spectrum: one clean peak that rings = good, a dull buzz = play.

Fasteners: S 3 x M3x16 + nylock, A 1 x M3x16 from below, B 1 x M3x20 from below.

## Open points
- XT60 holder at the rear, flow sensor under the spine behind the stack: not modelled yet.
- Battery mass and the real CG: weigh the pack, hang the drone by two points to check the balance, pick the spacer.
- Motor wires: outside, under the arm, two zip ties sit in the notches on top (the arm tapers, a tie without a notch slides to the motor). Easy to swap a motor.
- Arm stiffness and joint play: measure on the coupon before printing the full frame.
