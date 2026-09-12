# Frame, parametric models (CadQuery)

Two tracks:
1. **Drone v2** (current since 2026-09-04): clean plate `plate_v2.py` (stack, B, camera as on the printed plate; motors spread along Y to wheelbase 245) + `accessories.py` (deck, legs, GPS mast). Printing: `PRINT.md`, files in `print/`.
   `plate_asprinted.py` is the plate reconstructed from gcode, reference only.
2. Frame v1 from scratch: `build.py` (on hold).

```
uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python cadquery
.venv/bin/python params.py        # parameter consistency checks
.venv/bin/python build.py         # STL/STEP per part + assembly → out/
.venv/bin/python plate_v2.py           # plate v2 → out/plate_v2.step/stl
.venv/bin/python accessories.py        # deck + legs + GPS mast, interference check → out/acc_*.stl
.venv/bin/python check.py              # hole alignment, standoff fit, clearances
.venv/bin/python export_print.py       # all parts in print orientation → print/*.stl
.venv/bin/python viewer_data.py out/viewer.json [build|accessories]   # data for the web viewer
```

- `params.py`: all component and frame dimensions (the only place to edit).
- `build.py`: part geometry, assembly, interference check, export.
- `out/`: generated, keep out of git (except `frame_v1_assembly.step` if you want).

## Parts for the printed plate (accessories.py)
| File | Part | Qty | Mounting | Print |
|---|---|---|---|---|
| plate_v2 | plate 213 × 200 (motors 180 × 167, wheelbase 245), 6 mm: octagonal center box with gussets, arms with 14 mm waist and 6 × 6 rib, oval leg mounts, concave R8 transitions, battery zip tie slots | 1 | none | flat |
| camera_mount | camera cradle: 25×24 board in a slot between the walls (no screws), 15° tilt, flange with M4 nuts and a hook | 1 | 2 × M4×8 from under the plate | flange down |
| deck | ladder deck 129 × 58 × 3 + ribs: rails through the standoff points, tabs for Pi 5 (58 × 49, M2.5), pad for the mast, 17.5 g | 1 | 4 printed 35 mm standoffs on B (±34, ±19) | flat |
| leg | blade leg 72 mm, 12° tilt, all identical | 4 | 2 M3 bolts from below either side of the rib (mount A) + zip tie | flange down, no supports |
| gps_mount | 40 mm mast (ø12 tube, hollow cone) with ø55 cup ring for the GPS puck | 1 | 4 × M4×8 from below through the deck into nuts in the flange | standing |
| standoff_35 | deck standoff 35 mm, 11 mm hex, M4 nuts in pockets | 4 | M4×8 top and bottom (B via counterbore) | standing |

Plate: `docs/components/plate-asprinted.md`.

## v1 parts (build.py, on hold)
| File | Part | Qty | Print orientation |
|---|---|---|---|
| arm | arm | 4 | flat, 100 % infill |
| plate_A | lower base plate, 6 mm, heat-set inserts for the tray | 1 | flat |
| plate_B | upper base plate, 4 mm | 1 | flat |
| deck_E | Pi 5 deck, 130 × 90 × 3.5 | 1 | flat |
| tray_H | battery tray 150 × 60 × 3.5 with strap slots | 1 | flat |
| leg_I | leg 95 mm, flange on heat-set inserts in the arm | 4 | on its side |
| camera_F | camera bracket, 20° tilt | 1 | flange down (needs supports under the plate) |
| mast_G | GPS mast 40 mm | 1 | standing |

## v1 hardware
- Arm clamp: 12 × M3×22 through B–arm–A + nyloc nuts under A.
- Stack: 4 × M3 standoffs/bolts on plate B (30.5 pattern).
- Deck E: 4 M3×28 standoffs on B (±40 pattern on the axes) + 4 bolts.
- Tray H: 4 × M3×8 from below into heat-set inserts in A (80 × 40 pattern).
- Legs: 8 × M3×8 into heat-set inserts in the arms.
- Heat-set inserts M3×5.7: 4 (A) + 8 (arms) = 12.
