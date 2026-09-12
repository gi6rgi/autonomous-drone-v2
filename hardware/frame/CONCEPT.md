# Modular frame concept

Status: **v1 modeled** (`build.py`, 2026-09-03). Below is the concept; actual numbers live in `params.py` and `README.md`.

Change from the original concept: the stack sits **on top of plate B**, not inside the sandwich. The 10 mm gap between A and B (arm thickness) is a cavity for the XT60 board and wiring.

## Architecture: 5 levels, bottom to top
```
 [4] GPS/compass mast          (+30…50 above the deck)
 [3] Upper deck: Pi 5 / ESP32 / camera   (standoffs 25–30)
 [2] Base sandwich: lower plate + arms + upper plate; FC/ESC stack inside
 [1] Battery tray              (under the base, standoffs/legs)
 [0] Legs                      (ground clearance ≥ 30 mm from ground to battery)
```

## Parts and their role
| # | Part | Qty | Print | Replacement |
|---|---|---|---|---|
| A | Lower base plate | 1 | flat, 5–6 mm | rarely; deadcat variant = different plate |
| B | Upper base plate | 1 | flat, 4–5 mm | rarely |
| C | Arm | 4 identical | flat, 10–12 mm | consumable, 4 bolts |
| D | Base standoffs (between A and B) | 4–6 | off-the-shelf aluminum M3 × 25–30 | none |
| E | Upper deck "Pi 5" | 1 | flat, 3–4 mm | swappable: "ESP32" deck, "blank with grid" |
| F | Camera module (tilted plate + holder) | 1 | 2 parts | swappable: different holder for a different camera |
| G | GPS mast | 1 | standing, hollow | swappable (height) |
| H | Battery tray | 1 | flat, 4 mm + side ribs | swappable for a different battery |
| I | Leg | 4 | flat (L-shaped) | consumable |

## Common hole grid (key decision)
All plates (A, B, E, H) carry the same grid: **M3, 20 mm pitch**, plus a **30.5 × 30.5** pattern in the center (stack) and **58 × 49** on deck E (Pi 5).
- Any deck fits any 4 standoffs of the grid.
- Standoffs are standard aluminum M3, lengths 20/25/30/35.
- Heat-set inserts M3 in plate A from below (legs and tray) and in the arms (motor).

## Geometry (target)
- Wheelbase (motor-to-motor diagonal): **240 mm**, so motor 120 mm from center on the diagonal; prop ø129.5, so tip-to-tip gap between neighboring props ≈ 40 mm. Margin for 5.5" and a heavy build.
- Arm: length from base edge ≈ 75–85 mm, motor pad ø32–34 with 4 holes 16×16 for M3 + center hole ø11 for the shaft.
- Base: plate ≈ 140 × 110 mm (fits the bed). Arm mounting: 2 M3 bolts through A–C–B + key/slot against rotation.
- Stack inside the sandwich: A to B height = 30 mm (30 mm standoffs), fits ESC + FC + capacitor.
- Battery under A: tray H on 10–15 mm standoffs below A (room for XT60/XT90 wires), battery 140 × 50 × 55, two straps through the tray slots.
- Legs: bolt to A via heat-set inserts in the corners near the arms; height ≈ 90–100 mm; track width ≥ 90 mm; angled slightly outward.
- Deck E above B on 25–30 mm standoffs (wires, stack USB-C); Pi 5 on 6 mm standoffs above E, cooler on top, so top ≈ +30. Prop tops at +28 above the arm ≈ level of B+…, so deck E must either fit entirely inside the circle between the props (≤ 100 × 90 mm) or sit above the prop disc.
- Camera: module F on the front edge of E or B, tilted 15–30° down, lens ≥ 15 mm ahead of the plate outline. With 155° FoV the front props will be in frame with any reasonable geometry: accept either a mask in software or a deadcat plate A (v2 option).
- GPS: mast G on deck E at the rear, +40 mm. Cable to UART6.

## Mass (target)
PETG frame: A ≈ 60, B ≈ 45, C 4 × 30, E ≈ 35, F ≈ 25, H ≈ 40, I 4 × 12, standoffs/hardware ≈ 60, total **≈ 430–480 g**. Heavier than carbon (250–300), AUW ≈ 1.5 kg, thrust/weight ≈ 4:1, acceptable.

## Modeling tool
**CadQuery (Python)**: parametric code in `hardware/frame/`, STEP + STL export for the slicer. All component dimensions in a single `params.py`, so numbers can be corrected after measuring and everything regenerated.

## Decided (ADR-0002, 2026-09-03)
1. Battery underneath.
2. Symmetric X for v1.
3. No RC receiver, no space reserved for it.

Motor pad on the arm: **5 mm** (pocket from below) so the stock 11 mm screws (≈9 thread) get ≥ 4 mm engagement; rest of the arm 10–12 mm. Center hole of the pad ø11 for the shaft.
