"""Frame v1 parameters. All dimensions in mm.

Source of each number: docs/components/*.md. Change here, then
regenerate the model: `python build.py`.
"""

from dataclasses import dataclass


# ---------------------------------------------------------------- print
@dataclass(frozen=True)
class Print:
    bed_xy: float = 220.0          # Anycubic Kobra 2 Neo
    bed_z: float = 250.0
    nozzle: float = 0.4
    hole_comp: float = 0.25        # hole diameter allowance for PETG
    chamfer: float = 0.6           # bottom edge chamfer (elephant foot)
    fillet_inner: float = 2.5      # inner outline corners


# ---------------------------------------------------------------- hardware
@dataclass(frozen=True)
class Hardware:
    m3_through: float = 3.4        # M3 clearance hole (compensation included)
    m3_insert: float = 4.0         # M3 heat-set insert (OD 4.6, length 5.7)
    m3_insert_depth: float = 6.0
    m3_head_d: float = 6.0         # button-head
    m3_head_h: float = 2.0
    m3_nut_af: float = 5.6         # M3 nut across flats (pocket +0.2)
    m3_nut_h: float = 2.6
    m4_through: float = 4.4        # M4 clearance hole
    m4_nut_af: float = 7.3         # M4 nut DIN 934: 7.0 across flats + clearance
    m4_nut_h: float = 3.8          # nut 3.2 + 0.6: pocket roof is a bridge and sags, measured 2026-09-09
    m4_head_d: float = 8.0         # M4 head (button 7.6 / cylinder 7.0) + clearance
    m4_cbore: float = 2.5          # counterbore for M4×8 head under the plate
    m25_through: float = 2.9       # Pi 5
    m2_through: float = 2.4        # camera
    standoff_od: float = 5.0       # aluminium M3 standoffs (round/hex 5 mm)


# ---------------------------------------------------------------- motor
@dataclass(frozen=True)
class Motor:
    """RETEK LN2207, per manufacturer drawing."""
    hole_pitch: float = 11.31      # NOTE: side of hole square; drawing's 16 mm is between OPPOSITE holes (ø16 circle)
    hole_circle_d: float = 16.0    # diameter of circle through the 4 × M3
    bell_d: float = 27.5
    body_h: float = 19.2           # without shaft
    shaft_d: float = 4.95
    base_hole_d: float = 10.0      # hole in motor base
    wire_len: float = 150.0
    stock_screw_len: float = 11.0  # incl. head; thread ≈ 9
    pad_thickness: float = 5.0     # arm pad under motor → engagement ≥ 4 mm
    pad_d: float = 33.0            # support pad under motor
    pad_center_hole: float = 11.0  # for shaft/circlip


# ---------------------------------------------------------------- prop
@dataclass(frozen=True)
class Prop:
    """HQProp R37 5.1×3.7×3."""
    diameter: float = 129.5
    hub_h: float = 7.0
    tip_clearance: float = 15.0    # min gap between adjacent prop tips


# ---------------------------------------------------------------- stack
@dataclass(frozen=True)
class Stack:
    """SpeedyBee F405 V3 + BLS 50A."""
    pitch: float = 30.5
    hole_d: float = 4.0            # in stack boards; in base: for M3 standoffs
    fc_lwh: tuple = (41.6, 39.4, 7.8)
    esc_lwh: tuple = (45.6, 44.0, 6.1)
    height_assembled: float = 28.0 # ESC + standoffs + FC + dampers (estimate)
    cap_d: float = 13.0            # 1000 µF capacitor
    cap_len: float = 25.0


# ---------------------------------------------------------------- Pi 5
@dataclass(frozen=True)
class Pi5:
    board: tuple = (85.0, 56.0)
    hole_pitch: tuple = (58.0, 49.0)
    hole_offset: float = 3.5       # board edge to hole center
    hole_d: float = 2.7
    standoff_h: float = 6.0        # min under board (3 mm protrusions)
    top_clearance: float = 30.0    # ports/cooler above board


# ---------------------------------------------------------------- camera
@dataclass(frozen=True)
class Camera:
    """Arducam IMX219 175° ("OMP camera v2.3"). Measured pattern: 22 × 11."""
    board: tuple = (25.0, 24.0)    # width × height
    hole_pitch: tuple = (22.0, 11.0)  # NOTE: re-check (standard v2 = 21 × 12.5)
    hole_d: float = 2.2
    lens_h: float = 14.0           # lens protrusion above board
    lens_d: float = 14.0           # TODO: holder diameter at base, estimate
    tilt_deg: float = 20.0         # downward tilt


# ---------------------------------------------------------------- GPS
@dataclass(frozen=True)
class Gps:
    """Round M8N puck (from photo), dimensions TODO."""
    puck_d: float = 50.0
    puck_h: float = 13.0
    mast_h: float = 40.0
    mast_pad: float = 30.0         # pad for module base


# ---------------------------------------------------------------- battery
@dataclass(frozen=True)
class Battery:
    lwh: tuple = (140.0, 43.0, 38.0)   # measured
    envelope: tuple = (142.0, 45.0, 40.0)
    strap_w: float = 20.0
    mass_g: float = 650.0          # TODO: estimate
    pdb_lwh: tuple = (35.0, 30.0, 10.0)  # power board XT60 IN/OUT (from photo)


# ---------------------------------------------------------------- frame
@dataclass(frozen=True)
class Frame:
    wheelbase: float = 240.0       # motor-to-motor diagonal
    grid_pitch: float = 20.0       # common M3 grid on all plates
    base_plate_t: float = 5.0      # base bottom plate
    top_plate_t: float = 4.0       # base top plate
    deck_t: float = 3.5            # decks
    arm_t: float = 10.0            # arm thickness
    arm_w: float = 30.0            # arm width at motor
    arm_root_w: float = 34.0       # arm width at root
    base_gap: float = 30.0         # standoffs between base plates (stack inside)
    deck_gap: float = 28.0         # standoffs from top plate to Pi deck
    tray_gap: float = 14.0         # standoffs from bottom plate to battery tray
    leg_h: float = 95.0            # leg height from base bottom to ground
    ground_clearance: float = 30.0 # ground to battery bottom


PRINT = Print()
HW = Hardware()
MOTOR = Motor()
PROP = Prop()
STACK = Stack()
PI5 = Pi5()
CAM = Camera()
GPS = Gps()
BAT = Battery()
FRAME = Frame()


def check():
    """Quick consistency checks."""
    import math
    adj = FRAME.wheelbase / math.sqrt(2)          # distance between adjacent motors
    gap = adj - PROP.diameter
    assert gap >= PROP.tip_clearance, f"prop tip gap {gap:.1f} < {PROP.tip_clearance}"
    engagement = (MOTOR.stock_screw_len - HW.m3_head_h) - MOTOR.pad_thickness
    assert engagement >= 4.0, f"motor screw engagement {engagement:.1f} mm < 4"
    arm_reach = FRAME.wheelbase / 2                # center to motor axis
    assert arm_reach + MOTOR.bell_d / 2 < PRINT.bed_xy, "arm does not fit on print bed"
    return dict(adjacent_motor_distance=round(adj, 1), prop_tip_gap=round(gap, 1),
                motor_screw_engagement=engagement, arm_reach=arm_reach)


if __name__ == "__main__":
    for k, v in check().items():
        print(f"{k:28s} {v}")
