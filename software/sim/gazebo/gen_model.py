"""Generate models/drone_v2/model.sdf from mass_props.json (CAD) and the rotor/thrust constants below.

  python sim/gazebo/gen_model.py [--port 9002]

mass_props.json is written by hardware/frame/export_gazebo.py (needs CadQuery); this script only needs the stdlib,
so thrust tuning does not require the CAD environment.

Rotor model: ArduPilotPlugin drives each rotor joint in VELOCITY mode, gz LiftDrag on 2 virtual blades makes thrust.
Rotor speed is virtual: at 3500 rad/s a 1 ms physics step would turn the rotor 200° per step, so the max speed is
scaled down to ROTOR_MAX_RAD_S and the blade area scaled up to give the same thrust. Thrust ~ speed², max thrust
per motor = MAX_THRUST_N (RETEK LN2207 1960KV + 5.1" on 6S, ≈ 1.5–1.7 kg, docs/components).
"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).parent
MODEL = HERE / "models" / "drone_v2"

MAX_THRUST_N = 1.6 * 9.81          # per motor at 100 % (estimate, dimensions-summary.md)
ROTOR_MAX_RAD_S = 1000.0           # virtual, see docstring
CP_FRAC = 0.7                      # blade center of pressure at 0.7 R
A0, CLA, CDA = 0.3, 4.25, 0.10     # LiftDrag: angle of attack at rest, lift and drag slopes (same as Iris)
RHO = 1.2041
ROTOR_IZZ = 5e-5                   # virtual rotor inertia (motor bell + prop), keeps the velocity loop stable at 1 ms
ROTOR_P_GAIN = 0.02                # p_gain * dt / Izz = 0.4
ROTOR_CMD_MAX = 1.0                # N·m: 0 -> max speed in ≈ 50 ms
# ArduPilot motor order for Quad X: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW
MOTOR_ORDER = [(+1, -1, "ccw"), (-1, +1, "ccw"), (+1, +1, "cw"), (-1, -1, "cw")]


def blade_area(r_cp):
    """Blade area so that 2 blades give MAX_THRUST_N at ROTOR_MAX_RAD_S (LiftDrag: L = ½ρv²·A·CLA·α)."""
    v = ROTOR_MAX_RAD_S * r_cp
    return (MAX_THRUST_N / 2) / (0.5 * RHO * v * v * CLA * A0)


def material(r, g, b, a=1.0):
    return (f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>"
            f"<specular>0.2 0.2 0.2 1</specular></material>")


def mesh_visual(name, fname, color, pose="0 0 0 0 0 0"):
    return f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><mesh><uri>model://drone_v2/meshes/{fname}</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>
        {material(*color)}
      </visual>"""


def box_collision(name, center, size):
    c, s = center, size
    return f"""
      <collision name="{name}">
        <pose>{c[0]} {c[1]} {c[2]} 0 0 0</pose>
        <geometry><box><size>{s[0]} {s[1]} {s[2]}</size></box></geometry>
      </collision>"""


def foot_collision(i, p):
    return f"""
      <collision name="foot_{i}">
        <pose>{p[0]} {p[1]} {p[2] + 0.004} 0 0 0</pose>
        <geometry><sphere><radius>0.004</radius></sphere></geometry>
        <surface>
          <contact><ode><min_depth>0.0005</min_depth><max_vel>100</max_vel></ode></contact>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>"""


def rotor(i, x, y, z, direction, mp):
    m = mp["prop_mass_kg"] + 0.006                                    # prop + rotating part of the motor bell
    return f"""
    <link name="rotor_{i}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <inertial>
        <mass>{m:.4f}</mass>
        <inertia><ixx>{ROTOR_IZZ / 2}</ixx><iyy>{ROTOR_IZZ / 2}</iyy><izz>{ROTOR_IZZ}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>{mesh_visual("visual", f"prop_{direction}.stl", (0.15, 0.15, 0.15) if direction == "ccw" else (0.1, 0.35, 0.7))}
    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><damping>0.0</damping></dynamics>
      </axis>
    </joint>"""


def lift_drag(i, r_cp, area, direction):
    """Two opposite blades per rotor. CCW (positive joint speed): blade at +X moves toward +Y."""
    out = []
    for sign in (+1, -1):
        fwd = sign if direction == "ccw" else -sign
        out.append(f"""
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>{A0}</a0><alpha_stall>1.4</alpha_stall>
      <cla>{CLA}</cla><cda>{CDA}</cda><cma>0.0</cma>
      <cla_stall>-0.025</cla_stall><cda_stall>0.0</cda_stall><cma_stall>0.0</cma_stall>
      <area>{area:.6f}</area><air_density>{RHO}</air_density>
      <cp>{sign * r_cp:.4f} 0 0</cp><forward>0 {fwd} 0</forward><upward>0 0 1</upward>
      <link_name>rotor_{i}</link_name>
    </plugin>""")
    return "".join(out)


def control(i, direction):
    mult = ROTOR_MAX_RAD_S if direction == "ccw" else -ROTOR_MAX_RAD_S
    return f"""
      <control channel="{i}">
        <jointName>rotor_{i}_joint</jointName>
        <useForce>1</useForce>
        <multiplier>{mult:.0f}</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>{ROTOR_P_GAIN}</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
        <i_max>0</i_max><i_min>0</i_min>
        <cmd_max>{ROTOR_CMD_MAX}</cmd_max><cmd_min>-{ROTOR_CMD_MAX}</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>"""


def camera_sensor(mp):
    """Arducam IMX219 175°: equidistant fisheye, 155° horizontal over 640 px (same as drone/camera_model.py)."""
    c = mp["camera"]
    x, y, z = c["pos_m"]
    return f"""
    <link name="camera_link">
      <pose degrees="true">{x} {y} {z} 0 {c["tilt_deg"]} 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <sensor name="camera" type="wideanglecamera">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <topic>drone_v2/camera</topic>
        <camera>
          <horizontal_fov>{math.radians(155.0):.4f}</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.02</near><far>200</far></clip>
          <lens>
            <type>equidistant</type>
            <scale_to_hfov>true</scale_to_hfov>
            <cutoff_angle>{math.radians(89.0):.4f}</cutoff_angle>
            <env_texture_size>1024</env_texture_size>
          </lens>
        </camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>"""


def build(mp, port, with_camera=True):
    r_cp = mp["prop_radius_m"] * CP_FRAC
    area = blade_area(r_cp)
    I = mp["inertia_kg_m2"]
    cg = mp["cg_m"]
    z = mp["prop_plane_z_m"]
    rotors = [(i, sx * abs(mp["motors_m"][0][0]), sy * abs(mp["motors_m"][0][1]), d) for i, (sx, sy, d) in enumerate(MOTOR_ORDER)]
    bat, plate = mp["battery_box_m"], mp["plate_box_m"]

    base = f"""
    <link name="base_link">
      <inertial>
        <pose>{cg[0]} {cg[1]} {cg[2]} 0 0 0</pose>
        <mass>{mp["mass_kg"]}</mass>
        <inertia>
          <ixx>{I[0][0]}</ixx><ixy>{I[0][1]}</ixy><ixz>{I[0][2]}</ixz>
          <iyy>{I[1][1]}</iyy><iyz>{I[1][2]}</iyz><izz>{I[2][2]}</izz>
        </inertia>
      </inertial>{mesh_visual("frame", "frame.stl", (0.55, 0.57, 0.6))}{mesh_visual("electronics", "electronics.stl", (0.1, 0.45, 0.15))}{mesh_visual("battery", "battery.stl", (0.1, 0.1, 0.4))}{mesh_visual("motors", "motors.stl", (0.1, 0.1, 0.1))}{box_collision("battery_collision", bat["center"], bat["size"])}{box_collision("plate_collision", plate["center"], plate["size"])}{"".join(foot_collision(i, p) for i, p in enumerate(mp["feet_m"]))}
    </link>

    <link name="imu_link">
      <pose>{cg[0]} {cg[1]} {cg[2]} 0 0 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <sensor name="imu_sensor" type="imu">
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000</update_rate>
      </sensor>
    </link>
    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
    </joint>"""

    return f"""<?xml version="1.0"?>
<!-- generated by software/sim/gazebo/gen_model.py from mass_props.json; do not edit by hand -->
<!-- mass {mp["mass_kg"]} kg, max thrust {MAX_THRUST_N:.1f} N per motor, virtual rotor {ROTOR_MAX_RAD_S:.0f} rad/s, blade area {area:.5f} m² -->
<sdf version="1.9">
  <model name="drone_v2">{base}{"".join(rotor(i, x, y, z, d, mp) for i, x, y, d in rotors)}{camera_sensor(mp) if with_camera else ""}

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>{"".join(lift_drag(i, r_cp, area, d) for i, _, _, d in rotors)}{"".join(f'''
    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_{i}_joint</joint_name>
    </plugin>''' for i, *_ in rotors)}

    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>{port}</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_link::imu_sensor</imuName>{"".join(control(i, d) for i, _, _, d in rotors)}
    </plugin>
  </model>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9002, help="JSON FDM port: 9002 + 10 * SITL instance")
    ap.add_argument("--no-camera", action="store_true", help="without the camera sensor (faster, no rendering)")
    a = ap.parse_args()
    mp = json.loads((MODEL / "mass_props.json").read_text())
    (MODEL / "model.sdf").write_text(build(mp, a.port, not a.no_camera))
    (MODEL / "model.config").write_text("""<?xml version="1.0"?>
<model>
  <name>drone_v2</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Autonomous drone v2: 5" quad, 6S, RETEK LN2207, ArduPilot SITL (JSON)</description>
</model>
""")
    r_cp = mp["prop_radius_m"] * CP_FRAC
    hover = mp["mass_kg"] * 9.81 / 4 / MAX_THRUST_N
    print(f"model.sdf: mass {mp['mass_kg']} kg, blade area {blade_area(r_cp):.5f} m², port {a.port}")
    print(f"expected hover: thrust {hover:.0%} of max, rotor speed {math.sqrt(hover):.0%} of max")


if __name__ == "__main__":
    main()
