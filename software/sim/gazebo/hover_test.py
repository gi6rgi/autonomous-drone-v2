"""Hover check for the Gazebo model: takeoff, hold, land; prints hover throttle and stability.

  .venv/bin/python sim/gazebo/hover_test.py                # starts Gazebo (headless) + SITL itself
  .venv/bin/python sim/gazebo/hover_test.py --no-start     # attach to a running run_gazebo.py

Targets for the real drone (docs/components/dimensions-summary.md): hover ≈ 22–25 % throttle, no oscillation.
"""
import argparse
import math
import statistics as st
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from drone.mav import Drone  # noqa: E402
from sim.gazebo import run_gazebo  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-start", action="store_true")
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--alt", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=20.0, help="hover time, s")
    ap.add_argument("--camera", action="store_true", help="include the camera sensor (slower)")
    a = ap.parse_args()

    procs = [] if a.no_start else run_gazebo.start(a.instance, headless=True, camera=a.camera)
    try:
        d = Drone(f"tcp:127.0.0.1:{5760 + 10 * a.instance}")
        m = d.m
        m.mav.command_long_send(*d.target, 511, 0, 36, int(1e6 / 20), 0, 0, 0, 0, 0)   # SERVO_OUTPUT_RAW at 20 Hz
        print("waiting for GPS/EKF…")
        if not d.wait_ready(180):
            raise SystemExit("no position")
        time.sleep(5)                                                                  # EKF settles after the first fix
        d.set_mode("GUIDED"); d.wait_mode("GUIDED")
        d.arm()
        if not d.wait_armed(30):
            raise SystemExit("did not arm")
        d.takeoff(a.alt)
        t0 = time.time()
        if not d.wait_alt(a.alt, 0.3, 60):
            raise SystemExit(f"did not reach {a.alt} m, alt {d.state.alt:.2f}")
        print(f"at {a.alt} m after {time.time() - t0:.1f} s, holding {a.hold:.0f} s")

        alt, roll, pitch, pwm, pos = [], [], [], [], []
        t0 = time.time()
        while time.time() - t0 < a.hold:
            msg = m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "ATTITUDE":
                roll.append(math.degrees(msg.roll)); pitch.append(math.degrees(msg.pitch))
            elif t == "LOCAL_POSITION_NED":
                alt.append(-msg.z); pos.append((msg.x, msg.y))
            elif t == "SERVO_OUTPUT_RAW":
                pwm.append([msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw])
        m.param_fetch_one("MOT_THST_HOVER")
        hov = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)

        mean_pwm = [st.mean(p[i] for p in pwm) for i in range(4)] if pwm else [0] * 4
        throttle = [(v - 1000) / 1000 for v in mean_pwm]
        drift = max(math.hypot(x - pos[0][0], y - pos[0][1]) for x, y in pos) if pos else float("nan")
        print(f"alt       mean {st.mean(alt):.2f} m, std {st.pstdev(alt):.3f} m")
        print(f"roll      std {st.pstdev(roll):.2f}°, max |{max(map(abs, roll)):.1f}|°")
        print(f"pitch     std {st.pstdev(pitch):.2f}°, max |{max(map(abs, pitch)):.1f}|°")
        print(f"drift     {drift:.2f} m")
        print(f"motors    PWM {[round(v) for v in mean_pwm]}, throttle {[f'{v:.0%}' for v in throttle]}")
        if hov is not None:
            print(f"MOT_THST_HOVER {hov.param_value:.3f}")

        d.set_mode("LAND")
        t0 = time.time()
        while time.time() - t0 < 60:
            d.pump()
            if not d.state.armed:
                print(f"landed and disarmed after {time.time() - t0:.1f} s")
                break
            time.sleep(0.2)
    finally:
        run_gazebo.stop(procs)


if __name__ == "__main__":
    main()
