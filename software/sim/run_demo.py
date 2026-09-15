"""Демо: SITL + рендер камеры + ArUco-трекер + GUIDED-управление + 3D-просмотрщик.

  .venv/bin/python sim/run_demo.py                 # запускает SITL сам, открой http://localhost:8766
  .venv/bin/python sim/run_demo.py --no-sitl       # SITL уже запущен отдельно (sim/run_sitl.py)
  .venv/bin/python sim/run_demo.py --headless --duration 90 --speedup 5   # без браузера, для тестов

Фазы: takeoff → search (маркер не виден: медленный разворот) → follow (висим позади маркера на высоте
--alt так, чтобы он был в кадре) → land (по клавише L или --auto-land: снижение с сопровождением, LAND у земли).
Клик по земле в просмотрщике переставляет маркер — дрон следует за ним.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drone.camera_model import CAM_TILT_DEG, FisheyeCamera, R_ned_from_body, R_body_from_cam, CAM_OFFSET_BODY  # noqa: E402
from drone.mav import Drone, wrap_pi  # noqa: E402
from drone.tracker import ArucoTracker, FollowController  # noqa: E402
from sim.marker_renderer import Marker, Renderer  # noqa: E402
from sim import run_sitl  # noqa: E402


def standoff_target(marker_ne, yaw, alt, tilt_deg, view_below_axis_deg=35.0):
    """Точка зависания: позади маркера (против направления взгляда) так, чтобы маркер был на view_below_axis ниже оси камеры."""
    theta = math.radians(tilt_deg + view_below_axis_deg)
    d = alt / math.tan(theta)
    return np.array([marker_ne[0] - d * math.cos(yaw), marker_ne[1] - d * math.sin(yaw)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-sitl", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--alt", type=float, default=4.0, help="высота слежения, м")
    ap.add_argument("--marker", default="6,1", help="начальное положение маркера north,east")
    ap.add_argument("--marker-size", type=float, default=0.5)
    ap.add_argument("--tilt", type=float, default=CAM_TILT_DEG)
    ap.add_argument("--auto-land", action="store_true", help="сесть автоматически после захвата маркера")
    ap.add_argument("--duration", type=float, default=0, help="остановиться через N секунд (0 = бесконечно)")
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    a = ap.parse_args()

    sitl = None
    if not a.no_sitl:
        sitl = run_sitl.start(a.speedup, wipe=True)
        print("SITL запущен, pid", sitl.pid)
        time.sleep(2.0)

    mn, me = (float(v) for v in a.marker.split(","))
    cam = FisheyeCamera()
    renderer = Renderer(cam=cam, marker=Marker(size_m=a.marker_size, north=mn, east=me), tilt_deg=a.tilt)
    tracker = ArucoTracker(cam, a.marker_size, tilt_deg=a.tilt)
    ctrl = FollowController()
    viewer = None
    if not a.headless:
        from sim.viewer import Viewer
        viewer = Viewer(); viewer.start()
        print("просмотрщик: http://localhost:8766")

    drone = Drone(a.url)
    print("heartbeat получен; жду GPS/EKF…")
    if not drone.wait_ready(180):
        raise SystemExit("нет GPS-fix / позиции")
    drone.set_mode("GUIDED"); drone.wait_mode("GUIDED")
    drone.arm(); assert drone.wait_armed(30), "не армится"
    drone.takeoff(a.alt); print("взлёт…")
    assert drone.wait_alt(a.alt, 0.5, 90), "не набрал высоту"

    phase, land_req = "follow", a.auto_land
    est_ned, last_seen, seen = None, 0.0, False
    frames, t_start, t_frame_prev, fps = 0, time.time(), time.time(), 0.0
    R_bc = R_body_from_cam(a.tilt)
    last_vis = 0.0
    while True:
        drone.pump()
        st = drone.state
        now = time.time()
        if a.duration and now - t_start > a.duration:
            break
        # команды из просмотрщика
        if viewer:
            for cmd in viewer.pop_commands():
                if "marker" in cmd:
                    renderer.set_marker(*cmd["marker"]); print("маркер →", np.round(cmd["marker"], 2))
                if cmd.get("land"):
                    land_req = True
                if cmd.get("follow"):
                    land_req, phase = False, "follow"
        # кадр с "камеры" и детекция
        frame = renderer.render(np.array(st.pos_ned), st.roll, st.pitch, st.yaw)
        det = tracker.detect(frame, st.pos_ned, st.roll, st.pitch, st.yaw)
        frames += 1
        fps = 0.9 * fps + 0.1 / max(now - t_frame_prev, 1e-3); t_frame_prev = now
        if det is not None:
            est_ned = 0.7 * est_ned + 0.3 * det.marker_ned if est_ned is not None else det.marker_ned
            last_seen, seen = now, True
        else:
            seen = now - last_seen < 1.0
        # управление
        alt = st.alt
        if phase == "follow" and land_req and est_ned is not None and seen:
            phase = "land"
        if est_ned is None or not seen:
            drone.set_velocity_ned(0, 0, 0, yaw_rate=0.4)          # поиск: медленный разворот на месте
            err = None
        else:
            target_alt = a.alt
            if phase == "land":
                target_alt = max(alt - 0.8, 0.3)                    # снижаемся, держа маркер в кадре
            tgt = standoff_target(est_ned[:2], st.yaw, max(alt, 0.5), a.tilt)
            vn, ve, vd = ctrl(np.array([tgt[0], tgt[1], 0]), st.pos_ned, target_alt)
            # рыскание: держать маркер по центру кадра
            bearing = math.atan2(est_ned[1] - st.pos_ned[1], est_ned[0] - st.pos_ned[0])
            yaw_rate = float(np.clip(1.5 * wrap_pi(bearing - st.yaw), -0.8, 0.8))
            err = float(np.hypot(est_ned[0] - st.pos_ned[0], est_ned[1] - st.pos_ned[1]))
            if phase == "land" and alt < 0.6:
                drone.set_mode("LAND"); phase = "landed"; print("LAND: маркер на", np.round(est_ned, 2), "дрон на", np.round(st.pos_ned, 2))
            elif phase != "landed":
                drone.set_velocity_ned(vn, ve, vd, yaw_rate=yaw_rate)
        # просмотрщик
        if viewer and now - last_vis > 0.05:
            last_vis = now
            R_nb = R_ned_from_body(st.roll, st.pitch, st.yaw); R_nc = R_nb @ R_bc
            cam_pos = np.array(st.pos_ned) + R_nb @ CAM_OFFSET_BODY
            corners = []
            for u, v in ((0, 0), (cam.width - 1, 0), (cam.width - 1, cam.height - 1), (0, cam.height - 1)):
                du, dv = (u - cam.cx) / cam.f, (v - cam.cy) / cam.f
                th, ph = min(math.hypot(du, dv), 1.2), math.atan2(dv, du)
                ray = R_nc @ np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th)])
                corners.append((cam_pos + ray * 3.0).tolist())
            payload = dict(pose=dict(pos=list(map(float, st.pos_ned)), roll=st.roll, pitch=st.pitch, yaw=st.yaw),
                           mode=st.mode, armed=st.armed, speed=float(np.hypot(*st.vel_ned[:2])), phase=phase, seen=seen,
                           err=err, est=None if est_ned is None else [float(v) for v in est_ned[:2]],
                           marker=[renderer.marker.north, renderer.marker.east], frustum=corners, fps=fps,
                           t=now - t_start, marker_bitmap=(renderer.marker.bitmap() > 0).astype(int).tolist(),
                           marker_size=a.marker_size)
            viewer.send(payload)
            if frames % 2 == 0:
                viewer.send_frame(frame)
        if phase == "landed" and not st.armed:
            print("посадка завершена, дрон дизармлен"); break
        time.sleep(0.02)
    if a.headless:
        d = float(np.hypot(st.pos_ned[0] - renderer.marker.north, st.pos_ned[1] - renderer.marker.east))
        print(f"итог: фаза {phase}, дрон {np.round(st.pos_ned, 2)}, маркер ({renderer.marker.north}, {renderer.marker.east}), "
              f"расстояние по горизонтали {d:.2f} м, оценка маркера {None if est_ned is None else np.round(est_ned, 2)}, {fps:.1f} fps")
    if sitl:
        sitl.terminate()


if __name__ == "__main__":
    main()
