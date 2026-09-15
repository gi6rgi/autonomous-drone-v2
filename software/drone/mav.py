"""Тонкая обёртка над pymavlink: состояние дрона, режимы, взлёт, скоростные команды в GUIDED.

Одинаково работает с SITL (tcp:127.0.0.1:5760 / udp) и с реальным F405 по UART (/dev/serial0, 921600).
"""
import math
import time
from dataclasses import dataclass, field

from pymavlink import mavutil


@dataclass
class State:
    t: float = 0.0
    armed: bool = False
    mode: str = ""
    pos_ned: list = field(default_factory=lambda: [0.0, 0.0, 0.0])   # north, east, down (м) относительно home
    vel_ned: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rel_alt: float = 0.0
    gps_fix: int = 0
    sats: int = 0

    @property
    def alt(self) -> float:
        return -self.pos_ned[2]


class Drone:
    def __init__(self, url: str, baud: int = 921600, source_system: int = 250):
        self.m = mavutil.mavlink_connection(url, baud=baud, source_system=source_system, autoreconnect=True)
        self.state = State()
        self.m.wait_heartbeat(timeout=60)
        self.target = (self.m.target_system, self.m.target_component or 1)
        self._request_streams()

    def _request_streams(self):
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 30),
                           (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30),
                           (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
                           (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
                           (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)):
            self.m.mav.command_long_send(*self.target, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def pump(self, max_msgs: int = 200):
        """Прочитать накопившиеся сообщения и обновить состояние."""
        for _ in range(max_msgs):
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            s = self.state
            if t == "HEARTBEAT":
                s.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                s.mode = mavutil.mode_string_v10(msg)
            elif t == "ATTITUDE":
                s.roll, s.pitch, s.yaw = msg.roll, msg.pitch, msg.yaw
            elif t == "LOCAL_POSITION_NED":
                s.pos_ned = [msg.x, msg.y, msg.z]
                s.vel_ned = [msg.vx, msg.vy, msg.vz]
                s.t = time.time()
            elif t == "GLOBAL_POSITION_INT":
                s.rel_alt = msg.relative_alt / 1000.0
            elif t == "GPS_RAW_INT":
                s.gps_fix, s.sats = msg.fix_type, msg.satellites_visible

    # ------------------------------------------------------------ команды
    def set_mode(self, name: str):
        mode_id = self.m.mode_mapping()[name]
        self.m.mav.set_mode_send(self.target[0], mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    def arm(self, force: bool = False):
        self.m.mav.command_long_send(*self.target, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     1, 21196 if force else 0, 0, 0, 0, 0, 0)

    def takeoff(self, alt_m: float):
        self.m.mav.command_long_send(*self.target, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)

    def set_velocity_ned(self, vn: float, ve: float, vd: float, yaw_rate: float | None = None):
        """Скорость в NED (м/с) в GUIDED. Слать не реже 2 Гц, иначе автопилот зависнет на месте."""
        mask = 0b0000_1111_1100_0111 if yaw_rate is None else 0b0000_0111_1100_0111   # только скорость (+ yaw_rate)
        self.m.mav.set_position_target_local_ned_send(
            0, *self.target, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            0, 0, 0, vn, ve, vd, 0, 0, 0, 0, yaw_rate or 0)

    def send_landing_target(self, angle_x: float, angle_y: float, distance: float, size_x: float = 0, size_y: float = 0):
        """Штатная precision-landing цель ArduPilot (PLND_TYPE=1 companion). Углы в рад от оптической оси."""
        self.m.mav.landing_target_send(int(time.time() * 1e6), 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                       angle_x, angle_y, distance, size_x, size_y)

    # ------------------------------------------------------------ ожидания
    def wait_ready(self, timeout: float = 120) -> bool:
        """Ждать 3D-fix и позицию (EKF готов)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump()
            if self.state.gps_fix >= 3 and self.state.t > 0:
                return True
            time.sleep(0.1)
        return False

    def wait_mode(self, name: str, timeout: float = 10) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump()
            if self.state.mode == name:
                return True
            time.sleep(0.05)
        return False

    def wait_armed(self, timeout: float = 20) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump()
            if self.state.armed:
                return True
            time.sleep(0.05)
        return False

    def wait_alt(self, alt_m: float, tol: float = 0.3, timeout: float = 60) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump()
            if abs(self.state.alt - alt_m) < tol:
                return True
            time.sleep(0.1)
        return False


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
