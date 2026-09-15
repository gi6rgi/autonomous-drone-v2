"""Детекция ArUco в fisheye-кадре и оценка положения маркера в NED.

Пайплайн: detectMarkers → углы → cv2.fisheye.undistortPoints (нормализованные координаты) →
solvePnP (K = I) → t_cam → тело (с учётом наклона камеры) → NED (по крену/тангажу/рысканию).
"""
from dataclasses import dataclass

import cv2
import numpy as np

from .camera_model import CAM_OFFSET_BODY, CAM_TILT_DEG, FisheyeCamera, R_body_from_cam, R_ned_from_body

ARUCO_DICT = cv2.aruco.DICT_4X4_50


@dataclass
class Detection:
    id: int
    corners_px: np.ndarray            # (4, 2)
    center_px: np.ndarray             # (2,)
    t_cam: np.ndarray                 # положение центра маркера в системе камеры, м
    marker_ned: np.ndarray            # положение центра маркера в NED, м
    angle_x: float                    # угловое смещение от оптической оси (для LANDING_TARGET), рад
    angle_y: float
    distance: float


class ArucoTracker:
    def __init__(self, cam: FisheyeCamera, marker_size_m: float, tilt_deg: float = CAM_TILT_DEG, marker_id: int | None = None):
        self.cam = cam
        self.size = marker_size_m
        self.marker_id = marker_id
        self._R_bc = R_body_from_cam(tilt_deg)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), params)
        h = marker_size_m / 2
        self._obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)   # TL, TR, BR, BL

    def detect(self, frame_bgr: np.ndarray, pos_ned, roll: float, pitch: float, yaw: float) -> Detection | None:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._det.detectMarkers(gray)
        if ids is None:
            return None
        cands = [(int(i), c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners)]
        if self.marker_id is not None:
            cands = [c for c in cands if c[0] == self.marker_id]
        if not cands:
            return None
        mid, pts = max(cands, key=lambda c: cv2.contourArea(c[1].astype(np.float32)))   # самый крупный
        und = cv2.fisheye.undistortPoints(pts.reshape(-1, 1, 2).astype(np.float64), self.cam.K, self.cam.D).reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(self._obj, und, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        t_cam = tvec.ravel()
        R_nb = R_ned_from_body(roll, pitch, yaw)
        marker_ned = np.asarray(pos_ned) + R_nb @ (CAM_OFFSET_BODY + self._R_bc @ t_cam)
        center = pts.mean(axis=0)
        # угловые смещения от оптической оси (эквидистантная модель): tan-углы по нормализованным координатам
        cu = und.mean(axis=0)
        return Detection(mid, pts, center, t_cam, marker_ned, float(np.arctan(cu[0])), float(np.arctan(cu[1])),
                         float(np.linalg.norm(t_cam)))


class FollowController:
    """П-регулятор по горизонтальной ошибке в NED + удержание высоты над маркером. Выход — скорость NED."""

    def __init__(self, kp: float = 0.8, v_max: float = 2.0, kp_z: float = 0.7, vz_max: float = 0.8):
        self.kp, self.v_max, self.kp_z, self.vz_max = kp, v_max, kp_z, vz_max

    def __call__(self, marker_ned: np.ndarray, pos_ned, target_alt: float) -> tuple[float, float, float]:
        err_n = marker_ned[0] - pos_ned[0]
        err_e = marker_ned[1] - pos_ned[1]
        vn, ve = self.kp * err_n, self.kp * err_e
        sp = np.hypot(vn, ve)
        if sp > self.v_max:
            vn, ve = vn / sp * self.v_max, ve / sp * self.v_max
        alt_err = target_alt - (-pos_ned[2])                      # + → надо вверх
        vd = float(np.clip(-self.kp_z * alt_err, -self.vz_max, self.vz_max))
        return float(vn), float(ve), vd
