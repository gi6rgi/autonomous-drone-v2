"""Модель камеры Arducam IMX219 175° (fisheye, эквидистантная проекция r = f·θ).

Одна и та же модель используется рендером в симуляторе и трекером на дроне.
Оси камеры: x вправо, y вниз, z вперёд (оптическая ось). Пиксели: u вправо, v вниз.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FisheyeCamera:
    width: int = 640
    height: int = 480
    hfov_deg: float = 155.0            # Arducam B0392: 155°(H) × 115°(V) × 175°(D)

    @property
    def f(self) -> float:
        """Фокус в пикселях для эквидистантной модели: r = f·θ, где θ — угол от оптической оси."""
        return (self.width / 2) / np.radians(self.hfov_deg / 2)

    @property
    def cx(self) -> float:
        return self.width / 2 - 0.5

    @property
    def cy(self) -> float:
        return self.height / 2 - 0.5

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.f, 0, self.cx], [0, self.f, self.cy], [0, 0, 1.0]])

    @property
    def D(self) -> np.ndarray:
        """Коэффициенты cv2.fisheye (k1..k4). Для идеальной эквидистантной модели — нули."""
        return np.zeros((4, 1))

    def pixel_rays(self) -> np.ndarray:
        """Единичные направления лучей для каждого пикселя в системе камеры, форма (H, W, 3)."""
        v, u = np.mgrid[0:self.height, 0:self.width].astype(np.float64)
        du, dv = u - self.cx, v - self.cy
        r = np.hypot(du, dv)
        theta = r / self.f
        phi = np.arctan2(dv, du)
        s = np.sin(theta)
        rays = np.stack([s * np.cos(phi), s * np.sin(phi), np.cos(theta)], axis=-1)
        rays[theta > np.radians(89.0)] = np.nan          # за пределами полусферы — нет изображения
        return rays

    def project(self, p_cam: np.ndarray) -> np.ndarray:
        """Точки (N,3) в системе камеры → пиксели (N,2). Точки позади камеры дают NaN."""
        p = np.asarray(p_cam, dtype=np.float64).reshape(-1, 3)
        norm = np.linalg.norm(p, axis=1)
        theta = np.arccos(np.clip(p[:, 2] / norm, -1, 1))
        phi = np.arctan2(p[:, 1], p[:, 0])
        r = self.f * theta
        uv = np.stack([self.cx + r * np.cos(phi), self.cy + r * np.sin(phi)], axis=1)
        uv[theta > np.radians(89.0)] = np.nan
        return uv


# Расположение камеры на дроне (система тела FRD: x вперёд, y вправо, z вниз), метры
CAM_OFFSET_BODY = np.array([0.046, 0.0, -0.033])   # кронштейн спереди, объектив на 33 мм выше плиты
CAM_TILT_DEG = 20.0                                 # наклон оптической оси вниз от горизонта


def R_body_from_cam(tilt_deg: float = CAM_TILT_DEG) -> np.ndarray:
    """Матрица, переводящая вектор из системы камеры (x вправо, y вниз, z вперёд) в систему тела FRD."""
    # без наклона: cam z → body x, cam x → body y, cam y → body z
    base = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=np.float64)
    t = np.radians(tilt_deg)
    tilt = np.array([[np.cos(t), 0, -np.sin(t)], [0, 1, 0], [np.sin(t), 0, np.cos(t)]])   # поворот вокруг body y: ось x уходит к +z (вниз в FRD)
    return tilt @ base


def R_ned_from_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Матрица поворота из системы тела FRD в NED по углам Эйлера (рад, порядок ZYX как в ArduPilot)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx
