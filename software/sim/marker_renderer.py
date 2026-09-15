"""Рендер кадра камеры дрона: плоская земля с сеткой и ArUco-маркером, небо над горизонтом.

Для каждого пикселя: луч в системе камеры → тело → NED → пересечение с землёй (NED down = 0)
→ цвет из процедурной текстуры. Векторизовано numpy, 640×480 ≈ 15–25 мс.
Запуск как скрипта: рендерит тестовый кадр в sim/out/selftest.png.
"""
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drone.camera_model import CAM_OFFSET_BODY, CAM_TILT_DEG, FisheyeCamera, R_body_from_cam, R_ned_from_body  # noqa: E402

ARUCO_DICT = cv2.aruco.DICT_4X4_50


@dataclass
class Marker:
    id: int = 0
    size_m: float = 0.5                     # сторона чёрного квадрата маркера
    north: float = 0.0                      # положение центра, NED
    east: float = 0.0
    yaw_deg: float = 0.0

    def bitmap(self) -> np.ndarray:
        """Битмап маркера с белым полем (quiet zone): (6+2)×(6+2) ячеек, 0 = чёрный, 255 = белый."""
        d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        img = cv2.aruco.generateImageMarker(d, self.id, 6)      # 6 = 4 бита + рамка, по пикселю на ячейку
        return cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=255)

    @property
    def cell_m(self) -> float:
        return self.size_m / 6.0

    def corners_ned(self) -> np.ndarray:
        """4 угла чёрного квадрата в NED (z=0), порядок как у cv2.aruco: TL, TR, BR, BL при взгляде сверху."""
        h = self.size_m / 2
        local = np.array([[-h, -h], [-h, h], [h, h], [h, -h]])     # (north, east) до поворота
        c, s = np.cos(np.radians(self.yaw_deg)), np.sin(np.radians(self.yaw_deg))
        R = np.array([[c, -s], [s, c]])
        ne = local @ R.T + np.array([self.north, self.east])
        return np.hstack([ne, np.zeros((4, 1))])


@dataclass
class Renderer:
    cam: FisheyeCamera = field(default_factory=FisheyeCamera)
    marker: Marker = field(default_factory=Marker)
    tilt_deg: float = CAM_TILT_DEG
    grid_m: float = 1.0

    def __post_init__(self):
        self._rays = self.cam.pixel_rays()                        # (H, W, 3) в системе камеры
        self._R_bc = R_body_from_cam(self.tilt_deg)
        self._bitmap = self.marker.bitmap()

    def set_marker(self, north: float, east: float, yaw_deg: float | None = None):
        self.marker.north, self.marker.east = north, east
        if yaw_deg is not None:
            self.marker.yaw_deg = yaw_deg

    def render(self, pos_ned: np.ndarray, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Кадр BGR uint8 (H, W, 3). pos_ned — положение дрона (north, east, down), углы в радианах."""
        R_nb = R_ned_from_body(roll, pitch, yaw)
        R_nc = R_nb @ self._R_bc
        cam_pos = np.asarray(pos_ned, dtype=np.float64) + R_nb @ CAM_OFFSET_BODY
        d = self._rays @ R_nc.T                                    # лучи в NED, (H, W, 3)
        dz = d[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -cam_pos[2] / dz                                   # до земли (down = 0)
        hit = np.isfinite(t) & (t > 0) & (dz > 1e-6)
        img = np.empty((self.cam.height, self.cam.width, 3), np.uint8)
        # небо: градиент от горизонта
        elev = np.nan_to_num(np.clip(-dz, 0, 1))
        img[..., 0] = (200 - 60 * elev).astype(np.uint8)
        img[..., 1] = (190 - 60 * elev).astype(np.uint8)
        img[..., 2] = (180 - 90 * elev).astype(np.uint8)
        # земля
        tt = np.where(hit, t, 0.0)
        gn = cam_pos[0] + tt * d[..., 0]
        ge = cam_pos[1] + tt * d[..., 1]
        ground = np.full((self.cam.height, self.cam.width, 3), (92, 128, 96), np.uint8)   # BGR травянисто-серый
        # сетка 1 м, линия ≈ 3 см, ослабевает с дистанцией
        fn, fe = np.abs((gn / self.grid_m + 0.5) % 1.0 - 0.5), np.abs((ge / self.grid_m + 0.5) % 1.0 - 0.5)
        line = (fn < 0.03 / self.grid_m) | (fe < 0.03 / self.grid_m)
        ground[line] = (150, 165, 150)
        # маркер: перевод в локальные координаты маркера
        m = self.marker
        c, s = np.cos(np.radians(m.yaw_deg)), np.sin(np.radians(m.yaw_deg))
        ln = np.nan_to_num((gn - m.north) * c + (ge - m.east) * s)
        le = np.nan_to_num(-(gn - m.north) * s + (ge - m.east) * c)
        half = m.cell_m * 4                                        # чёрный квадрат 6 ячеек + поле 1 ячейка
        inside = (np.abs(ln) < half) & (np.abs(le) < half)
        ci = 7 - np.clip(((ln + half) / m.cell_m).astype(int), 0, 7)   # строка битмапа: север = верх маркера (иначе зеркало)
        cj = np.clip(((le + half) / m.cell_m).astype(int), 0, 7)   # столбец = east
        val = self._bitmap[ci, cj]
        ground[inside] = np.stack([val, val, val], axis=-1)[inside]
        # дальность: лёгкая дымка
        dist = np.clip(tt / 60.0, 0, 1)[..., None]
        ground = (ground * (1 - 0.5 * dist) + np.array([200, 190, 180]) * 0.5 * dist).astype(np.uint8)
        img[hit] = ground[hit]
        img[~np.isfinite(dz)] = (20, 20, 20)                       # за пределами объектива
        return img


if __name__ == "__main__":
    r = Renderer()
    out = Path(__file__).parent / "out"; out.mkdir(exist_ok=True)
    # дрон на 4 м высоты, 3 м южнее маркера, смотрит на север, камера наклонена на 20° вниз
    frame = r.render(np.array([-3.0, 0.5, -4.0]), 0.0, 0.0, 0.0)
    cv2.imwrite(str(out / "selftest.png"), frame)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    print("selftest.png записан; найдено маркеров:", 0 if ids is None else len(ids), "ids:", None if ids is None else ids.ravel().tolist())
