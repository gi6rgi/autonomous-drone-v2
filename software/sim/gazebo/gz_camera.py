"""Frames from the Gazebo camera (topic drone_v2/camera) as numpy arrays, via gz-transport Python bindings.

  cam = GzCamera(); frame = cam.read(timeout=5)     # BGR uint8 (H, W, 3), same layout as sim/marker_renderer.py
  .venv/bin/python sim/gazebo/gz_camera.py           # save one frame to sim/out/gazebo_camera.png (sim must be running)

Bindings come from Homebrew (gz-transport13, gz-msgs10) and are built for Python 3.12; the venv needs protobuf.
"""
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

BREW_SITE = Path("/opt/homebrew/lib/python3.12/site-packages")
BINDINGS = Path(__file__).resolve().parent / ".gzpy"


def _ensure_bindings():
    """Copy of Homebrew's gz Python packages with an rpath to /opt/homebrew/lib.
    The originals find their dylibs via @rpath relative to the Homebrew Python, which fails from a uv venv."""
    if not (BINDINGS / "gz" / "transport13").exists():
        shutil.rmtree(BINDINGS, ignore_errors=True)
        shutil.copytree(BREW_SITE / "gz", BINDINGS / "gz", symlinks=False)
        for so in BINDINGS.rglob("*.so"):
            so.chmod(0o755)
            subprocess.run(["install_name_tool", "-add_rpath", "/opt/homebrew/lib", str(so)], check=True, capture_output=True)
            subprocess.run(["codesign", "-f", "-s", "-", str(so)], check=True, capture_output=True)
    if str(BINDINGS) not in sys.path:
        sys.path.insert(0, str(BINDINGS))


_ensure_bindings()
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.transport13 import Node  # noqa: E402

TOPIC = "drone_v2/camera"


class GzCamera:
    def __init__(self, topic: str = TOPIC, partition: str | None = "drone_v2"):
        if partition is not None:
            os.environ["GZ_PARTITION"] = partition                   # must be set before the Node is created
        self._node = Node()
        self._lock = threading.Lock()
        self._frame, self._stamp = None, 0.0
        if not self._node.subscribe(Image, topic, self._on_image):
            raise RuntimeError(f"cannot subscribe to {topic}")

    def _on_image(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._lock:
            self._frame = np.ascontiguousarray(img[:, :, ::-1])      # RGB -> BGR for OpenCV
            self._stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    def read(self, timeout: float = 1.0):
        """Latest frame and its sim time stamp, or (None, 0) if nothing arrived within timeout."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self._frame is not None:
                    return self._frame, self._stamp
            time.sleep(0.01)
        return None, 0.0


if __name__ == "__main__":
    import argparse

    import cv2

    ap = argparse.ArgumentParser()
    ap.add_argument("--partition", default="drone_v2")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "out" / "gazebo_camera.png"))
    a = ap.parse_args()
    frame, stamp = GzCamera(partition=a.partition).read(timeout=20)
    if frame is None:
        raise SystemExit(f"no frames on {TOPIC} (partition {a.partition})")
    cv2.imwrite(a.out, frame)
    print(f"frame {frame.shape[1]}x{frame.shape[0]} at sim t={stamp:.2f}s -> {a.out}")
