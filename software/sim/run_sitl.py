"""Запуск ArduPilot SITL (ArduCopter) с параметрами нашего дрона.

Использование:  .venv/bin/python sim/run_sitl.py [--speedup 1] [--home lat,lon,alt,yaw]
SITL слушает TCP 5760 (первый GCS-порт). Наш код подключается к tcp:127.0.0.1:5760.
Состояние (eeprom, логи) — в sim/run/.
"""
import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path

ARDUPILOT = Path(os.environ.get("ARDUPILOT_DIR", Path.home() / "Code" / "ardupilot"))
BIN = ARDUPILOT / "build" / "sitl" / "bin" / "arducopter"
DEFAULTS = ARDUPILOT / "Tools" / "autotest" / "default_params" / "copter.parm"
OURS = Path(__file__).parent / "drone_v2.parm"
RUN_DIR = Path(__file__).parent / "run"


def sitl_command(speedup: float = 1.0, home: str = "55.7539,37.6208,150,0", instance: int = 0, wipe: bool = False):
    cmd = [str(BIN), "-S", "--model", "+", "--speedup", str(speedup), "--defaults", f"{DEFAULTS},{OURS}",
           f"-I{instance}", "--home", home, "--sim-address", "127.0.0.1"]
    if wipe:
        cmd.append("-w")
    return cmd


def start(speedup: float = 1.0, home: str = "55.7539,37.6208,150,0", wipe: bool = False, quiet: bool = True) -> subprocess.Popen:
    if not BIN.exists():
        raise SystemExit(f"нет бинарника SITL: {BIN}. Собрать: cd {ARDUPILOT} && ./waf configure --board sitl && ./waf copter")
    RUN_DIR.mkdir(exist_ok=True)
    out = open(RUN_DIR / "sitl.log", "w") if quiet else None
    return subprocess.Popen(sitl_command(speedup, home, wipe=wipe), cwd=RUN_DIR, stdout=out, stderr=subprocess.STDOUT)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--home", default="55.7539,37.6208,150,0")
    ap.add_argument("--wipe", action="store_true", help="сбросить eeprom (параметры) SITL")
    a = ap.parse_args()
    p = start(a.speedup, a.home, a.wipe, quiet=False)
    print("SITL запущен, pid", p.pid, "— MAVLink на tcp:127.0.0.1:5760. Ctrl+C для остановки.")
    try:
        p.wait()
    except KeyboardInterrupt:
        p.send_signal(signal.SIGINT)
        p.wait()
    sys.exit(p.returncode)
