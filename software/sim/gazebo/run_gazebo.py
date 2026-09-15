"""Gazebo Harmonic + ArduPilot SITL (JSON) with the drone v2 model.

  .venv/bin/python sim/gazebo/run_gazebo.py              # server + GUI + SITL, MAVLink on tcp:127.0.0.1:5760
  .venv/bin/python sim/gazebo/run_gazebo.py --headless   # no GUI
  .venv/bin/python sim/gazebo/run_gazebo.py --instance 1 --no-camera   # second copy: MAVLink 5770, FDM port 9012

On macOS the Gazebo server and GUI must be separate processes; both run in the gz partition "drone_v2"
so they do not mix with other Gazebo sessions on this machine. To attach a GUI later:
  GZ_PARTITION=drone_v2 GZ_SIM_RESOURCE_PATH=... gz sim -g
Needs: brew gz-harmonic, ~/Code/ardupilot_gazebo built (ArduPilotPlugin), ArduCopter SITL (see software/README.md).
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from sim import run_sitl  # noqa: E402
from sim.gazebo import gen_model  # noqa: E402

ARDUPILOT_GAZEBO = Path(os.environ.get("ARDUPILOT_GAZEBO_DIR", Path.home() / "Code" / "ardupilot_gazebo"))
WORLD = HERE / "worlds" / "drone_v2_field.sdf"
PARAMS = HERE / "drone_v2_gazebo.parm"
RUN_DIR = HERE / "run"
PARTITION = "drone_v2"


def gz_env(partition: str = PARTITION) -> dict:
    env = dict(os.environ)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(filter(None, [str(ARDUPILOT_GAZEBO / "build"), env.get("GZ_SIM_SYSTEM_PLUGIN_PATH")]))
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(filter(None, [str(HERE / "models"), str(HERE / "worlds"),
                                                                str(ARDUPILOT_GAZEBO / "models"), env.get("GZ_SIM_RESOURCE_PATH")]))
    env["GZ_PARTITION"] = partition
    return env


def sitl_command(instance: int, speedup: float, home: str, wipe: bool) -> list:
    cmd = [str(run_sitl.BIN), "-S", "--model", "JSON", "--speedup", str(speedup),
           "--defaults", f"{run_sitl.DEFAULTS},{run_sitl.OURS},{PARAMS}",
           f"-I{instance}", "--home", home, "--sim-address", "127.0.0.1"]
    return cmd + (["-w"] if wipe else [])


def start(instance: int = 0, headless: bool = False, camera: bool = True, speedup: float = 1.0,
          home: str = "55.7539,37.6208,150,0", wipe: bool = True, verbose: int = 2) -> list:
    """Start gz server (+ GUI) and SITL. Returns the processes, SITL last."""
    if not run_sitl.BIN.exists():
        raise SystemExit(f"no SITL binary: {run_sitl.BIN}")
    if not (ARDUPILOT_GAZEBO / "build" / "libArduPilotPlugin.dylib").exists():
        raise SystemExit(f"no ArduPilotPlugin in {ARDUPILOT_GAZEBO / 'build'}")
    port = 9002 + 10 * instance
    mp = json.loads((gen_model.MODEL / "mass_props.json").read_text())
    (gen_model.MODEL / "model.sdf").write_text(gen_model.build(mp, port, camera))

    run = RUN_DIR / f"i{instance}"
    run.mkdir(parents=True, exist_ok=True)
    env = gz_env(f"{PARTITION}_{instance}" if instance else PARTITION)
    procs = []
    log = open(run / "gz_server.log", "w")
    procs.append(subprocess.Popen(["gz", "sim", "-s", "-r", f"-v{verbose}", str(WORLD)], env=env, cwd=run,
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    if not headless:
        procs.append(subprocess.Popen(["gz", "sim", "-g", f"-v{verbose}"], env=env, cwd=run,
                                      stdout=open(run / "gz_gui.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    time.sleep(3.0)                                               # let the plugin bind its FDM port before SITL starts sending
    procs.append(subprocess.Popen(sitl_command(instance, speedup, home, wipe), cwd=run,
                                  stdout=open(run / "sitl.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    return procs


def stop(procs: list):
    for p in reversed(procs):
        if p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    t0 = time.time()
    for p in procs:
        try:
            p.wait(timeout=max(0.1, 5 - (time.time() - t0)))
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--headless", action="store_true", help="no Gazebo GUI")
    ap.add_argument("--no-camera", action="store_true", help="model without the camera sensor")
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--home", default="55.7539,37.6208,150,0")
    ap.add_argument("--keep-params", action="store_true", help="do not wipe SITL eeprom")
    a = ap.parse_args()
    procs = start(a.instance, a.headless, not a.no_camera, a.speedup, a.home, not a.keep_params)
    print(f"Gazebo + SITL running, MAVLink tcp:127.0.0.1:{5760 + 10 * a.instance}, logs in {RUN_DIR / f'i{a.instance}'}. Ctrl+C to stop.")
    try:
        while all(p.poll() is None for p in procs):
            time.sleep(0.5)
        print("a process exited:", [(p.args[0], p.returncode) for p in procs if p.poll() is not None])
    except KeyboardInterrupt:
        pass
    finally:
        stop(procs)


if __name__ == "__main__":
    main()
