"""Headless unitree_mujoco sim runner (no GLFW viewer).

Identical DDS bridge + physics as simulate_python/unitree_mujoco.py, but
without the MuJoCo viewer -- so it runs in containers / CI / headless hosts
(no X, no GPU). Gate scripts connect over DDS exactly as with the viewer sim.

Usage:
    .venv/bin/python scripts/sim_headless.py [--duration 60]
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path

# pygame is imported by the bridge; with USE_JOYSTICK=0 it is never init'd,
# but force a dummy video driver so the import is safe in headless contexts.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import mujoco  # noqa: E402

_SIMDIR = (Path(__file__).resolve().parents[1]
           / "external" / "unitree_mujoco" / "simulate_python")
sys.path.insert(0, str(_SIMDIR))
from unitree_sdk2py_bridge import UnitreeSdk2Bridge  # noqa: E402
import config  # noqa: E402
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()

    # config.ROBOT_SCENE is relative to the simulate_python dir
    os.chdir(_SIMDIR)

    mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    mj_data = mujoco.MjData(mj_model)
    mj_model.opt.timestep = config.SIMULATE_DT
    time.sleep(0.2)

    stop = threading.Event()

    def _sig(_s, _f):
        stop.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    locker = threading.Lock()

    def simulation_thread():
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        unitree = UnitreeSdk2Bridge(mj_model, mj_data)
        if config.PRINT_SCENE_INFORMATION:
            unitree.PrintSceneInformation()
        while not stop.is_set():
            step_start = time.perf_counter()
            locker.acquire()
            mujoco.mj_step(mj_model, mj_data)
            locker.release()
            wait = mj_model.opt.timestep - (time.perf_counter() - step_start)
            if wait > 0:
                time.sleep(wait)

    t = threading.Thread(target=simulation_thread, daemon=True)
    t.start()
    print(f"[sim_headless] running {args.duration}s (no viewer). "
          f"DDS domain={config.DOMAIN_ID} iface={config.INTERFACE}", flush=True)
    stop.wait(timeout=args.duration)
    stop.set()
    t.join(timeout=1.0)
    print("[sim_headless] stopped", flush=True)


if __name__ == "__main__":
    main()