"""Headless unitree_mujoco sim runner (no GLFW viewer) with rangefinder support.

Identical DDS bridge + physics as simulate_python/unitree_mujoco.py, but
without the viewer — runs in containers/CI/headless. If the scene has
MuJoCo <rangefinder> sensors, their distances are published on DDS topic
"rt/rangefinders" (RangefinderData: forward, left, right in metres).

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

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

_SIMDIR = (Path(__file__).resolve().parents[1]
           / "external" / "unitree_mujoco" / "simulate_python")
sys.path.insert(0, str(_SIMDIR))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root for codeep
from unitree_sdk2py_bridge import UnitreeSdk2Bridge  # noqa: E402
import config  # noqa: E402
from unitree_sdk2py.core.channel import (  # noqa: E402
    ChannelFactoryInitialize,
    ChannelPublisher,
)
from codeep.robot.rangefinder_idl import RangefinderData  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--viewer", action="store_true",
                    help="launch the GLFW viewer (still publishes rangefinders)")
    args = ap.parse_args()

    if not args.viewer:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    os.chdir(_SIMDIR)
    mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    mj_data = mujoco.MjData(mj_model)
    mj_model.opt.timestep = config.SIMULATE_DT
    time.sleep(0.2)

    # detect rangefinder sensors
    rf_names = ["rf_center_dist", "rf_left_dist", "rf_right_dist"]
    rf_adrs = []
    for n in rf_names:
        sid = mujoco.mj_name2id(mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, n)
        if sid >= 0:
            rf_adrs.append(mj_model.sensor_adr[sid])
        else:
            rf_adrs.append(None)
    has_rf = any(a is not None for a in rf_adrs)
    if has_rf:
        print(f"[sim_headless] rangefinder sensors detected: {rf_adrs}", flush=True)

    stop = threading.Event()

    def _sig(_s, _f):
        stop.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    locker = threading.Lock()
    rf_pub = None

    def simulation_thread():
        nonlocal rf_pub
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        unitree = UnitreeSdk2Bridge(mj_model, mj_data)
        if config.PRINT_SCENE_INFORMATION:
            unitree.PrintSceneInformation()
        if has_rf:
            rf_pub = ChannelPublisher("rt/rangefinders", RangefinderData)
            rf_pub.Init()
        while not stop.is_set():
            step_start = time.perf_counter()
            locker.acquire()
            mujoco.mj_step(mj_model, mj_data)
            if has_rf and rf_pub is not None:
                vals = [99.0, 99.0, 99.0]  # [forward(center), left, right]; 99 = no hit
                for idx, adr in enumerate(rf_adrs):
                    if adr is not None:
                        v = float(mj_data.sensordata[adr])
                        # MuJoCo rangefinder returns -1 when no surface hit
                        vals[idx] = v if v > 0 else 99.0
                rf_pub.Write(RangefinderData(forward=vals[0], left=vals[1], right=vals[2]))
            locker.release()
            wait = mj_model.opt.timestep - (time.perf_counter() - step_start)
            if wait > 0:
                time.sleep(wait)

    viewer = None
    if args.viewer:
        viewer = mujoco.viewer.launch_passive(mj_model, mj_data)
        viewer.sync()

    t = threading.Thread(target=simulation_thread, daemon=True)
    t.start()
    print(f"[sim] running {args.duration}s ({'viewer' if args.viewer else 'headless'}). "
          f"DDS domain={config.DOMAIN_ID} iface={config.INTERFACE} "
          f"rangefinder={'on' if has_rf else 'off'}", flush=True)

    end = time.time() + args.duration
    while not stop.is_set() and time.time() < end:
        if viewer is not None:
            with locker:
                if viewer.is_running():
                    viewer.sync()
                else:
                    stop.set()
            time.sleep(config.VIEWER_DT)
        else:
            time.sleep(0.5)
    stop.set()
    t.join(timeout=1.0)
    if viewer is not None:
        viewer.close()
    print("[sim] stopped", flush=True)


if __name__ == "__main__":
    main()