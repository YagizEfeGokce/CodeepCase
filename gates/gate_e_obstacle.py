"""Gate E — obstacle avoidance: navigate to (5,0) with a box obstacle at
(2.5,0). The ObstacleAvoider detects it, stops, detours around, and resumes.

Run while the unitree_mujoco sim (scene_obstacle.xml) is up:
  .venv/bin/python gates/gate_e_obstacle.py

Validation:
  * reaches final target (5,0) within 0.35 m
  * never collides: min distance to obstacle center > obstacle radius (0.25 m)
  * stays upright (z > 0.20)
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner import RLRunner
from codeep.control.nav import NavController
from codeep.control.avoider import ObstacleAvoider

TARGET = (5.0, 0.0)
OBSTACLE = (2.5, 0.0, 0.25)  # x, y, radius


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=35.0)
    ap.add_argument("--stand", type=float, default=3.0)
    ap.add_argument("--max-vx", type=float, default=0.30)
    ap.add_argument("--yaw-bias", type=float, default=0.16)
    args = ap.parse_args()

    runner = RLRunner(stand_time=args.stand)
    runner.start()
    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); runner.stop(); sys.exit(2)
    print(f"[init] pose={runner.pose()}")
    print(f"[stand] standing up {args.stand}s ...")
    time.sleep(args.stand)

    nav = NavController(runner, max_vx=args.max_vx, goal_tol=0.25, kp_yaw=1.3,
                        kp_lat_yaw=0.6, yaw_bias=args.yaw_bias)
    avoider = ObstacleAvoider(nav, obstacles=[OBSTACLE], reaction_dist=1.0,
                              clearance=0.20, margin=0.45, stop_time=0.0, dt=0.1)
    avoider.set_target(*TARGET)
    print(f"[gate E] target={TARGET} obstacle={OBSTACLE} -- navigating ...")

    samples = []
    start_t = time.time()
    last_state = None
    while time.time() - start_t < args.duration and not avoider.reached:
        r = avoider.step()
        if r is not None:
            p = r["pose"]
            samples.append((time.time() - start_t, p[0], p[1], p[2], r["state"],
                            math.hypot(TARGET[0] - p[0], TARGET[1] - p[1])))
            if r["state"] != last_state:
                last_state = r["state"]
                print(f"  [state] {last_state:10s} pose=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                      f"dist_to_target={math.hypot(TARGET[0]-p[0],TARGET[1]-p[1]):.2f}")
            if int(time.time() - start_t) % 5 == 0 and len(samples) > 1 and abs(samples[-1][0] - int(samples[-1][0])) < 0.2:
                print(f"  t={samples[-1][0]:4.1f}s state={r['state']:10s} "
                      f"x={p[0]:+.3f} y={p[1]:+.3f} dist={samples[-1][5]:.2f} "
                      f"min_obs={avoider.min_obs_dist:.2f}")
        time.sleep(0.1)

    runner.set_command(0.0, 0.0, 0.0)
    time.sleep(0.5)
    runner.stop()

    if not samples:
        print("[FAIL] no samples"); sys.exit(2)
    arr = np.array([[s[0], s[1], s[2], s[3]] for s in samples])
    _t, x, y, z = [arr[:, i] for i in range(4)]
    final_pose = samples[-1][1:4]
    final_dist = math.hypot(TARGET[0] - final_pose[0], TARGET[1] - final_pose[1])

    metrics = {
        "final_dist_to_target_m": float(final_dist),
        "min_obs_dist_m": float(avoider.min_obs_dist),
        "detected": bool(avoider.detected),
        "detour": avoider.detour,
        "reached": bool(avoider.reached),
        "z_min": float(z.min()),
    }
    checks = {
        "reached target (dist<0.35)": metrics["final_dist_to_target_m"] < 0.35,
        "no collision (min_obs>0.25)": metrics["min_obs_dist_m"] > 0.25,
        "obstacle detected": metrics["detected"],
        "z_min>0.20": metrics["z_min"] > 0.20,
    }
    print("\n=== Gate E metrics ===")
    for k, v in metrics.items():
        print(f"  {k:24s}: {v}")
    print("\n=== Gate E checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    overall = all(checks.values())
    print(f"\n=== Gate E: {'PASS' if overall else 'FAIL'} ===")
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()