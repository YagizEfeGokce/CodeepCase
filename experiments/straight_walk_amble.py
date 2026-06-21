"""Straight-walk with amble_with_yaw (vy-tracking) policy + real lateral control.

amble_with_yaw supports lateral vy (verified: vy=+0.4 -> +0.68 m lateral),
and it runs in unitree_mujoco (trained for this bridge). NavController with
use_vy=True corrects lateral drift with a real vy command -> straight line.

  .venv/bin/python gates/straight_walk_amble.py [--target 5 0] [--duration 30]
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner import RLRunner
from codeep.control.nav import NavController

EXT = ("external/unitree-sim2real/RL_policy_runner/policies/individual_gaits/"
       "amble_with_yaw.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, nargs=2, default=[5.0, 0.0])
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--stand", type=float, default=3.0)
    ap.add_argument("--max-vx", type=float, default=0.3)
    ap.add_argument("--max-vy", type=float, default=0.4)
    ap.add_argument("--kp-lat", type=float, default=1.5)
    ap.add_argument("--kp-yaw", type=float, default=1.3)
    ap.add_argument("--max-wz", type=float, default=0.8)
    ap.add_argument("--kp-lat-yaw", type=float, default=0.0)
    ap.add_argument("--ki-lat", type=float, default=0.0)
    args = ap.parse_args()

    runner = RLRunner(policy_path=EXT, num_obs=42, stand_time=args.stand)
    runner.start()
    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); runner.stop(); sys.exit(2)
    print(f"[init] pose={runner.pose()}")
    print(f"[stand] standing up {args.stand}s (amble_with_yaw) ...")
    time.sleep(args.stand)
    start_pose = runner.pose()
    if start_pose is None:
        print("[FAIL] lost telemetry"); runner.stop(); sys.exit(2)

    nav = NavController(runner, max_vx=args.max_vx, goal_tol=0.20, kp_yaw=args.kp_yaw,
                        kp_lat=args.kp_lat, max_vy=args.max_vy, use_vy=True,
                        yaw_bias=0.0, kp_lat_yaw=args.kp_lat_yaw, ki_lat=args.ki_lat,
                        max_wz=args.max_wz, min_align=0.35)
    nav.set_target(args.target[0], args.target[1])
    print(f"[walk] amble+vy steering to {args.target} for {args.duration}s ...")

    samples = []
    start_t = time.time()
    while time.time() - start_t < args.duration and not nav.reached:
        r = nav.step()
        if r is not None:
            p = r["pose"]
            samples.append((time.time() - start_t, p[0], p[1], p[2], r["vx"], r["vy"], r["wz"]))
            if int(time.time() - start_t) % 4 == 0 and len(samples) > 1 and abs(samples[-1][0] - int(samples[-1][0])) < 0.2:
                print(f"  t={samples[-1][0]:4.1f}s x={p[0]:+.3f} y={p[1]:+.3f} "
                      f"dist={r['dist']:.2f} cmd=({r['vx']:.2f},{r['vy']:+.2f},{r['wz']:+.2f})")
        time.sleep(0.1)

    runner.set_command(0.0, 0.0, 0.0)
    time.sleep(0.5)
    runner.stop()

    if not samples:
        print("[FAIL] no samples"); sys.exit(2)
    arr = np.array(samples)
    _t, x, y, z, vx, vy, wz = [arr[:, i] for i in range(7)]
    fwd = x[-1] - start_pose[0]
    metrics = {
        "forward_progress_m": float(fwd),
        "max_abs_y_m": float(np.max(np.abs(y))),
        "final_dist_to_target_m": float(math.hypot(args.target[0] - x[-1], args.target[1] - y[-1])),
        "z_min": float(z.min()), "z_max": float(z.max()),
        "reached": bool(nav.reached),
    }
    checks = {
        "forward > 1.0m": metrics["forward_progress_m"] > 1.0,
        "max|y| < 0.20m (straight)": metrics["max_abs_y_m"] < 0.20,
        "z_min > 0.20": metrics["z_min"] > 0.20,
    }
    print("\n=== straight-walk (amble+vy) metrics ===")
    for k, v in metrics.items():
        print(f"  {k:24s}: {v:.4f}")
    print("\n=== straight-walk checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    print(f"\n=== straight-walk (amble+vy): {'PASS' if all(checks.values()) else 'FAIL'} ===")
    sys.exit(0 if all(checks.values()) else 1)


if __name__ == "__main__":
    main()