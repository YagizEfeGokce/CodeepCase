"""Straight-walk with the vy-tracking (omnidirectional) policy.

Uses RLRunnerVy (walk.pt, supports lateral vy) + NavController with use_vy=True
so lateral drift is corrected by a real lateral command (like a normal gait),
not a yaw-bias hack. Target (5,0) on the clean scene.

  .venv/bin/python gates/straight_walk_vy.py [--target 5 0] [--duration 25]
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner_vy import RLRunnerVy
from codeep.control.nav import NavController


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, nargs=2, default=[5.0, 0.0])
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--stand", type=float, default=4.0)
    ap.add_argument("--max-vx", type=float, default=0.4)
    ap.add_argument("--max-vy", type=float, default=0.4)
    ap.add_argument("--kp-lat", type=float, default=1.0)
    args = ap.parse_args()

    runner = RLRunnerVy(stand_time=args.stand)
    runner.start()
    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); runner.stop(); sys.exit(2)
    print(f"[init] pose={runner.pose()}")
    print(f"[stand] standing up {args.stand}s (walk.pt policy) ...")
    time.sleep(args.stand)
    start_pose = runner.pose()
    if start_pose is None:
        print("[FAIL] lost telemetry"); runner.stop(); sys.exit(2)

    nav = NavController(runner, max_vx=args.max_vx, goal_tol=0.20, kp_yaw=1.3,
                        kp_lat=args.kp_lat, max_vy=args.max_vy, use_vy=True,
                        yaw_bias=0.0, kp_lat_yaw=0.0, min_align=0.35)
    nav.set_target(args.target[0], args.target[1])
    print(f"[walk] vy-policy steering to {args.target} for {args.duration}s ...")

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
    print("\n=== straight-walk (vy-policy) metrics ===")
    for k, v in metrics.items():
        print(f"  {k:24s}: {v:.4f}")
    print("\n=== straight-walk checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    print(f"\n=== straight-walk (vy-policy): {'PASS' if all(checks.values()) else 'FAIL'} ===")
    sys.exit(0 if all(checks.values()) else 1)


if __name__ == "__main__":
    main()