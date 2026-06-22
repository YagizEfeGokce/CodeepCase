"""Straight-walk demo: closed-loop heading + lateral control drives the Go2
to a far target on +x (5, 0) so it walks in a straight line, not drifting.

Run while the unitree_mujoco sim (clean scene) is up:
  .venv/bin/python gates/straight_walk.py [--target 5 0] [--duration 25]

Reports straightness metrics: forward progress, max |y| deviation, final distance.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner import RLRunner
from codeep.locomotion.rl_runner_onnx import RLRunnerOnnx
from codeep.control.nav import NavController


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, nargs=2, default=[5.0, 0.0])
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--stand", type=float, default=3.0)
    ap.add_argument("--max-vx", type=float, default=0.30)
    ap.add_argument("--kp-yaw", type=float, default=1.3)
    ap.add_argument("--kp-lat", type=float, default=0.9)
    ap.add_argument("--kp-lat-yaw", type=float, default=0.6)
    ap.add_argument("--yaw-bias", type=float, default=0.16)
    ap.add_argument("--ki-lat", type=float, default=0.0)
    ap.add_argument("--max-vy", type=float, default=0.0)
    ap.add_argument("--onnx", action="store_true",
                    help="use the diasAiMaster ONNX vy-tracking policy (enables vy, drops yaw_bias)")
    ap.add_argument("--policy", type=str, default=None, help="ONNX model path (with --onnx)")
    args = ap.parse_args()

    # vy-tracking policy: real lateral correction, no yaw_bias feedforward hack.
    use_vy = args.onnx
    yaw_bias = 0.0 if args.onnx else args.yaw_bias
    max_vy = 0.3 if args.onnx else args.max_vy
    runner = (RLRunnerOnnx(policy_path=args.policy, stand_time=args.stand) if args.onnx
              else RLRunner(stand_time=args.stand))
    runner.start()

    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); runner.stop(); sys.exit(2)
    print(f"[init] pose={runner.pose()}")

    print(f"[stand] standing up {args.stand}s ...")
    time.sleep(args.stand)
    start_pose = runner.pose()
    if start_pose is None:
        print("[FAIL] lost telemetry"); runner.stop(); sys.exit(2)

    nav = NavController(runner, max_vx=args.max_vx, goal_tol=0.25,
                        kp_yaw=args.kp_yaw, kp_lat=args.kp_lat, max_vy=max_vy,
                        kp_lat_yaw=args.kp_lat_yaw, yaw_bias=yaw_bias,
                        ki_lat=args.ki_lat, use_vy=use_vy)
    nav.set_target(args.target[0], args.target[1])
    policy_name = "ONNX vy-tracking" if args.onnx else "all_gait trot"
    print(f"[walk] {policy_name}: target=({args.target[0]:.2f},{args.target[1]:.2f}) "
          f"max_vx={args.max_vx} max_vy={max_vy} yaw_bias={yaw_bias} use_vy={use_vy} for {args.duration}s ...")

    samples = []
    start_t = time.time()
    while time.time() - start_t < args.duration and not nav.reached:
        r = nav.step()
        if r is not None:
            samples.append((time.time() - start_t, r["pose"][0], r["pose"][1], r["pose"][2],
                            r["dist"], math.degrees(r["yaw_err"]), r["vx"], r["wz"]))
            if int(time.time() - start_t) % 4 == 0 and len(samples) > 1 and abs(samples[-1][0] - int(samples[-1][0])) < 0.2:
                print(f"  t={samples[-1][0]:4.1f}s x={r['pose'][0]:+.3f} y={r['pose'][1]:+.3f} "
                      f"dist={r['dist']:.2f} yaw_err={math.degrees(r['yaw_err']):+.1f}deg "
                      f"vx={r['vx']:.2f} wz={r['wz']:+.2f}")
        time.sleep(0.1)

    runner.set_command(0.0, 0.0, 0.0)
    time.sleep(0.5)
    runner.stop()

    if not samples:
        print("[FAIL] no samples"); sys.exit(2)
    arr = np.array(samples)
    _t, x, y, z, dist, yaw_err, vx, wz = [arr[:, i] for i in range(8)]
    fwd = x[-1] - start_pose[0]
    metrics = {
        "forward_progress_m": float(fwd),
        "max_abs_y_m": float(np.max(np.abs(y))),
        "final_dist_to_target_m": float(dist[-1]),
        "z_min": float(z.min()), "z_max": float(z.max()),
        "reached": bool(nav.reached),
    }
    checks = {
        "forward > 1.0m": metrics["forward_progress_m"] > 1.0,
        "max|y| < 0.30m (straight)": metrics["max_abs_y_m"] < 0.30,
        "z_min > 0.20": metrics["z_min"] > 0.20,
    }
    print("\n=== Straight-walk metrics ===")
    for k, v in metrics.items():
        print(f"  {k:24s}: {v:.4f}")
    print("\n=== Straight-walk checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    print(f"\n=== Straight-walk: {'PASS' if all(checks.values()) else 'FAIL'} ===")
    sys.exit(0 if all(checks.values()) else 1)


if __name__ == "__main__":
    main()