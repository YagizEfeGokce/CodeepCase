"""Probe: does the diasAiMaster ONNX velocity policy walk straight in unitree_mujoco?

Open-loop (no NavController, no yaw_bias): send a pure forward velocity command
and measure the policy's NATIVE lateral drift + forward progress + stability.
This is the sim-to-sim transfer test flagged in README §10 / experiments README.

Baseline to beat: all_gait policy + yaw_bias feedforward ≈ 0.18 m lateral drift
over 5 m (~3-4%). If this policy drifts less WITHOUT yaw_bias, it's the
vy-supported straight-walk winner and drops into the nav stack unchanged
(same interface as RLRunner).

Run while the unitree_mujoco sim (clean scene) is up:
  .venv/bin/python experiments/straight_walk_onnx.py [--vx 0.5] [--duration 10]
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner_onnx import RLRunnerOnnx


def quat_to_euler(w, x, y, z):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.5)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--stand", type=float, default=3.0)
    ap.add_argument("--policy", type=str, default=None)
    args = ap.parse_args()

    runner = RLRunnerOnnx(policy_path=args.policy, stand_time=args.stand)
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
        print("[FAIL] lost telemetry before walk"); runner.stop(); sys.exit(2)

    runner.set_command(args.vx, args.vy, args.wz)
    print(f"[walk] ONNX policy, command vx={args.vx} vy={args.vy} wz={args.wz} "
          f"for {args.duration}s (NO yaw_bias, open-loop) ...")

    samples = []
    start_t = time.time()
    last_print = 0.0
    while time.time() - start_t < args.duration:
        p = runner.pose(); q = runner.quaternion()
        if p and q:
            roll, pitch, yaw = quat_to_euler(*q)
            t = time.time() - start_t
            samples.append((t, p[0], p[1], p[2], roll, pitch, yaw))
            if t - last_print >= 1.0:
                print(f"  t={t:4.1f}s x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f} "
                      f"roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f}")
                last_print = t
        time.sleep(0.05)

    runner.set_command(0.0, 0.0, 0.0)
    time.sleep(1.0)
    runner.stop()

    if not samples:
        print("[FAIL] no samples"); sys.exit(2)
    arr = np.array(samples)
    _t, x, y, z, roll, pitch, yaw = [arr[:, i] for i in range(7)]
    dx = x[-1] - start_pose[0]
    dy = y[-1] - start_pose[1]
    max_abs_y = float(np.max(np.abs(y - start_pose[1])))
    fell = float(np.min(z)) < 0.20 or float(np.max(np.abs(roll))) > math.radians(30) \
        or float(np.max(np.abs(pitch))) > math.radians(45)

    print("\n=== straight-walk probe (ONNX diasAiMaster velocity policy) ===")
    print(f"  forward dx   : {dx:+.3f} m   (target vx={args.vx} for {args.duration}s)")
    print(f"  lateral dy   : {dy:+.3f} m   (all_gait baseline ~0.18 m w/ yaw_bias)")
    print(f"  max |dy|     : {max_abs_y:.3f} m")
    print(f"  drift ratio  : {abs(dy)/max(1e-3,abs(dx))*100:.1f}% of forward progress")
    print(f"  min base z   : {float(np.min(z)):.3f} m   (fell if <0.20)")
    print(f"  max |roll|   : {math.degrees(float(np.max(np.abs(roll)))):.1f} deg")
    print(f"  max |pitch|  : {math.degrees(float(np.max(np.abs(pitch)))):.1f} deg")
    print(f"  FELL OVER    : {fell}")
    if not fell and dx > 0.5 and abs(dy) < 0.18:
        print("  -> TRANSFER OK: walks straight without yaw_bias. Drop into nav stack.")
    elif not fell and dx > 0.2:
        print("  -> PARTIAL: moves forward but drifts; nav-stack closed-loop may fix.")
    elif not fell:
        print("  -> STALLS: stands but does not progress (sim-to-sim gap, like walk.pt).")
    else:
        print("  -> UNSTABLE: fell over (obs/mapping mismatch — recheck deploy.yaml).")


if __name__ == "__main__":
    main()