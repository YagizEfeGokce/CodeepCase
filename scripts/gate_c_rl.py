"""Gate C (RL) — make Go2 walk forward using the pre-trained trot policy,
and validate forward motion + upright stability.

Run while the unitree_mujoco sim is up:
  .venv/bin/python scripts/gate_c_rl.py [--vx 0.3] [--duration 15]

Validation:
  * forward displacement > 0.5 m
  * base height z > 0.20 m (didn't fall)
  * |roll| < 30 deg, |pitch| < 45 deg
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner import RLRunner


def quat_to_euler(w, x, y, z):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.30)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--stand", type=float, default=3.0)
    ap.add_argument("--policy", type=str, default=None)
    args = ap.parse_args()

    runner = RLRunner(stand_time=args.stand)
    runner.start()

    # wait for telemetry
    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); runner.stop(); sys.exit(2)
    print(f"[init] pose={runner.pose()}")

    # stand-up phase
    print(f"[stand] standing up {args.stand}s ...")
    time.sleep(args.stand)

    start_pose = runner.pose()
    if start_pose is None:
        print("[FAIL] lost telemetry before walk"); runner.stop(); sys.exit(2)
    start_t = time.time()
    runner.set_command(args.vx, args.vy, args.wz)
    print(f"[walk] command vx={args.vx} vy={args.vy} wz={args.wz} for {args.duration}s ...")

    samples = []
    while time.time() - start_t < args.duration:
        p = runner.pose(); q = runner.quaternion()
        if p and q:
            roll, pitch, yaw = quat_to_euler(*q)
            samples.append((time.time() - start_t, p[0], p[1], p[2], roll, pitch, yaw))
            if int(time.time() - start_t) % 3 == 0 and len(samples) > 1 and abs(samples[-1][0] - int(samples[-1][0])) < 0.2:
                print(f"  t={samples[-1][0]:4.1f}s x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f} "
                      f"roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f}")
        time.sleep(0.05)

    runner.set_command(0.0, 0.0, 0.0)
    time.sleep(1.0)
    runner.stop()

    if not samples:
        print("[FAIL] no samples"); sys.exit(2)
    arr = np.array(samples)
    _t, x, y, z, roll, pitch, yaw = [arr[:, i] for i in range(7)]
    disp = x[-1] - start_pose[0]
    achieved_vx = disp / max(1e-3, samples[-1][0])

    metrics = {
        "start_x": float(start_pose[0]), "end_x": float(x[-1]),
        "disp_x_m": float(disp), "achieved_vx": float(achieved_vx),
        "z_min": float(z.min()), "z_max": float(z.max()),
        "roll_deg_max": float(math.degrees(np.max(np.abs(roll)))),
        "pitch_deg_max": float(math.degrees(np.max(np.abs(pitch)))),
        "y_drift_m": float(abs(y[-1] - start_pose[1])),
    }
    checks = {
        "disp_x > 0.5m": metrics["disp_x_m"] > 0.5,
        "z_min > 0.20": metrics["z_min"] > 0.20,
        "|roll|<30deg": metrics["roll_deg_max"] < 30.0,
        "|pitch|<45deg": metrics["pitch_deg_max"] < 45.0,
    }
    print("\n=== Gate C (RL) metrics ===")
    for k, v in metrics.items():
        print(f"  {k:16s}: {v:.4f}")
    print("\n=== Gate C (RL) checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    overall = all(checks.values())
    print(f"\n=== Gate C (RL): {'PASS' if overall else 'FAIL'} ===")
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()