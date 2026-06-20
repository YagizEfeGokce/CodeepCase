"""Gate B — launch sim (detached) is done separately; this script is the
controller + monitor that makes Go2 stand still and validates it for N seconds.

Run AFTER the sim is running:
    .venv/bin/python gates/gate_b_stand.py [--duration 30] [--ramp 1.2]

Validation criteria (robot "stands still without a problem"):
  * base height z stays in [0.25, 0.50] m during hold
  * |vx|, |vy| <= 0.10 m/s during hold
  * horizontal drift from hold-start <= 0.20 m
  * base roll & pitch stay within 20 deg (not tipped over)
"""
from __future__ import annotations

import argparse
import math
import threading
import time
import sys

import numpy as np

sys.path.insert(0, ".")
from codeep.robot.go2_client import Go2Client, STAND_UP, STAND_DOWN, NUM_MOTORS

DT = 0.002  # 500 Hz publish rate (matches unitree_mujoco example)


def quat_to_euler(w, x, y, z):
    """Return roll, pitch, yaw (rad) from a quaternion."""
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
    ap.add_argument("--duration", type=float, default=30.0, help="hold time (s)")
    ap.add_argument("--ramp", type=float, default=1.2, help="stand-up ramp time (s)")
    args = ap.parse_args()

    client = Go2Client()

    # Wait for telemetry from the sim
    t0 = time.time()
    while client.pose() is None and time.time() - t0 < 5.0:
        time.sleep(0.05)
    if client.pose() is None:
        print("[FAIL] No SportModeState telemetry received within 5s. Is the sim running?")
        sys.exit(2)
    print(f"[init] first pose = {client.pose()}")

    # Publisher thread: ramp stand_down->stand_up, then hold stance at 500Hz.
    stop = threading.Event()
    phase_done = threading.Event()

    def publish_loop():
        t = 0.0
        while not stop.is_set():
            step_start = time.perf_counter()
            if t < args.ramp:
                phase = math.tanh(t / 1.2)
                q = phase * STAND_UP + (1 - phase) * STAND_DOWN
                kp = phase * 50.0 + (1 - phase) * 20.0
                client.send_motors(q, [kp] * NUM_MOTORS, [3.5] * NUM_MOTORS)
            else:
                if not phase_done.is_set():
                    phase_done.set()
                client.hold_stance(kp=50.0, kd=3.5)
            t += DT
            remaining = DT - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)

    th = threading.Thread(target=publish_loop, daemon=True)
    th.start()

    # Wait until ramp completes (hold phase begins)
    if not phase_done.wait(timeout=args.ramp + 2.0):
        print("[FAIL] ramp did not complete in time")
        stop.set(); sys.exit(2)

    # Collect samples during the hold
    start_pose = client.pose()
    if start_pose is None:
        print("[FAIL] lost telemetry before hold"); stop.set(); sys.exit(2)
    samples = []
    t_start = time.time()
    print(f"[hold] start_pose={start_pose} | monitoring {args.duration}s ...")
    while time.time() - t_start < args.duration:
        p = client.pose(); v = client.velocity(); q = client.quaternion()
        if p and v and q:
            roll, pitch, yaw = quat_to_euler(*q)
            samples.append((time.time() - t_start, p[0], p[1], p[2], v[0], v[1], v[2], roll, pitch, yaw))
        if int(time.time() - t_start) % 5 == 0 and len(samples) > 1:
            last = samples[-1]
            print(f"  t={last[0]:4.1f}s pos=({last[1]:+.3f},{last[2]:+.3f},{last[3]:+.3f}) "
                  f"vel=({last[4]:+.3f},{last[5]:+.3f},{last[6]:+.3f}) "
                  f"r/p/y(deg)={math.degrees(last[7]):+.1f}/{math.degrees(last[8]):+.1f}/{math.degrees(last[9]):+.1f}")
        time.sleep(0.05)

    stop.set()
    th.join(timeout=1.0)

    if not samples:
        print("[FAIL] no samples collected during hold")
        sys.exit(2)

    arr = np.array(samples)
    t, px, py, pz, vx, vy, vz, roll, pitch, yaw = [arr[:, i] for i in range(10)]

    drift = math.hypot(px[-1] - start_pose[0], py[-1] - start_pose[1])
    metrics = {
        "z_min": float(pz.min()), "z_max": float(pz.max()),
        "absvx_max": float(np.max(np.abs(vx))), "absvy_max": float(np.max(np.abs(vy))),
        "drift_m": float(drift),
        "roll_deg_max": float(math.degrees(np.max(np.abs(roll)))),
        "pitch_deg_max": float(math.degrees(np.max(np.abs(pitch)))),
        "n_samples": len(samples),
    }

    checks = {
        "height in [0.25,0.50]": 0.25 <= metrics["z_min"] and metrics["z_max"] <= 0.50,
        "|vx|<=0.10": metrics["absvx_max"] <= 0.10,
        "|vy|<=0.10": metrics["absvy_max"] <= 0.10,
        "drift<=0.20m": metrics["drift_m"] <= 0.20,
        "|roll|<=20deg": metrics["roll_deg_max"] <= 20.0,
        "|pitch|<=20deg": metrics["pitch_deg_max"] <= 20.0,
    }

    print("\n=== Gate B metrics ===")
    for k, v in metrics.items():
        print(f"  {k:18s}: {v:.4f}")
    print("\n=== Gate B checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    overall = all(checks.values())
    print(f"\n=== Gate B: {'PASS' if overall else 'FAIL'} ===")
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()