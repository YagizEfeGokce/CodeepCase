"""Gate C — make Go2 walk forward with our open-loop trot, and validate.

Flow: stand-up ramp -> hold stance -> ramp stride 0->vx -> walk for `duration`
-> measure forward displacement + upright stability. Run while the sim is up.

  .venv/bin/python gates/gate_c_walk.py [--vx 0.2] [--duration 15]

Validation (robot walks forward without falling):
  * forward displacement > 0.5 m
  * base height z stays > 0.20 m (didn't collapse/fall)
  * |roll| max < 30 deg, |pitch| max < 45 deg (trot pitch oscillates; allow margin)
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time

import numpy as np

sys.path.insert(0, ".")
from codeep.robot.go2_client import Go2Client, STAND_DOWN, NUM_MOTORS
from codeep.control.trot import TrotGait, CRAWL_PHASE_OFF
from codeep.control.kinematics import stance_pose

DT = 0.002
KP_WALK = 50.0
KD_WALK = 3.5
H_WALK = 0.28  # crouched stance foot-below-hip (low CoM, stable walk)


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
    ap.add_argument("--vx", type=float, default=0.10)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--stand", type=float, default=2.0)
    ap.add_argument("--ramp", type=float, default=1.0)
    ap.add_argument("--kp", type=float, default=KP_WALK)
    ap.add_argument("--kd", type=float, default=KD_WALK)
    args = ap.parse_args()

    kp_walk = args.kp
    kd_walk = args.kd

    client = Go2Client()
    t0 = time.time()
    while client.pose() is None and time.time() - t0 < 5.0:
        time.sleep(0.05)
    if client.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); sys.exit(2)
    print(f"[init] pose={client.pose()}")

    stop = threading.Event()
    gait = TrotGait(T=1.0, duty=0.75, height=H_WALK, swing_h=0.05,
                    max_stride=0.10, phase_off=CRAWL_PHASE_OFF, press=0.02)
    stand_q = stance_pose(H_WALK)

    # phase machine: 0=stand_ramp, 1=stand_hold, 2=walk_ramp, 3=walk, 4=done
    state = {"mode": "stand_ramp", "t_local": 0.0, "walk_start_x": None}

    def publish_loop():
        # stand-up ramp (1.2s): stand_down -> stand_up
        ramp_up = 1.2
        t = 0.0
        while not stop.is_set():
            s = time.perf_counter()
            mode = state["mode"]
            if mode == "stand_ramp":
                phase = math.tanh(t / 1.2)
                q = phase * stand_q + (1 - phase) * STAND_DOWN
                kp = phase * 50.0 + (1 - phase) * 20.0
                client.send_motors(q, [kp] * NUM_MOTORS, [kd_walk] * NUM_MOTORS)
                if t >= ramp_up:
                    state["mode"] = "stand_hold"; state["t_local"] = 0.0
            elif mode == "stand_hold":
                client.send_motors(stand_q, [kp_walk] * NUM_MOTORS, [kd_walk] * NUM_MOTORS)
            elif mode == "walk_ramp":
                # blend stride 0 -> vx over `ramp` s while gait phase runs
                blend = min(1.0, state["t_local"] / args.ramp)
                tg = state["t_local"]  # gait time (continuous)
                q = gait.joint_targets(tg, vx=args.vx * blend)
                client.send_motors(q, [kp_walk] * NUM_MOTORS, [kd_walk] * NUM_MOTORS)
                if state["t_local"] >= args.ramp:
                    state["mode"] = "walk"; state["t_local"] = 0.0
            elif mode == "walk":
                tg = args.ramp + state["t_local"]  # keep gait phase continuous
                q = gait.joint_targets(tg, vx=args.vx)
                client.send_motors(q, [kp_walk] * NUM_MOTORS, [kd_walk] * NUM_MOTORS)
            elif mode == "done":
                client.send_motors(stand_q, [kp_walk] * NUM_MOTORS, [kd_walk] * NUM_MOTORS)
            state["t_local"] += DT
            t += DT
            r = DT - (time.perf_counter() - s)
            if r > 0:
                time.sleep(r)

    def _sig(_s, _f):
        stop.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    th = threading.Thread(target=publish_loop, daemon=True)
    th.start()

    # orchestrate mode transitions + telemetry
    time.sleep(1.2)  # stand_ramp
    state["mode"] = "stand_hold"; state["t_local"] = 0.0
    print(f"[stand_hold] holding stance {args.stand}s ...")
    time.sleep(args.stand)
    walk0_pose = client.pose()
    if walk0_pose is None:
        print("[FAIL] lost telemetry before walk"); stop.set(); sys.exit(2)
    walk0_t = time.time()
    state["mode"] = "walk_ramp"; state["t_local"] = 0.0
    print(f"[walk_ramp] ramping stride 0 -> vx={args.vx} over {args.ramp}s ...")
    time.sleep(args.ramp)
    state["mode"] = "walk"; state["t_local"] = 0.0
    print(f"[walk] trotting forward {args.duration}s ...")

    samples = []
    while time.time() - walk0_t < args.ramp + args.duration:
        p = client.pose(); q = client.quaternion()
        if p and q:
            roll, pitch, yaw = quat_to_euler(*q)
            samples.append((time.time() - walk0_t, p[0], p[1], p[2], roll, pitch, yaw))
            if int(time.time() - walk0_t) % 3 == 0 and len(samples) > 1 and abs(samples[-1][0] - int(samples[-1][0])) < 0.2:
                print(f"  t={samples[-1][0]:4.1f}s x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f} "
                      f"roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f}")
        time.sleep(0.05)

    state["mode"] = "done"
    time.sleep(1.0)
    stop.set(); th.join(timeout=1.0)

    if not samples:
        print("[FAIL] no samples"); sys.exit(2)
    arr = np.array(samples)
    _t, x, y, z, roll, pitch, yaw = [arr[:, i] for i in range(7)]
    disp = x[-1] - walk0_pose[0]
    achieved_vx = disp / max(1e-3, (samples[-1][0]))

    metrics = {
        "start_x": float(walk0_pose[0]), "end_x": float(x[-1]),
        "disp_x_m": float(disp), "achieved_vx": float(achieved_vx),
        "z_min": float(z.min()), "z_max": float(z.max()),
        "roll_deg_max": float(math.degrees(np.max(np.abs(roll)))),
        "pitch_deg_max": float(math.degrees(np.max(np.abs(pitch)))),
        "y_drift_m": float(abs(y[-1] - walk0_pose[1])),
    }
    checks = {
        "disp_x > 0.5m": metrics["disp_x_m"] > 0.5,
        "z_min > 0.20": metrics["z_min"] > 0.20,
        "|roll|<30deg": metrics["roll_deg_max"] < 30.0,
        "|pitch|<45deg": metrics["pitch_deg_max"] < 45.0,
    }
    print("\n=== Gate C metrics ===")
    for k, v in metrics.items():
        print(f"  {k:16s}: {v:.4f}")
    print("\n=== Gate C checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    overall = all(checks.values())
    print(f"\n=== Gate C: {'PASS' if overall else 'FAIL'} ===")
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()