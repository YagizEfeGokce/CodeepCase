"""Gate G (bonus) — Unitree G1 humanoid walks forward using the pre-trained
motion policy from unitree_rl_gym (deploy_mujoco pipeline). This is a separate,
self-contained MuJoCo run (own viewer + g1_description MJCF, torques applied
directly) -- an extra experiment on the G1 as the PDF's bonus item, parallel to
the Go2 DDS-bridge stack.

  .venv/bin/python gates/gate_g_g1.py [--duration 15]

Validation:
  * forward displacement > 0.5 m (G1 walks forward)
  * stays upright: base z stays > 0.5 m, |roll|<30 deg, |pitch|<30 deg
"""
from __future__ import annotations

import argparse
import contextlib
import math
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import torch
import yaml

RLGYM = Path(__file__).resolve().parents[1] / "external" / "unitree_rl_gym"
sys.path.insert(0, str(RLGYM))
from legged_gym import LEGGED_GYM_ROOT_DIR  # noqa: E402


def get_gravity_orientation(q):
    qw, qx, qy, qz = q
    g = np.zeros(3)
    g[0] = 2 * (-qz * qx + qw * qy)
    g[1] = -2 * (qz * qy + qw * qx)
    g[2] = 1 - 2 * (qw * qw + qz * qz)
    return g


def pd_control(t, q, kp, tdq, dq, kd):
    return (t - q) * kp + (tdq - dq) * kd


def quat_to_euler(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--headless", action="store_true", help="viewer olmadan (konteyner/CI)")
    args = ap.parse_args()

    with open(f"{LEGGED_GYM_ROOT_DIR}/deploy/deploy_mujoco/configs/g1.yaml") as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
    policy_path = cfg["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
    xml_path = cfg["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
    sim_dt = cfg["simulation_dt"]
    decim = cfg["control_decimation"]
    kps = np.array(cfg["kps"], dtype=np.float32)
    kds = np.array(cfg["kds"], dtype=np.float32)
    default_angles = np.array(cfg["default_angles"], dtype=np.float32)
    ang_vel_scale = cfg["ang_vel_scale"]
    dof_pos_scale = cfg["dof_pos_scale"]
    dof_vel_scale = cfg["dof_vel_scale"]
    action_scale = cfg["action_scale"]
    cmd_scale = np.array(cfg["cmd_scale"], dtype=np.float32)
    num_actions = cfg["num_actions"]
    num_obs = cfg["num_obs"]
    cmd = np.array(cfg["cmd_init"], dtype=np.float32)

    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    counter = 0

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = sim_dt
    policy = torch.jit.load(policy_path)

    print(f"[gate G] G1 loaded: nu={m.nu} nq={m.nq} | policy={Path(policy_path).name} | cmd={list(cmd)}")

    samples = []
    start_x = None
    ctx = contextlib.nullcontext() if args.headless else mujoco.viewer.launch_passive(m, d)
    with ctx as viewer:
        start = time.time()
        while (args.headless or (viewer is not None and viewer.is_running())) and time.time() - start < args.duration:
            step_start = time.time()
            tau = pd_control(target_dof_pos, d.qpos[7:], kps,
                             np.zeros_like(kds), d.qvel[6:], kds)
            d.ctrl[:] = tau
            mujoco.mj_step(m, d)
            counter += 1
            if counter % decim == 0:
                qj = (d.qpos[7:] - default_angles) * dof_pos_scale
                dqj = d.qvel[6:] * dof_vel_scale
                quat = d.qpos[3:7]
                omega = d.qvel[3:6] * ang_vel_scale
                grav = get_gravity_orientation(quat)
                period = 0.8
                phase = (counter * sim_dt) % period / period
                sin_p = math.sin(2 * math.pi * phase)
                cos_p = math.cos(2 * math.pi * phase)
                obs[:3] = omega
                obs[3:6] = grav
                obs[6:9] = cmd * cmd_scale
                obs[9:9 + num_actions] = qj
                obs[9 + num_actions:9 + 2 * num_actions] = dqj
                obs[9 + 2 * num_actions:9 + 3 * num_actions] = action
                obs[9 + 3 * num_actions:9 + 3 * num_actions + 2] = [sin_p, cos_p]
                action = policy(torch.from_numpy(obs).unsqueeze(0)).detach().numpy().squeeze()
                target_dof_pos = action * action_scale + default_angles

            if viewer is not None:
                viewer.sync()
            x, y, z = float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2])
            if start_x is None:
                start_x = x
            roll, pitch, yaw = quat_to_euler(*d.qpos[3:7])
            samples.append((time.time() - start, x, y, z, math.degrees(roll), math.degrees(pitch)))
            if int(time.time() - start) % 3 == 0 and len(samples) > 1 and abs(samples[-1][0] - int(samples[-1][0])) < 0.1:
                print(f"  t={samples[-1][0]:4.1f}s x={x:+.3f} y={y:+.3f} z={z:+.3f} "
                      f"roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f}")
            wait = sim_dt - (time.time() - step_start)
            if wait > 0:
                time.sleep(wait)

    if not samples:
        print("[FAIL] no samples"); sys.exit(2)
    arr = np.array(samples)
    _t, x, y, z, roll, pitch = [arr[:, i] for i in range(6)]
    disp = x[-1] - start_x
    metrics = {
        "forward_disp_m": float(disp),
        "z_min": float(z.min()), "z_max": float(z.max()),
        "roll_deg_max": float(np.max(np.abs(roll))),
        "pitch_deg_max": float(np.max(np.abs(pitch))),
    }
    checks = {
        "forward > 0.5m": metrics["forward_disp_m"] > 0.5,
        "upright z>0.5": metrics["z_min"] > 0.5,
        "|roll|<30deg": metrics["roll_deg_max"] < 30.0,
        "|pitch|<30deg": metrics["pitch_deg_max"] < 30.0,
    }
    print("\n=== Gate G (G1) metrics ===")
    for k, v in metrics.items():
        print(f"  {k:16s}: {v:.4f}")
    print("\n=== Gate G (G1) checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    overall = all(checks.values())
    print(f"\n=== Gate G (G1): {'PASS' if overall else 'FAIL'} ===")
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()