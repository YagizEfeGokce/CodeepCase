"""RL locomotion runner for a vy-tracking (omnidirectional) Go2 policy.

Wraps the pre-trained 'walk.pt' policy from saifahmadgit/go2-sim2real-deploy:
omnidirectional velocity tracking (vx, vy, wz) with Per-Leg Stiffness (PLS),
16 actions (12 joint targets + 4 per-leg Kp), 49-dim obs, rsl_rl ActorCritic
[512,256,128]. Unlike the all_gait policy, this one tracks `vy`, so the
NavController can do real closed-loop lateral correction -> straight walk.

Exposes the same high-level interface as rl_runner.RLRunner:
    r = RLRunnerVy(); r.start(); r.set_command(vx, vy, wz); r.pose(); r.stop()
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import torch

# Reuse the deploy script's pure functions/constants (main() is guarded).
_DEPLOY = (Path(__file__).resolve().parents[2]
           / "external" / "go2-sim2real-deploy" / "example"
           / "go2" / "low_level" / "final")
sys.path.insert(0, str(_DEPLOY))
import go2_policy_walk as _gpw  # noqa: E402

from unitree_sdk2py.core.channel import (  # noqa: E402
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_  # noqa: E402
from unitree_sdk2py.idl.unitree_go.msg.dds_ import (  # noqa: E402
    LowCmd_,
    LowState_,
    SportModeState_,
)
from unitree_sdk2py.utils.crc import CRC  # noqa: E402
from unitree_sdk2py.utils.thread import RecurrentThread  # noqa: E402

DEFAULT_WALK_PT = str(_DEPLOY / "walk.pt")

POLICY_HZ = 50.0
LOWCMD_HZ = 500.0
STAND_KP = 40.0
STAND_KD = 0.5
STAND_SECONDS = 4.0
MAX_STEP_RAD = 0.1
ACTION_CLIP = 100.0


class RLRunnerVy:
    def __init__(self, policy_path: str | None = None, domain_id: int = 1,
                 interface: str = "lo", stand_time: float = STAND_SECONDS):
        self.policy_path = policy_path or DEFAULT_WALK_PT
        self.domain_id = domain_id
        self.interface = interface
        self.stand_time = stand_time
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._pose = None
        self._vel = None
        self._quat = None
        self._lowstate = None
        self._command = np.zeros(3, dtype=np.float32)  # [vx, vy, wz]

    # ---- telemetry ----
    def pose(self):
        with self._lock:
            return None if self._pose is None else list(self._pose)

    def velocity(self):
        with self._lock:
            return None if self._vel is None else list(self._vel)

    def quaternion(self):
        with self._lock:
            return None if self._quat is None else list(self._quat)

    def set_command(self, vx: float, vy: float, wz: float):
        with self._lock:
            self._command[:] = [vx, vy, wz]

    def _on_highstate(self, msg: SportModeState_):
        with self._lock:
            self._pose = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
            self._vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]

    def _on_lowstate(self, msg: LowState_):
        with self._lock:
            try:
                self._quat = [float(msg.imu_state.quaternion[0]), float(msg.imu_state.quaternion[1]),
                              float(msg.imu_state.quaternion[2]), float(msg.imu_state.quaternion[3])]
            except Exception:
                self._quat = None
            self._lowstate = msg

    # ---- lifecycle ----
    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self):
        ChannelFactoryInitialize(self.domain_id, self.interface)
        high_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        high_sub.Init(self._on_highstate, 10)
        low_sub = ChannelSubscriber("rt/lowstate", LowState_)
        low_sub.Init(self._on_lowstate, 10)
        pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        pub.Init()
        crc = CRC()
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0
        for i in range(20):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 0.0

        policy = _gpw.load_policy(self.policy_path)
        DEFAULT = _gpw.DEFAULT_DOF_POS
        STAND = _gpw.STAND_DOF_POS

        shared = {"tq": STAND.clone(), "kp": torch.full((12,), STAND_KP, dtype=torch.float32),
                  "kd": torch.full((12,), STAND_KD, dtype=torch.float32)}

        def writer():
            tq = shared["tq"]; kp = shared["kp"]; kd = shared["kd"]
            for i in range(12):
                cmd.motor_cmd[i].q = float(tq[i])
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = float(kp[i])
                cmd.motor_cmd[i].kd = float(kd[i])
                cmd.motor_cmd[i].tau = 0.0
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)

        writer_thread = RecurrentThread(interval=1.0 / LOWCMD_HZ, target=writer, name="lowcmd_vy")
        writer_thread.Start()

        # wait for first lowstate
        t0 = time.time()
        while self._lowstate is None and time.time() - t0 < 8.0 and not self._stop.is_set():
            time.sleep(0.02)
        if self._lowstate is None:
            print("[rl_vy] no lowstate; aborting loop"); return

        # --- stand ramp: current q -> STAND over stand_time ---
        raw0 = _gpw.lowstate_to_raw(self._lowstate)
        start_q = torch.tensor([m["q_rad"] for m in raw0["motors"]], dtype=torch.float32)
        prev_q = start_q.clone()
        ramp_steps = max(1, int(self.stand_time * POLICY_HZ))
        for k in range(ramp_steps):
            if self._stop.is_set():
                return
            alpha = (k + 1) / float(ramp_steps)
            desired = (1 - alpha) * start_q + alpha * STAND
            desired = _gpw.slew_limit(prev_q, desired, MAX_STEP_RAD)
            shared["tq"] = desired.clone()
            shared["kp"][:] = STAND_KP
            shared["kd"][:] = STAND_KD
            prev_q = desired.clone()
            time.sleep(1.0 / POLICY_HZ)

        # --- policy loop ---
        last_action = torch.zeros(_gpw.NUM_ACT, dtype=torch.float32)
        prev_target_q = STAND.clone()
        dt = 1.0 / POLICY_HZ
        while not self._stop.is_set():
            step_start = time.perf_counter()
            ls = self._lowstate
            if ls is None:
                time.sleep(dt); continue
            raw = _gpw.lowstate_to_raw(ls)
            with self._lock:
                command = list(self._command)
            obs = _gpw.build_obs(raw, command, last_action)
            with torch.no_grad():
                action_raw = _gpw.policy.act_inference(obs.unsqueeze(0)).squeeze(0)
            action_clip = torch.clamp(action_raw, -ACTION_CLIP, ACTION_CLIP)
            pos_action = action_clip[:_gpw.NUM_POS_ACTIONS]
            policy_target_q = DEFAULT + _gpw.ACTION_SCALE * pos_action
            target_q = _gpw.slew_limit(prev_target_q, policy_target_q, MAX_STEP_RAD)
            prev_target_q = target_q.clone()
            shared["tq"] = target_q.clone()
            if _gpw.PLS_ENABLE and action_clip.shape[0] > _gpw.NUM_POS_ACTIONS:
                kp_12, kd_12 = _gpw.compute_pls_kp_kd(action_clip[_gpw.NUM_POS_ACTIONS:])
                shared["kp"] = kp_12.clone()
                shared["kd"] = kd_12.clone()
            last_action = action_clip.clone()
            wait = dt - (time.perf_counter() - step_start)
            if wait > 0:
                time.sleep(wait)