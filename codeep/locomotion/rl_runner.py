"""RL trot locomotion layer for the Go2.

Wraps a pre-trained Unitree Go2 RL trot policy (from
shivam-sood00/unitree-sim2real, which targets unitree_mujoco's DDS bridge)
and exposes a simple high-level interface:

    runner = RLRunner()
    runner.start()                # stands up, then runs the policy in a thread
    runner.set_command(vx, vy, wz)  # desired body velocity (m/s, m/s, rad/s)
    pose = runner.pose()          # world [x,y,z] from SportModeState
    runner.stop()

The navigation layer (target following, waypoint manager, obstacle avoidance)
owns the (vx, vy, wz) commands; this layer only turns them into joint targets
via the pre-trained gait policy -- mirroring how the real Go2 sport-mode
service works (gait = black box, you send velocity).
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import numpy as np

# Make the external RL_policy_runner importable, then reuse its RLPolicy class
# and module-level config (kps/kds/default_angles/mapping/control_type/...).
_EXT = (Path(__file__).resolve().parents[2]
        / "external" / "unitree-sim2real" / "RL_policy_runner" / "sim2sim")
sys.path.insert(0, str(_EXT))
import run_rl_policy as _rp  # noqa: E402
from run_rl_policy import RLPolicy  # noqa: E402

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


class RLRunner:
    def __init__(self, policy_path: str | None = None, num_obs: int | None = None,
                 domain_id: int = 1, interface: str = "lo",
                 stand_time: float = 3.0, sim_dt: float = 0.005):
        if num_obs is not None:
            _rp.num_obs = num_obs
        self.policy_path = policy_path or _rp.policy_path
        self.policy = RLPolicy()
        self.domain_id = domain_id
        self.interface = interface
        self.stand_time = stand_time
        self.sim_dt = sim_dt
        self._stop = threading.Event()
        self._thread = None
        self._pub = None

    # ---- control ----
    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_command(self, vx: float, vy: float, wz: float):
        with self.policy.command_lock:
            self.policy.command[:] = [vx, vy, wz]

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    # ---- telemetry ----
    def pose(self):
        hs = self.policy.high_state
        if hs is None:
            return None
        return [float(hs.position[0]), float(hs.position[1]), float(hs.position[2])]

    def velocity(self):
        hs = self.policy.high_state
        if hs is None:
            return None
        return [float(hs.velocity[0]), float(hs.velocity[1]), float(hs.velocity[2])]

    def quaternion(self):
        ls = self.policy.low_state
        if ls is None:
            return None
        try:
            return [float(ls.imu_state.quaternion[0]), float(ls.imu_state.quaternion[1]),
                    float(ls.imu_state.quaternion[2]), float(ls.imu_state.quaternion[3])]
        except Exception:
            return None

    # ---- main loop ----
    def _loop(self):
        ChannelFactoryInitialize(self.domain_id, self.interface)
        high_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        high_sub.Init(self.policy.HighStateHandler, 10)
        low_sub = ChannelSubscriber("rt/lowstate", LowState_)
        low_sub.Init(self.policy.LowStateHandler, 10)
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        crc = CRC()

        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0

        self.policy.load_policy(self.policy_path)

        # --- stand-up phase: hold default angles so the dog stands ---
        stand_steps = int(self.stand_time / self.sim_dt)
        for _ in range(stand_steps):
            if self._stop.is_set():
                return
            for i in range(12):
                cmd.motor_cmd[i].q = float(_rp.default_angles[i])
                cmd.motor_cmd[i].kp = 30.0
                cmd.motor_cmd[i].kd = 1.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].tau = 0.0
            cmd.crc = crc.Crc(cmd)
            self._pub.Write(cmd)
            time.sleep(self.sim_dt)

        # --- policy phase ---
        raw, calc = self.policy.get_action()
        np.copyto(self.policy.prev_action, raw)
        decim = _rp.control_decimation
        kps = _rp.kps
        kds = _rp.kds
        loop_c = 0
        while not self._stop.is_set():
            step_start = time.perf_counter()
            if loop_c % decim == 0:
                raw, calc = self.policy.get_action()
                np.copyto(self.policy.prev_action, raw)
            if _rp.control_type == "position":
                for i in range(12):
                    mi = int(_rp.mapping[i])
                    cmd.motor_cmd[i].q = float(calc[mi]) + float(_rp.default_angles[i])
                    cmd.motor_cmd[i].kp = float(kps)
                    cmd.motor_cmd[i].kd = float(kds)
                    cmd.motor_cmd[i].dq = 0.0
                    cmd.motor_cmd[i].tau = 0.0
            else:  # torque
                for i in range(12):
                    mi = int(_rp.mapping[i])
                    cmd.motor_cmd[i].q = 0.0
                    cmd.motor_cmd[i].kp = 0.0
                    cmd.motor_cmd[i].kd = 0.0
                    cmd.motor_cmd[i].dq = 0.0
                    cmd.motor_cmd[i].tau = float(calc[mi])
            cmd.crc = crc.Crc(cmd)
            self._pub.Write(cmd)
            remaining = self.sim_dt - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)
            loop_c += 1