"""Go2 DDS client over unitree_sdk2_python (unitree_go IDL).

Wraps the low-level LowCmd/LowState/SportModeState topics exposed by the
unitree_mujoco `simulate_python` bridge. Provides a small high-level API:
stand pose hold, raw motor-target send, and pose/IMU telemetry read.

Topic / sensor facts (verified against simulate_python/unitree_sdk2py_bridge.py
and unitree_robots/go2/go2.xml):
  * rt/lowcmd        : LowCmd_  (we publish, 12 motors, PD: ctrl = tau + kp*(q-q_meas) + kd*(dq-dq_meas))
  * rt/lowstate      : LowState_ (motor q/dq/tau + imu_state quaternion/gyro/acc)
  * rt/sportmodestate: SportModeState_ (position[0:3], velocity[0:3] = world pose/vel of base imu site)

Motor index order (go2.xml jointpos): FR_hip, FR_thigh, FR_calf,
FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf.
"""
from __future__ import annotations

import threading
import numpy as np

from unitree_sdk2py.core.channel import (  # pi-lens-ignore: reportMissingImports
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.utils.crc import CRC

DOMAIN_ID = 1
INTERFACE = "lo"
TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"

NUM_MOTORS = 12

# Official Go2 joint poses (rad), from unitree_mujoco/example/python/stand_go2.py.
STAND_UP = np.array(
    [
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
    ],
    dtype=float,
)
STAND_DOWN = np.array(
    [
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375,
        0.0473455, 1.22187, -2.44375,
        -0.0473455, 1.22187, -2.44375,
    ],
    dtype=float,
)


class Go2Client:
    """High-level DDS client for the Go2 in unitree_mujoco."""

    def __init__(self, domain_id: int = DOMAIN_ID, interface: str = INTERFACE):
        ChannelFactoryInitialize(domain_id, interface)

        self.crc = CRC()
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0] = 0xFE
        self.cmd.head[1] = 0xEF
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01
            self.cmd.motor_cmd[i].q = 0.0
            self.cmd.motor_cmd[i].kp = 0.0
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kd = 0.0
            self.cmd.motor_cmd[i].tau = 0.0

        self.pub = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
        self.pub.Init()

        self._lock = threading.Lock()
        self._pose = None  # [x, y, z]
        self._vel = None  # [vx, vy, vz]
        self._quat = None  # [w, x, y, z] base orientation
        self._lowstate = None

        self.high_sub = ChannelSubscriber(TOPIC_HIGHSTATE, SportModeState_)
        self.high_sub.Init(self._on_highstate, 10)
        self.low_sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self.low_sub.Init(self._on_lowstate, 10)

    # ---- DDS callbacks -------------------------------------------------
    def _on_highstate(self, msg: SportModeState_):
        with self._lock:
            self._pose = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
            self._vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]

    def _on_lowstate(self, msg: LowState_):
        with self._lock:
            try:
                self._quat = [float(msg.imu_state.quaternion[0]),
                              float(msg.imu_state.quaternion[1]),
                              float(msg.imu_state.quaternion[2]),
                              float(msg.imu_state.quaternion[3])]
            except Exception:
                self._quat = None
            self._lowstate = msg

    # ---- telemetry access ---------------------------------------------
    def pose(self):
        with self._lock:
            return None if self._pose is None else list(self._pose)

    def velocity(self):
        with self._lock:
            return None if self._vel is None else list(self._vel)

    def quaternion(self):
        with self._lock:
            return None if self._quat is None else list(self._quat)

    def motor_state(self):
        """Return list of (q, dq) per motor from the last LowState."""
        with self._lock:
            if self._lowstate is None:
                return None
            return [(float(self._lowstate.motor_state[i].q),
                     float(self._lowstate.motor_state[i].dq)) for i in range(NUM_MOTORS)]

    # ---- command helpers ----------------------------------------------
    def send_motors(self, q, kp, kd, dq=None, tau=None):
        """Publish a LowCmd with per-motor PD targets.

        q, kp, kd: length-12 arrays. dq, tau: optional length-12 (default 0).
        """
        c = self.cmd
        for i in range(NUM_MOTORS):
            c.motor_cmd[i].q = float(q[i])
            c.motor_cmd[i].kp = float(kp[i])
            c.motor_cmd[i].kd = float(kd[i])
            c.motor_cmd[i].dq = float(dq[i]) if dq is not None else 0.0
            c.motor_cmd[i].tau = float(tau[i]) if tau is not None else 0.0
        c.crc = self.crc.Crc(c)
        self.pub.Write(c)

    def hold_stance(self, kp: float = 50.0, kd: float = 3.5):
        self.send_motors(STAND_UP, [kp] * NUM_MOTORS, [kd] * NUM_MOTORS)

    def relax(self):
        """Zero gains (motors free)."""
        self.send_motors(np.zeros(NUM_MOTORS), [0.0] * NUM_MOTORS, [0.0] * NUM_MOTORS)