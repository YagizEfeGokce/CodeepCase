"""RL locomotion runner for the diasAiMaster/unitree-go2-velocity-flat ONNX policy.

Loads `policy.onnx` (mjlab PPO, flat terrain) via onnxruntime and runs it over
the unitree_mujoco DDS bridge. The ONNX graph folds empirical obs
normalization in (first nodes: Sub(mean) -> Div(std)), so we feed RAW 45-dim
obs and get 12 raw joint-position actions.

Policy is vy-supporting (omnidirectional velocity tracking), so the
NavController can do real closed-loop lateral correction -> straight walk
without the yaw_bias feedforward hack the all_gait policy needs.

Same high-level interface as rl_runner.RLRunner:
    r = RLRunnerOnnx(); r.start(); r.set_command(vx, vy, wz); r.pose(); r.stop()

Constants come from the model's params/deploy.yaml (single source of truth):
  obs order (45): base_ang_vel(3), projected_gravity(3), velocity_commands(3),
                  joint_pos_rel(12), joint_vel_rel(12), last_action(12)
  action: target_q[policy_j] = default_joint_pos[j] + 0.5 * action[j]
  joint_ids_map = [3,4,5,0,1,2,9,10,11,6,7,8]  (motor_i -> policy_index; involution)
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

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

DEFAULT_MODEL = str(Path(__file__).resolve().parents[2]
                    / "external" / "diasAiMaster_go2_velocity_flat" / "policy.onnx")

# --- deploy.yaml constants (policy order) ---
DEFAULT_POS = np.array([-0.1, 0.9, -1.8, 0.1, 0.9, -1.8,
                        -0.1, 0.9, -1.8, 0.1, 0.9, -1.8], dtype=np.float32)
# motor_i -> policy_index; involution (its own inverse), so policy_j -> motor_j too.
MAPPING = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8], dtype=np.int32)
ACTION_SCALE = 0.5
STIFFNESS_POLICY = np.array([20, 20, 40, 20, 20, 40, 20, 20, 40, 20, 20, 40], dtype=np.float32)
DAMPING_POLICY = np.array([1, 1, 2, 1, 1, 2, 1, 1, 2, 1, 1, 2], dtype=np.float32)
# per-motor (unitree LowState motor order) gains
KPS = STIFFNESS_POLICY[MAPPING]
KDS = DAMPING_POLICY[MAPPING]
# per-motor default pos = DEFAULT_POS[policy_index_for_motor_i] = DEFAULT_POS[MAPPING[i]]
DEFAULT_POS_MOTOR = DEFAULT_POS[MAPPING]

SIM_DT = 0.005
DECIMATION = 4
NUM_OBS = 45
GRAVITY = np.array([0.0, 0.0, -1.0], dtype=np.float32)


def _quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix (world<-body) from w,x,y,z quaternion."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=np.float32)


class RLRunnerOnnx:
    def __init__(self, policy_path: str | None = None, domain_id: int = 1,
                 interface: str = "lo", stand_time: float = 3.0, sim_dt: float = SIM_DT,
                 ros2_cmd: bool = False):
        self.policy_path = policy_path or DEFAULT_MODEL
        self.domain_id = domain_id
        self.interface = interface
        self.stand_time = stand_time
        self.sim_dt = sim_dt
        self.ros2_cmd = ros2_cmd  # if True, read (vx,vy,wz) from DDS rt/cmd_vel (ROS2 bridge)
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._command = np.zeros(3, dtype=np.float32)  # [vx, vy, wz]
        self._pose = None
        self._vel = None
        self._quat = None
        self._lowstate = None

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

    # ---- DDS handlers ----
    def _on_highstate(self, msg: SportModeState_):
        with self._lock:
            self._pose = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
            self._vel = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]

    def _on_lowstate(self, msg: LowState_):
        with self._lock:
            try:
                q = msg.imu_state.quaternion
                self._quat = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
            except Exception:
                self._quat = None
            self._lowstate = msg

    def _on_cmd_vel(self, msg):
        # ROS2 bridge: rt/cmd_vel (CmdVel) -> self._command (vx, vy, wz)
        with self._lock:
            self._command[:] = [float(msg.vx), float(msg.vy), float(msg.wz)]

    # ---- lifecycle ----
    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _build_obs(self, ls: LowState_, last_action: np.ndarray) -> np.ndarray:
        # body angular velocity
        gyro = ls.imu_state.gyroscope
        ang_vel = np.array([gyro[0], gyro[1], gyro[2]], dtype=np.float32)
        # projected gravity = R^T @ [0,0,-1]
        q = ls.imu_state.quaternion
        R = _quat_to_rot(q[0], q[1], q[2], q[3])
        proj_grav = R.T @ GRAVITY
        # joint states: read motor order -> reorder to policy order via MAPPING (involution)
        motor_q = np.fromiter((ls.motor_state[i].q for i in range(12)), dtype=np.float32)
        motor_dq = np.fromiter((ls.motor_state[i].dq for i in range(12)), dtype=np.float32)
        dof_pos = motor_q[MAPPING] - DEFAULT_POS      # policy order
        dof_vel = motor_dq[MAPPING]                    # policy order
        obs = np.empty(NUM_OBS, dtype=np.float32)
        obs[0:3] = ang_vel
        obs[3:6] = proj_grav
        with self._lock:
            obs[6:9] = self._command
        obs[9:21] = dof_pos
        obs[21:33] = dof_vel
        obs[33:45] = last_action
        return obs

    def _loop(self):
        ChannelFactoryInitialize(self.domain_id, self.interface)
        high_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        high_sub.Init(self._on_highstate, 10)
        low_sub = ChannelSubscriber("rt/lowstate", LowState_)
        low_sub.Init(self._on_lowstate, 10)
        if self.ros2_cmd:
            # ROS2 bridge mode: drive the policy from DDS rt/cmd_vel (CmdVel)
            # published by codeep/ros2_bridge.py from ROS2 /go2/cmd_vel.
            from codeep.robot.cmd_vel_idl import CmdVel  # lazy (keeps non-ROS2 path decoupled)
            cmd_sub = ChannelSubscriber("rt/cmd_vel", CmdVel)
            cmd_sub.Init(self._on_cmd_vel, 10)
        pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        pub.Init()
        crc = CRC()
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0

        sess = ort.InferenceSession(self.policy_path, providers=["CPUExecutionProvider"])
        in_name = sess.get_inputs()[0].name

        # wait for first lowstate
        t0 = time.time()
        while self._lowstate is None and time.time() - t0 < 8.0 and not self._stop.is_set():
            time.sleep(0.02)
        if self._lowstate is None:
            print("[onnx] no lowstate; aborting loop")
            return

        def write_cmd(target_q_motor: np.ndarray, kp: np.ndarray, kd: np.ndarray):
            for i in range(12):
                cmd.motor_cmd[i].q = float(target_q_motor[i])
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = float(kp[i])
                cmd.motor_cmd[i].kd = float(kd[i])
                cmd.motor_cmd[i].tau = 0.0
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)

        # --- stand-up: smooth ramp current pose -> default, model deploy gains ---
        # (the model's way: ramp start_q -> default_joint_pos over stand_time with the
        # deploy.yaml per-joint stiffness/damping (KPS/KDS), slew-limited for a safe,
        # jerk-free wake-up — instead of abruptly holding the default pose).
        ls0 = self._lowstate
        start_q = np.array([ls0.motor_state[i].q for i in range(12)], dtype=np.float32)
        stand_steps = max(1, int(self.stand_time / self.sim_dt))
        STAND_MAX_STEP = 0.05  # rad per sim step — slew cap for smoothness
        prev_q = start_q.copy()
        for k in range(stand_steps):
            if self._stop.is_set():
                return
            alpha = (k + 1) / stand_steps
            desired = (1.0 - alpha) * start_q + alpha * DEFAULT_POS_MOTOR
            target = prev_q + np.clip(desired - prev_q, -STAND_MAX_STEP, STAND_MAX_STEP)
            write_cmd(target, KPS, KDS)
            prev_q = target
            time.sleep(self.sim_dt)

        # --- policy loop ---
        last_action = np.zeros(12, dtype=np.float32)
        # initial motor targets = default pose (updated each policy step)
        target_motor = DEFAULT_POS_MOTOR.copy()
        loop_c = 0
        while not self._stop.is_set():
            step_start = time.perf_counter()
            if loop_c % DECIMATION == 0:
                ls = self._lowstate
                if ls is not None:
                    obs = self._build_obs(ls, last_action)
                    action = sess.run(None, {in_name: obs.reshape(1, -1).astype(np.float32)})[0][0]
                    last_action = action.astype(np.float32)
                # policy target in policy order -> motor order
                target_policy = DEFAULT_POS + ACTION_SCALE * last_action
                target_motor = target_policy[MAPPING]
            write_cmd(target_motor, KPS, KDS)
            remaining = self.sim_dt - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)
            loop_c += 1