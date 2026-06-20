"""Navigation controller (Layer 3): closed-loop steering of the Go2 to a
2D target point, on top of the RL locomotion layer.

We command the RL runner with body velocity (vx forward, vy lateral, wz yaw).
A P-controller drives heading + distance + lateral offset to zero so the
robot reaches the target. This is our own code; the RL policy is treated as a
black-box "sport mode" that tracks (vx, vy, wz).

Yaw is extracted from the base IMU quaternion. Lateral correction is kept
small so it does not fight the heading loop.
"""
from __future__ import annotations

import math
from typing import Any


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NavController:
    def __init__(self, runner,
                 kp_lin: float = 0.6, kp_yaw: float = 1.0, kp_lat: float = 0.5,
                 kp_lat_yaw: float = 0.6, yaw_bias: float = 0.0,
                 min_align: float = 0.35,
                 max_vx: float = 0.30, max_vy: float = 0.0, max_wz: float = 0.8,
                 goal_tol: float = 0.25, slow_yaw_err: float = 0.6,
                 goal_vx_scale: float = 1.0):
        self.runner = runner
        self.kp_lin = kp_lin
        self.kp_yaw = kp_yaw
        self.kp_lat = kp_lat
        self.kp_lat_yaw = kp_lat_yaw  # feedback: cancel lateral crab via extra yaw
        self.yaw_bias = yaw_bias      # feedforward heading offset into the drift (rad)
        self.min_align = min_align    # keep min forward speed so the policy keeps walking while turning
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_wz = max_wz
        self.goal_tol = goal_tol
        self.slow_yaw_err = slow_yaw_err
        self.goal_vx_scale = goal_vx_scale
        self.target = None
        self.reached = False

    def set_target(self, x: float, y: float):
        self.target = (x, y)
        self.reached = False

    def _yaw(self):
        q = self.runner.quaternion()
        return None if q is None else yaw_from_quat(q)

    def step(self) -> dict[str, Any] | None:
        p = self.runner.pose()
        yaw = self._yaw()
        if p is None or yaw is None or self.target is None:
            return None
        tx, ty = self.target
        dx = tx - p[0]
        dy = ty - p[1]
        dist = math.hypot(dx, dy)
        des_heading = math.atan2(dy, dx) + self.yaw_bias
        yaw_err = wrap_angle(des_heading - yaw)

        # yaw rate: null heading error + extra yaw to cancel lateral crab
        # (the all_gait policy ignores vy, so we steer into the drift with yaw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        lat_body = -sy * dx + cy * dy
        wz = max(-self.max_wz, min(self.max_wz, self.kp_yaw * yaw_err + self.kp_lat_yaw * lat_body))

        # forward speed, scaled down when not yet aligned with the target
        align = max(self.min_align, 1.0 - abs(yaw_err) / self.slow_yaw_err)
        vx = max(0.0, min(self.max_vx, self.kp_lin * dist)) * align * self.goal_vx_scale

        # lateral velocity command is ineffective on this policy -> keep 0
        vy = 0.0

        if dist <= self.goal_tol:
            vx = vy = wz = 0.0
            self.reached = True

        self.runner.set_command(vx, vy, wz)
        return dict(dist=dist, yaw_err=yaw_err, vx=vx, vy=vy, wz=wz,
                    pose=p, reached=self.reached)