"""Open-loop trot gait generator for the Go2.

Diagonal pairs in phase (FR+RL, FL+RR). Each leg's foot follows a stance
(push back -> body forward) / swing (lift + return) trajectory in the body
frame, then analytic IK maps foot targets to 12 joint angles.

Body velocity mapping: foot sweeps stride S during stance, so
    v_body = S / t_stance = S / (duty * T)
and we set S = vx * duty * T (clamped) to track a commanded forward speed vx.
Steering (vy, wz) hooks are present but tuned in Gate D; Gate C uses vx only.
"""
from __future__ import annotations

import math

from .kinematics import (D, HIP_POS, LEG_ORDER, SIDE, legs_to_motor_targets)

STAND_HEIGHT = 0.28  # foot below hip (crouched stance; low CoM for stable walking)

# Crawl gait: one foot swings at a time, 3 always planted (quasi-static,
# body stays level). Swing windows are non-overlapping and CoM stays inside
# the support triangle for every swing config (verified geometrically).
CRAWL_PHASE_OFF = {"FR": 0.0, "RR": 0.25, "FL": 0.5, "RL": 0.75}
# Trot gait: diagonal pairs together.
TROT_PHASE_OFF = {"FR": 0.0, "FL": 0.5, "RR": 0.5, "RL": 0.0}


class TrotGait:
    def __init__(self, T: float = 0.5, duty: float = 0.5,
                 height: float = STAND_HEIGHT, swing_h: float = 0.06,
                 max_stride: float = 0.12, phase_off=None, press: float = 0.02):
        self.T = T
        self.duty = duty
        self.H = height
        self.swing_h = swing_h
        self.max_stride = max_stride
        self.press = press  # stance foot presses into ground (extra z) for grip
        # default: trot (diagonal pairs). Pass CRAWL_PHASE_OFF for a crawl.
        self.phase_off = dict(phase_off) if phase_off is not None else dict(TROT_PHASE_OFF)

    def foot_target(self, leg: str, t: float, vx: float, vy: float = 0.0, wz: float = 0.0):
        ph = (t / self.T + self.phase_off[leg]) % 1.0
        S = max(-self.max_stride, min(self.max_stride, vx * self.duty * self.T))
        if ph < self.duty:
            p = ph / self.duty
            fx = S * (-0.5 + p)   # stance: foot sweeps back->front (rel body)
            fz = -(self.H + self.press)   # press foot into ground for grip
        else:
            p = (ph - self.duty) / (1.0 - self.duty)
            fx = S * (0.5 - p)    # swing: return front->back through the air
            fz = -self.H + self.swing_h * math.sin(math.pi * p)
        # steering hooks (tuned in Gate D): lateral shift for vy, yaw via hip_x
        hx, _hy = HIP_POS[leg]
        fy = SIDE[leg] * D + vy * 0.5
        fx += -wz * hx  # front/rear opposite fore-aft => yaw (sign tuned later)
        return fx, fy, fz

    def joint_targets(self, t: float, vx: float, vy: float = 0.0, wz: float = 0.0):
        targets = {leg: self.foot_target(leg, t, vx, vy, wz) for leg in LEG_ORDER}
        return legs_to_motor_targets(targets)