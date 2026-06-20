"""Analytic leg IK/FK for the Unitree Go2 (geometry from unitree_robots/go2/go2.xml).

Coordinate convention: foot targets are expressed relative to the *hip joint*
(abduction joint), body frame: +x forward, +y left, +z up. Standing foot rel hip
is approximately (0, +/-0.0955, -0.358).

Joint conventions (verified against stand pose q=[0.006,0.609,-1.218 ...]):
  * q_hip   : abduction about +x; ~0 in stance.
  * q_thigh: FE about +y; 0 = thigh straight down; + => foot forward.
  * q_calf : knee; always negative (range -2.72..-0.84); q_calf = -(knee opening).

Motor index order (matches go2.xml actuator/sensor order):
  0 FR_hip 1 FR_thigh 2 FR_calf | 3 FL_hip 4 FL_thigh 5 FL_calf
  6 RR_hip 7 RR_thigh 8 RR_calf | 9 RL_hip10 RL_thigh11 RL_calf
"""
from __future__ import annotations

import math
import numpy as np

L1 = 0.213  # thigh length (hip FE joint -> knee)
L2 = 0.213  # calf length (knee -> foot)
D = 0.0955  # lateral offset hip joint -> thigh FE joint

# Hip joint position in body frame (x forward, y left) -- used for body-frame
# foot targets / steering, not for IK itself.
HIP_POS = {
    "FR": (+0.1934, -0.0465),
    "FL": (+0.1934, +0.0465),
    "RR": (-0.1934, -0.0465),
    "RL": (-0.1934, +0.0465),
}
SIDE = {"FR": -1, "RR": -1, "FL": +1, "RL": +1}  # +1 left, -1 right
LEG_ORDER = ["FR", "FL", "RR", "RL"]
# index of first motor for each leg in the 12-vector
LEG_MOTOR_OFFSET = {"FR": 0, "FL": 3, "RR": 6, "RL": 9}


def leg_ik(fx: float, fy: float, fz: float, side: int):
    """Solve one leg's joint angles for a foot target rel hip.

    Returns (q_hip, q_thigh, q_calf). fz should be negative (foot below hip).
    """
    # --- abduction: foot . y' = side*D, where y' = (0, cos t, sin t) ---
    A, B, C = fy, fz, side * D
    R = math.hypot(A, B)
    if R < abs(C) + 1e-6:
        R = abs(C) + 1e-6
    gamma = math.atan2(B, A)
    ac = math.acos(max(-1.0, min(1.0, C / R)))
    t1 = gamma - ac
    t2 = gamma + ac
    q_hip = t1 if abs(t1) <= abs(t2) else t2

    # in-plane "down" coordinate z' = foot . (0, -sin t, cos t)
    zp = -fy * math.sin(q_hip) + fz * math.cos(q_hip)
    # MuJoCo thigh rotates about +y: +q_thigh moves foot toward -x, so the
    # in-plane x that satisfies the 2-link geometry is -fx.
    x = -fx
    dist = math.hypot(x, zp)
    dist = max(abs(L1 - L2) + 1e-4, min(L1 + L2 - 1e-4, dist))

    cos_knee = (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    q_calf = -math.acos(cos_knee)  # knee bends back (negative)

    q_thigh = math.atan2(x, -zp) - math.atan2(L2 * math.sin(q_calf),
                                              L1 + L2 * math.cos(q_calf))
    return q_hip, q_thigh, q_calf


def stance_foot_rel_hip(side: int, height: float = 0.358):
    """Default standing foot target rel hip (straight down)."""
    return (0.0, side * D, -height)


def stance_pose(height: float = 0.28):
    """Joint-angle vector (12,) for a straight-down crouched stance at the given
    foot-below-hip height. Lower height => more crouched => lower CoM (more
    stable for walking)."""
    q = np.zeros(12)
    for leg in LEG_ORDER:
        qh, qt, qc = leg_ik(0.0, SIDE[leg] * D, -height, SIDE[leg])
        off = LEG_MOTOR_OFFSET[leg]
        q[off + 0] = qh
        q[off + 1] = qt
        q[off + 2] = qc
    return q


def legs_to_motor_targets(foot_targets):
    """foot_targets: dict leg -> (fx, fy, fz) rel hip. Returns 12-vector of
    (q) joint angles in motor order."""
    q = np.zeros(12)
    for leg in LEG_ORDER:
        fx, fy, fz = foot_targets[leg]
        qh, qt, qc = leg_ik(fx, fy, fz, SIDE[leg])
        off = LEG_MOTOR_OFFSET[leg]
        q[off + 0] = qh
        q[off + 1] = qt
        q[off + 2] = qc
    return q