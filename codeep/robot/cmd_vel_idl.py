"""DDS IDL type for high-level velocity commands (ROS2 bridge cmd_vel channel).

The ROS2 bridge subscribes to ROS2 /go2/cmd_vel (geometry_msgs/Twist) and
republishes it on DDS topic "rt/cmd_vel" as CmdVel(vx, vy, wz). RLRunnerOnnx
in ros2_cmd mode subscribes to rt/cmd_vel and drives the policy from it --
so the Go2 can be commanded from a standard ROS2 cmd_vel topic.

No `from __future__ import annotations` here (cyclonedds reads
cls.__annotations__ raw; stringized 'float' is unresolvable -- see
rangefinder_idl.py note).
"""
from dataclasses import dataclass
from cyclonedds.idl import IdlStruct


@dataclass
class CmdVel(IdlStruct, typename="CmdVel"):
    vx: float   # forward velocity (m/s)
    vy: float   # lateral velocity (m/s)
    wz: float   # yaw rate (rad/s)