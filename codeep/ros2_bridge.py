"""ROS2 bridge node for the Go2 (DDS <-> ROS2).

Exposes the existing unitree_mujoco DDS interface as standard ROS2 topics and
services, so the Go2 can be driven from a plain ROS2 stack (cmd_vel + state +
stop). This realises the case's optional "ROS2 topic/service yapısının etkin
kullanılması" bonus.

DDS -> ROS2 (state out):
  rt/lowstate       -> /go2/joint_states (sensor_msgs/JointState)
                    -> /go2/imu          (sensor_msgs/Imu)
  rt/sportmodestate -> /go2/pose         (geometry_msgs/PoseStamped)
  rt/rangefinders   -> /go2/range_{forward,left,right} (sensor_msgs/Range)

ROS2 -> DDS (command in):
  /go2/cmd_vel (geometry_msgs/Twist) -> rt/cmd_vel (CmdVel vx,vy,wz)
  /go2/stop    (std_srvs/Trigger)    -> rt/cmd_vel (0,0,0)

Run (needs ROS2 + the venv's cyclonedds/unitree_sdk2py on PYTHONPATH;
use scripts/ros2_env.sh to set the env):
  python3 codeep/ros2_bridge.py
"""
from __future__ import annotations

import math
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState, Imu, Range
from std_srvs.srv import Trigger

# unitree DDS IDLs + codeep IDLs (venv site-packages via PYTHONPATH)
from unitree_sdk2py.core.channel import (  # noqa: E402
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import (  # noqa: E402
    LowState_,
    SportModeState_,
)
from codeep.robot.rangefinder_idl import RangefinderData  # noqa: E402
from codeep.robot.cmd_vel_idl import CmdVel  # noqa: E402

# unitree LowState motor order (FR/FL/RR/RL x hip/thigh/calf)
JOINT_NAMES = ["FR_hip", "FL_hip", "RR_hip", "RL_hip",
               "FR_thigh", "FL_thigh", "RR_thigh", "RL_thigh",
               "FR_calf", "FL_calf", "RR_calf", "RL_calf"]


class Go2Ros2Bridge(Node):
    def __init__(self, domain_id: int = 1, interface: str = "lo"):
        super().__init__("go2_ros2_bridge")
        self._lock = threading.Lock()
        self._imu_quat = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z (cached for pose orientation)

        # --- ROS2 publishers (state out) ---
        self.pub_pose = self.create_publisher(PoseStamped, "/go2/pose", 10)
        self.pub_js = self.create_publisher(JointState, "/go2/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/go2/imu", 10)
        self.pub_rf = {
            "forward": self.create_publisher(Range, "/go2/range_forward", 10),
            "left": self.create_publisher(Range, "/go2/range_left", 10),
            "right": self.create_publisher(Range, "/go2/range_right", 10),
        }

        # --- ROS2 subscribers / services (command in) ---
        self.create_subscription(Twist, "/go2/cmd_vel", self._on_cmd_vel, 10)
        self.create_service(Trigger, "/go2/stop", self._on_stop)

        # --- DDS side ---
        ChannelFactoryInitialize(domain_id, interface)
        self._cmd_pub = ChannelPublisher("rt/cmd_vel", CmdVel)
        self._cmd_pub.Init()
        low_sub = ChannelSubscriber("rt/lowstate", LowState_)
        low_sub.Init(self._on_lowstate, 10)
        high_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        high_sub.Init(self._on_highstate, 10)
        rf_sub = ChannelSubscriber("rt/rangefinders", RangefinderData)
        rf_sub.Init(self._on_rf, 10)

        self.get_logger().info("go2_ros2_bridge up: DDS <-> ROS2 "
                               "(/go2/pose, /go2/joint_states, /go2/imu, "
                               "/go2/range_*, /go2/cmd_vel, /go2/stop)")

    # --- DDS -> ROS2 callbacks (fire on DDS reader threads) ---
    def _on_lowstate(self, msg: LowState_):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [float(msg.motor_state[i].q) for i in range(12)]
        js.velocity = [float(msg.motor_state[i].dq) for i in range(12)]
        self.pub_js.publish(js)

        imu = Imu()
        imu.header.stamp = js.header.stamp
        imu.header.frame_id = "go2_imu"
        q = msg.imu_state.quaternion
        with self._lock:
            self._imu_quat = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        imu.orientation.w = float(q[0]); imu.orientation.x = float(q[1])
        imu.orientation.y = float(q[2]); imu.orientation.z = float(q[3])
        g = msg.imu_state.gyroscope
        imu.angular_velocity.x = float(g[0]); imu.angular_velocity.y = float(g[1]); imu.angular_velocity.z = float(g[2])
        a = msg.imu_state.accelerometer
        imu.linear_acceleration.x = float(a[0]); imu.linear_acceleration.y = float(a[1]); imu.linear_acceleration.z = float(a[2])
        self.pub_imu.publish(imu)

    def _on_highstate(self, msg: SportModeState_):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(msg.position[0])
        pose.pose.position.y = float(msg.position[1])
        pose.pose.position.z = float(msg.position[2])
        with self._lock:
            w, x, y, z = self._imu_quat
        pose.pose.orientation.w = w; pose.pose.orientation.x = x
        pose.pose.orientation.y = y; pose.pose.orientation.z = z
        self.pub_pose.publish(pose)

    def _on_rf(self, msg: RangefinderData):
        stamp = self.get_clock().now().to_msg()
        for key, val in (("forward", msg.forward), ("left", msg.left), ("right", msg.right)):
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = f"go2_rf_{key}"
            r.radiation_type = Range.LASER
            r.field_of_view = 0.05
            r.min_range = 0.0
            r.max_range = 30.0
            r.range = float(val) if 0.0 < val < 30.0 else float("inf")
            self.pub_rf[key].publish(r)

    # --- ROS2 -> DDS (command in) ---
    def _on_cmd_vel(self, msg: Twist):
        self._publish_cmd(float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def _on_stop(self, _req, resp):
        self._publish_cmd(0.0, 0.0, 0.0)
        resp.success = True
        resp.message = "stop commanded (cmd_vel=0)"
        return resp

    def _publish_cmd(self, vx: float, vy: float, wz: float):
        self._cmd_pub.Write(CmdVel(vx=vx, vy=vy, wz=wz))


def main():
    rclpy.init()
    node = Go2Ros2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()