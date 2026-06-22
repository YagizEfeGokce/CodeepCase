"""ROS2 demo client — drives the Go2 from standard ROS2 topics/services.

Publishes /go2/cmd_vel (drive forward at 0.3 m/s) for 6 s, then calls the
/go2/stop service, while subscribing to /go2/pose. Proves the ROS2
topic + service path end-to-end (cmd_vel -> motion, stop service -> halt,
pose -> telemetry).

Run with system python3 after sourcing scripts/ros2_env.sh:
  source scripts/ros2_env.sh
  python3 scripts/ros2_demo_client.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger


class DemoClient(Node):
    def __init__(self):
        super().__init__("go2_demo_client")
        self.cmd_pub = self.create_publisher(Twist, "/go2/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/go2/pose", self._on_pose, 10)
        self.cli = self.create_client(Trigger, "/go2/stop")
        self._pose = None
        self._stopped = False
        self._t0 = self.get_clock().now()
        self._last_log = -1.0
        self.timer = self.create_timer(0.1, self._tick)

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg.pose.position

    def _tick(self):
        elapsed = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        if elapsed < 6.0:
            t = Twist()
            t.linear.x = 0.3
            self.cmd_pub.publish(t)
        elif not self._stopped:
            # call /go2/stop once
            if self.cli.service_is_ready():
                fut = self.cli.call_async(Trigger.Request())
                self.get_logger().info("-> calling /go2/stop service")
                self._stop_fut = fut
            self._stopped = True
        if int(elapsed) != int(self._last_log):
            p = self._pose
            ps = f"({p.x:+.2f},{p.y:+.2f},{p.z:.2f})" if p else "<no pose yet>"
            self.get_logger().info(f"t={elapsed:4.1f}s pose={ps}")
            self._last_log = elapsed
        if elapsed > 10.0:
            raise SystemExit  # demo done


def main():
    rclpy.init()
    node = DemoClient()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()