"""Run RLRunnerOnnx in ros2_cmd mode — the Go2 stands up, then is driven from
ROS2 /go2/cmd_vel (geometry_msgs/Twist) via the DDS bridge (rt/cmd_vel).

Run with the VENV python (has onnxruntime + cyclonedds + unitree_sdk2py).
Needs CYCLONEDDS_HOME + LD_LIBRARY_PATH (source scripts/ros2_env.sh first):

  source scripts/ros2_env.sh
  .venv/bin/python scripts/ros2_run.py

Then from a ROS2 shell: `ros2 topic pub /go2/cmd_vel geometry_msgs/Twist ...`
or run scripts/ros2_demo_client.py.
"""
import sys
import time

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner_onnx import RLRunnerOnnx


def main():
    r = RLRunnerOnnx(ros2_cmd=True, stand_time=3.0)
    r.start()
    t0 = time.time()
    while r.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if r.pose() is None:
        print("[ros2_run] no telemetry; is the sim running?"); r.stop(); sys.exit(2)
    print(f"[ros2_run] stood up at {r.pose()}; command via ROS2 /go2/cmd_vel "
          f"(Twist: linear.x=vx, linear.y=vy, angular.z=wz) or /go2/stop", flush=True)
    last = 0.0
    try:
        while True:
            p = r.pose()
            now = time.time() - t0
            if p and now - last >= 1.0:
                print(f"[ros2_run] t={now:4.1f}s pose=({p[0]:+.2f},{p[1]:+.2f},{p[2]:.2f})", flush=True)
                last = now
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    r.set_command(0.0, 0.0, 0.0)
    time.sleep(0.3)
    r.stop()


if __name__ == "__main__":
    main()