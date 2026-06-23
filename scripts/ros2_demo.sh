#!/usr/bin/env bash
# ros2_demo.sh — end-to-end ROS2 wrapper demo.
# Starts the sim (clean scene), the ONNX runner in ros2_cmd mode, the
# DDS<->ROS2 bridge, then a ROS2 demo client that drives the Go2 forward via
# /go2/cmd_vel and halts it via the /go2/stop service, logging /go2/pose.
# Tears everything down at the end.
#
#   bash scripts/ros2_demo.sh
set -u
cd "$(dirname "$0")/.."
ROOT="$PWD"
PYVENV="$ROOT/.venv/bin/python"

source scripts/ros2_env.sh >/dev/null

PIDS=()
cleanup() {
	for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null; done
	for p in $(ps -eo pid,args | grep -E "sim_headless\.py|ros2_bridge\.py|ros2_run\.py" | grep -v "grep\|bash -c" | awk '{print $1}'); do
		kill -9 "$p" 2>/dev/null
	done
}
trap cleanup EXIT

echo "=== 1) sim (clean scene, headless) ==="
bash scripts/use_scene.sh clean >/dev/null
(SDL_VIDEODRIVER=dummy "$PYVENV" scripts/sim_headless.py --duration 60) >/tmp/ros2_sim.log 2>&1 &
PIDS+=("$!")
sleep 5
ps -eo pid,args | grep -q "sim_headless\.py" && echo "[demo] sim up" || {
	echo "[demo] sim failed"
	tail -15 /tmp/ros2_sim.log
	exit 1
}

echo "=== 2) ONNX runner (ros2_cmd mode) ==="
("$PYVENV" scripts/ros2_run.py) >/tmp/ros2_runner.log 2>&1 &
PIDS+=("$!")
sleep 5
grep -q "stood up" /tmp/ros2_runner.log && echo "[demo] runner up + stood up" || {
	echo "[demo] runner log:"
	tail -15 /tmp/ros2_runner.log
}

echo "=== 3) ROS2 bridge (DDS <-> ROS2) ==="
(python3 codeep/ros2_bridge.py) >/tmp/ros2_bridge.log 2>&1 &
PIDS+=("$!")
sleep 4
grep -q "go2_ros2_bridge up" /tmp/ros2_bridge.log && echo "[demo] bridge up" || {
	echo "[demo] bridge log:"
	tail -15 /tmp/ros2_bridge.log
}

echo "=== 4) ROS2 topics/services now available ==="
ros2 topic list 2>/dev/null | grep '^/go2/' | sed 's/^/  topic: /'
ros2 service list 2>/dev/null | grep '^/go2/' | sed 's/^/  service: /'

echo "=== 5) demo client: drive forward (/go2/cmd_vel) then /go2/stop ==="
python3 scripts/ros2_demo_client.py 2>&1 | grep -E "demo_client|pose=|stop" | head -20
echo "=== demo done ==="
