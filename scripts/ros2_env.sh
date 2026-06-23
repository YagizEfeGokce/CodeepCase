#!/usr/bin/env bash
# ros2_env.sh — env for the ROS2 bridge + ros2_cmd runner.
# Source it before running codeep/ros2_bridge.py or scripts/ros2_demo_client.py
# (they use system python3 + rclpy from ROS2 Jazzy + the venv's cyclonedds /
#  unitree_sdk2py via PYTHONPATH).
#
#   source scripts/ros2_env.sh
#   python3 codeep/ros2_bridge.py        # the DDS<->ROS2 bridge node
#
# The runner (scripts/ros2_run.py) uses the venv python and only needs
# CYCLONEDDS_HOME / LD_LIBRARY_PATH (set here too), not ROS2.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$PWD"

# ROS2 Jazzy (rclpy + geometry_msgs/sensor_msgs/std_srvs)
if [ -f /opt/ros/jazzy/setup.bash ]; then
	# ROS2 setup.bash references unset vars; temporarily disable nounset.
	_ros2_old_opts="${-:-}"
	set +u
	# shellcheck disable=SC1091
	source /opt/ros/jazzy/setup.bash
	case "$_ros2_old_opts" in *u*) set -u ;; esac
else
	echo "[ros2_env] /opt/ros/jazzy/setup.bash yok — ROS2 Jazzy kurulu değil." >&2
	exit 1
fi

# CycloneDDS C lib (venv's cyclonedds Python bindings need it)
export CYCLONEDDS_HOME="$ROOT/external/cyclonedds/install"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"

# Make the venv's cyclonedds + unitree_sdk2py + codeep importable from system python3
export PYTHONPATH="$ROOT/.venv/lib/python3.12/site-packages:$ROOT/external/unitree_sdk2_python:$ROOT:${PYTHONPATH:-}"

echo "[ros2_env] ROS_DISTRO=$ROS_DISTRO  CYCLONEDDS_HOME set  PYTHONPATH+=venv,codeep"
