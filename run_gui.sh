#!/bin/bash
# Restart the robot_control_gui_jazzy GUI cleanly.
#
# Why a wrapper? Two reasons:
#   1. After editing .cpp/.h you MUST rebuild before restarting, otherwise
#      the GUI runs stale code (this happened during testing).
#   2. The mentorpi bridge dies if a stale robot_control_gui_jazzy_node is
#      still alive — kill leftover instances first.
#
# Usage:
#   ./run_gui.sh           # normal start
#   ./run_gui.sh --no-build # skip the colcon build (faster, use when code is fresh)

WS=/home/robot/ros2_gui/robot_control_gui_ros2
PKG=robot_control_gui_jazzy
NODE=${PKG}_node

if [ ! -d "$WS/install/$PKG" ]; then
  echo "[run_gui] ERROR: $WS/install/$PKG not found. Run colcon build first." >&2
  exit 1
fi

if [[ "$1" != "--no-build" ]]; then
  echo "[run_gui] colcon build $PKG ..."
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select "$PKG" --cmake-clean-first 2>&1 | tail -5
fi

echo "[run_gui] killing any leftover $NODE ..."
pkill -f "$NODE" 2>/dev/null
sleep 2

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
# Hard-pin the domain. mentorpi runs on 0; override your shell's ROS_DOMAIN_ID
# (e.g. 232) by passing ROS_DOMAIN_ID=0 explicitly. Pass a different value
# if your robot is on a different id.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_OVERRIDE:-0}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}

echo "[run_gui] starting $NODE (DOMAIN_ID=$ROS_DOMAIN_ID, scope=$ROS_AUTOMATIC_DISCOVERY_RANGE) ..."
exec ros2 run "$PKG" "$NODE"