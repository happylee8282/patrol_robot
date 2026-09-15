#!/usr/bin/env bash
set -eo pipefail

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: ROS2 Humble setup not found"
  exit 1
fi

echo "Topic:"
ros2 topic info /battery_status

echo
echo "One message:"
ros2 topic echo --once /battery_status
