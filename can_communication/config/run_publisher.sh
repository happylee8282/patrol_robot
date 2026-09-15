#!/usr/bin/env bash
set -eo pipefail

BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "[battery-can] ERROR: /opt/ros/humble/setup.bash not found"
  exit 1
fi

export ROS_DOMAIN_ID=84

exec python3 "${BASE_DIR}/battery_status_publisher.py" \
  --ros-args \
  --params-file "${BASE_DIR}/config/battery_can.yaml"
