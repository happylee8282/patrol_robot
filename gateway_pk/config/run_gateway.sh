#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=84
export PYTHONUNBUFFERED=1

# Run this script after sourcing the workspace that contains gateway_pk.
exec ros2 launch gateway_pk bringup_gateway.launch.py
