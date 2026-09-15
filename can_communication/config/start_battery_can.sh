#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/2] Setting up AGX Orin CAN0..."
sudo "${BASE_DIR}/can_setup.sh"

echo
echo "[2/2] Starting ROS2 /battery_status publisher..."
exec "${BASE_DIR}/run_publisher.sh"
