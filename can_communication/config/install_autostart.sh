#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/unicon/battery_can_ros2"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo ./install_autostart.sh"
  exit 1
fi

chmod +x \
  "${PROJECT_DIR}/can_setup.sh" \
  "${PROJECT_DIR}/battery_status_publisher.py" \
  "${PROJECT_DIR}/run_publisher.sh" \
  "${PROJECT_DIR}/start_battery_can.sh" \
  "${PROJECT_DIR}/verify_battery_topic.sh" \
  "${PROJECT_DIR}/tools/can_reset.sh"

cp "${PROJECT_DIR}/systemd/battery-can-setup.service" /etc/systemd/system/
cp "${PROJECT_DIR}/systemd/battery-status-publisher.service" /etc/systemd/system/

systemctl daemon-reload
systemctl enable battery-can-setup.service
systemctl enable battery-status-publisher.service

echo "Installed. Start with:"
echo "  sudo systemctl start battery-can-setup.service"
echo "  sudo systemctl start battery-status-publisher.service"
