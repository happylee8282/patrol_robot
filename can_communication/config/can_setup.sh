#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-500000}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "[battery-can] ERROR: run with sudo"
  exit 1
fi

modprobe can
modprobe can_raw
modprobe mttcan

for _ in $(seq 1 30); do
  if ip link show "${IFACE}" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "[battery-can] ERROR: ${IFACE} not found"
  exit 1
fi

# Jetson AGX Orin CAN0 pinmux
# Pin 29 = CAN0_DIN / RX
# Pin 31 = CAN0_DOUT / TX
busybox devmem 0x0c303018 32 0x0000C458
busybox devmem 0x0c303010 32 0x0000C400

ip link set "${IFACE}" down 2>/dev/null || true
ip link set "${IFACE}" type can bitrate "${BITRATE}"
ip link set "${IFACE}" up

echo "[battery-can] ready"
ip -details -statistics link show "${IFACE}"
