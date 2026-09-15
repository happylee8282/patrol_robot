#!/usr/bin/env python3
"""Configure the Jetson AGX Orin native CAN0 interface."""

import os
import subprocess
import sys
import time


INTERFACE = "can0"
BITRATE = "500000"
INTERFACE_WAIT_SEC = 30


def privileged_command(*command: str) -> None:
    """Run a command as root, using non-interactive sudo when necessary."""
    full_command = list(command)
    if os.geteuid() != 0:
        full_command = ["sudo", "-n", *full_command]

    print(f"[can_up] running: {' '.join(command)}", flush=True)
    subprocess.run(full_command, check=True)


def interface_exists() -> bool:
    result = subprocess.run(
        ["ip", "link", "show", INTERFACE],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def main() -> None:
    try:
        privileged_command("modprobe", "can")
        privileged_command("modprobe", "can_raw")
        privileged_command("modprobe", "mttcan")

        for _ in range(INTERFACE_WAIT_SEC):
            if interface_exists():
                break
            time.sleep(1.0)
        else:
            raise RuntimeError(f"{INTERFACE} was not found")

        # Jetson AGX Orin 40-pin CAN0 pinmux:
        # Pin 29 = CAN0_DIN/RX, Pin 31 = CAN0_DOUT/TX.
        privileged_command(
            "busybox", "devmem", "0x0c303018", "32", "0x0000C458")
        privileged_command(
            "busybox", "devmem", "0x0c303010", "32", "0x0000C400")

        # Ignore only the expected error produced when an already-down
        # interface is requested to go down again.
        down_command = ["ip", "link", "set", INTERFACE, "down"]
        if os.geteuid() != 0:
            down_command = ["sudo", "-n", *down_command]
        subprocess.run(down_command, check=False)

        privileged_command(
            "ip", "link", "set", INTERFACE,
            "type", "can", "bitrate", BITRATE,
        )
        privileged_command("ip", "link", "set", INTERFACE, "up")

        print(
            f"[can_up] {INTERFACE} is ready at {BITRATE} bit/s",
            flush=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError, RuntimeError) as error:
        print(f"[can_up] ERROR: {error}", file=sys.stderr, flush=True)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
