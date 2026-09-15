#!/usr/bin/env python3
import json
import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_MASK = 0x1FFFFFFF


def decode_mode(value: int) -> str:
    mapping = {
        0x00: "Standby",
        0x01: "Charging",
        0x02: "Discharging",
    }
    return mapping.get(value, f"Unknown(0x{value:02X})")


def decode_fault(value: int) -> str:
    if value == 0x00:
        return "Normal"
    return f"Fault(0x{value:02X})"


class BatteryCanPublisher(Node):
    def __init__(self):
        super().__init__("battery_can_publisher")

        self.declare_parameter("interface", "can0")
        self.declare_parameter("can_id", 0x100)
        self.declare_parameter("topic", "/battery_status")
        self.declare_parameter("poll_period_sec", 0.01)
        self.declare_parameter("remaining_time_endian", "little")

        self.interface = str(self.get_parameter("interface").value)
        self.target_id = int(self.get_parameter("can_id").value)
        self.topic_name = str(self.get_parameter("topic").value)
        self.poll_period = float(self.get_parameter("poll_period_sec").value)
        self.remaining_time_endian = str(
            self.get_parameter("remaining_time_endian").value
        ).lower()

        self.publisher = self.create_publisher(String, self.topic_name, 10)

        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.interface,))
        self.sock.setblocking(False)

        self.timer = self.create_timer(self.poll_period, self.poll_can)

        self.get_logger().info(
            f"Listening on {self.interface}, CAN ID=0x{self.target_id:X}, "
            f"publishing ONE topic: {self.topic_name}"
        )

    def parse_frame(self, data: bytes):
        if len(data) < 8:
            return None

        soc = int(data[0])
        soh = int(data[1])
        mode = decode_mode(int(data[2]))
        fault = decode_fault(int(data[3]))

        if self.remaining_time_endian == "big":
            remaining_time_min = (int(data[4]) << 8) | int(data[5])
        else:
            remaining_time_min = (int(data[5]) << 8) | int(data[4])

        # Supervisor requested the five BMS values in ONE ROS2 topic.
        return {
            "mode": mode,
            "fault": fault,
            "soc": soc,
            "soh": soh,
            "remaining_time_min": remaining_time_min,
        }

    def poll_can(self):
        while True:
            try:
                frame = self.sock.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                return

            if len(frame) != CAN_FRAME_SIZE:
                continue

            raw_id, dlc, raw_data = struct.unpack(CAN_FRAME_FMT, frame)
            arbitration_id = raw_id & CAN_EFF_MASK

            if arbitration_id != self.target_id or dlc < 8:
                continue

            payload = self.parse_frame(raw_data[:8])
            if payload is None:
                continue

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryCanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
