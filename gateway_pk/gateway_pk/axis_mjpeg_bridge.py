#!/usr/bin/env python3
"""ROS 2 Humble CompressedImage -> HTTP MJPEG bridge for Coastal Patrol."""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FrameStore:
    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0
        self.received_at = 0.0
        self.frame_id = ""
        self.format = ""

    def update(self, message):
        frame = bytes(message.data)
        if not frame:
            return
        with self.condition:
            self.frame = frame
            self.sequence += 1
            self.received_at = time.time()
            self.frame_id = message.header.frame_id
            self.format = message.format
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return self.frame, self.sequence

    def wait_for_next(self, last_sequence, timeout=2.0):
        with self.condition:
            self.condition.wait_for(
                lambda: self.sequence != last_sequence,
                timeout=timeout,
            )
            return self.frame, self.sequence

    def health(self, topic):
        with self.condition:
            age = None if not self.received_at else time.time() - self.received_at
            return {
                "ok": self.frame is not None and age is not None and age < 3.0,
                "topic": topic,
                "frame_count": self.sequence,
                "frame_age_sec": None if age is None else round(age, 3),
                "frame_id": self.frame_id,
                "format": self.format,
            }


class CameraSubscriber(Node):
    def __init__(self, topic, store):
        super().__init__("coastal_axis_mjpeg_bridge")
        self.create_subscription(
            CompressedImage,
            topic,
            store.update,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Camera topic: {topic}")


def handler_factory(store, topic):
    class Handler(BaseHTTPRequestHandler):
        def common_headers(self):
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Access-Control-Allow-Origin", "*")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/health":
                payload = json.dumps(store.health(topic)).encode("utf-8")
                self.send_response(200)
                self.common_headers()
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return

            if path == "/snapshot":
                frame, _ = store.snapshot()
                if frame is None:
                    self.send_error(503, "Waiting for ROS camera frame")
                    return
                self.send_response(200)
                self.common_headers()
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                return

            if path not in ("/", "/stream"):
                self.send_error(404)
                return

            self.send_response(200)
            self.common_headers()
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=coastalframe",
            )
            self.end_headers()
            sequence = -1
            try:
                while rclpy.ok():
                    frame, next_sequence = store.wait_for_next(sequence)
                    if frame is None or next_sequence == sequence:
                        continue
                    sequence = next_sequence
                    self.wfile.write(b"--coastalframe\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def log_message(self, message_format, *args):
            return

    return Handler


def parse_args(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/axis/image_raw/compressed")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    # ros2 launch appends --ros-args; leave those arguments for rclpy.
    parsed, _ = parser.parse_known_args(args)
    return parsed


def main(args=None):
    args = parse_args(args)
    store = FrameStore()
    rclpy.init()
    node = CameraSubscriber(args.topic, store)
    server = ThreadingHTTPServer(
        (args.host, args.port),
        handler_factory(store, args.topic),
    )
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f"MJPEG stream: http://{args.host}:{args.port}/stream")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
