import asyncio
import json
import queue
import threading
import time
from typing import Any, Dict

from aiohttp import WSMsgType, web
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from axis_camera_msgs.msg import Axis


class AxisWebBridge(Node):
    def __init__(self) -> None:
        super().__init__("axis_web_bridge")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 8766)
        self.declare_parameter("command_timeout", 0.5)

        self.bind_host = self.get_parameter("bind_host").value
        self.port = int(self.get_parameter("port").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)

        self.latest_image = b""
        self.image_sequence = 0
        self.latest_state: Dict[str, Any] = {
            "type": "ptz_state",
            "pan": 0.0,
            "tilt": 0.0,
            "zoom": 0.0,
            "connected": False,
        }
        self.state_sequence = 0
        self.command_queue: queue.Queue = queue.Queue()
        self.last_command_time = 0.0
        self.ptz_moving = False
        self.started_at = time.monotonic()

        self.image_subscriber = self.create_subscription(
            CompressedImage,
            "/axis/image_raw/compressed",
            self.on_image,
            10,
        )
        self.state_subscriber = self.create_subscription(
            Axis,
            "/axis/state",
            self.on_state,
            10,
        )
        self.command_publisher = self.create_publisher(
            Axis,
            "/axis/axis_cmd",
            10,
        )
        self.command_timer = self.create_timer(0.02, self.process_commands)
        self.watchdog_timer = self.create_timer(0.1, self.check_watchdog)

        self.web_loop = asyncio.new_event_loop()
        self.web_thread = threading.Thread(target=self.run_web_server, daemon=True)
        self.web_thread.start()

        self.get_logger().info(
            f"AXIS web bridge: http://{self.bind_host}:{self.port} "
            f"(stream=/stream.mjpg, websocket=/ws)"
        )

    def on_image(self, message: CompressedImage) -> None:
        if message.data:
            self.latest_image = bytes(message.data)
            self.image_sequence += 1

    def on_state(self, message: Axis) -> None:
        self.latest_state = {
            "type": "ptz_state",
            "pan": float(message.pan),
            "tilt": float(message.tilt),
            "zoom": float(message.zoom),
            "focus": float(message.focus),
            "brightness": float(message.brightness),
            "iris": float(message.iris),
            "autofocus": bool(message.autofocus),
            "connected": True,
            "timestamp": time.time(),
        }
        self.state_sequence += 1

    def process_commands(self) -> None:
        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return

            pan = max(-100.0, min(100.0, float(command.get("pan", 0.0))))
            tilt = max(-100.0, min(100.0, float(command.get("tilt", 0.0))))
            active = bool(command.get("active", False))
            if not active:
                pan = 0.0
                tilt = 0.0

            message = Axis()
            message.stamp = self.get_clock().now().to_msg()
            message.pan = pan
            message.tilt = tilt
            message.zoom = float(command.get("zoom", 0.0))
            self.command_publisher.publish(message)

            self.last_command_time = time.monotonic()
            self.ptz_moving = abs(pan) > 0.0 or abs(tilt) > 0.0
            self.get_logger().info(f"Glass PTZ command: pan={pan:.1f}, tilt={tilt:.1f}")

    def publish_stop(self) -> None:
        message = Axis()
        message.stamp = self.get_clock().now().to_msg()
        message.pan = 0.0
        message.tilt = 0.0
        self.command_publisher.publish(message)
        self.last_command_time = time.monotonic()
        self.ptz_moving = False

    def check_watchdog(self) -> None:
        if self.ptz_moving and time.monotonic() - self.last_command_time > self.command_timeout:
            self.publish_stop()
            self.get_logger().warning("Web PTZ timeout -> automatic stop")

    async def index_handler(self, _request: web.Request) -> web.Response:
        return web.json_response({
            "service": "axis_web_bridge",
            "stream": "/stream.mjpg",
            "websocket": "/ws",
            "health": "/health",
        })

    async def health_handler(self, _request: web.Request) -> web.Response:
        return web.json_response({
            "ok": True,
            "imageReceived": bool(self.latest_image),
            "imageSequence": self.image_sequence,
            "stateReceived": bool(self.latest_state.get("connected")),
            "uptimeSeconds": round(time.monotonic() - self.started_at, 1),
        }, headers={"Access-Control-Allow-Origin": "*"})

    async def stream_handler(self, _request: web.Request) -> web.StreamResponse:
        response = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
                "Access-Control-Allow-Origin": "*",
            },
        )
        await response.prepare(_request)
        last_sequence = -1
        try:
            while True:
                if self.latest_image and self.image_sequence != last_sequence:
                    image = self.latest_image
                    await response.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(image)}\r\n\r\n".encode()
                        + image
                        + b"\r\n"
                    )
                    last_sequence = self.image_sequence
                await asyncio.sleep(0.01)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def websocket_handler(self, request: web.Request) -> web.WebSocketResponse:
        socket = web.WebSocketResponse(heartbeat=15.0)
        await socket.prepare(request)
        await socket.send_json({"type": "connection_status", "status": "ok"})
        last_state_sequence = -1

        async def send_states() -> None:
            nonlocal last_state_sequence
            while not socket.closed:
                if self.state_sequence != last_state_sequence:
                    await socket.send_json(self.latest_state)
                    last_state_sequence = self.state_sequence
                await asyncio.sleep(0.05)

        state_task = asyncio.create_task(send_states())
        try:
            async for message in socket:
                if message.type != WSMsgType.TEXT:
                    continue
                try:
                    packet = json.loads(message.data)
                except json.JSONDecodeError:
                    await socket.send_json({"type": "error", "message": "invalid_json"})
                    continue

                if packet.get("type") == "camera_ptz":
                    self.command_queue.put(packet)
                    await socket.send_json({"type": "command_ack", "command": "camera_ptz"})
        finally:
            state_task.cancel()
            self.command_queue.put({"pan": 0.0, "tilt": 0.0, "active": False})
        return socket

    def run_web_server(self) -> None:
        asyncio.set_event_loop(self.web_loop)
        app = web.Application()
        app.router.add_get("/", self.index_handler)
        app.router.add_get("/health", self.health_handler)
        app.router.add_get("/stream.mjpg", self.stream_handler)
        app.router.add_get("/ws", self.websocket_handler)
        runner = web.AppRunner(app)

        async def start() -> None:
            await runner.setup()
            site = web.TCPSite(runner, self.bind_host, self.port)
            await site.start()

        self.web_loop.run_until_complete(start())
        self.web_loop.run_forever()

    def destroy_node(self) -> bool:
        if self.ptz_moving:
            self.publish_stop()
        if self.web_loop.is_running():
            self.web_loop.call_soon_threadsafe(self.web_loop.stop)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AxisWebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
