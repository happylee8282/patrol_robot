#!/usr/bin/env python3
"""CoastalPatrol Meta Display WebSocket <-> ROS 2 gateway.

Run this inside a sourced ROS 2 environment. Topic names are read from
topics.json so the webapp does not need to change when the robot team renames
a topic.
"""

import asyncio
import json
import math
import os
import pathlib
import threading
import time

from ament_index_python.packages import get_package_share_directory

# CoastalPatrol robot-wide ROS 2 DDS domain.
# Set before rclpy.init() so the gateway always joins domain 84.
os.environ["ROS_DOMAIN_ID"] = "84"

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16, Int32, String
import websockets

try:
    from axis_camera_msgs.msg import Axis
except ImportError:
    Axis = None


PACKAGE_SHARE = pathlib.Path(get_package_share_directory("gateway_pk"))
CONFIG_DIRECTORY = PACKAGE_SHARE / "config"
TOPICS = json.loads(
    (CONFIG_DIRECTORY / "topics.json").read_text(encoding="utf-8")
)
# Runtime state must remain writable even when the package is installed under
# /opt or another read-only prefix.
STATE_DIRECTORY = pathlib.Path(
    os.environ.get("XDG_STATE_HOME", pathlib.Path.home() / ".ros")
) / "gateway_pk"
STATE_FILE = STATE_DIRECTORY / "navigation_state.json"
WAYPOINT_SCHEME = "former_wp2_is_wp1_blue_corner_is_wp2_v1"


def load_current_waypoint():
    try:
        state = json.loads(STATE_FILE.read_text(encoding="utf-8"))
        waypoint = int(state.get("currentWaypoint", 1))
        if state.get("waypointScheme") != WAYPOINT_SCHEME:
            waypoint = 1
        return waypoint if 1 <= waypoint <= 4 else 1
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return 1


class MetaGateway(Node):
    def __init__(self, broadcast):
        super().__init__("meta_glass_gateway")
        self.broadcast = broadcast
        self.last_drive_at = 0.0
        self.was_moving = False
        self.last_ptz_at = 0.0
        self.ptz_moving = False
        self.goal_tracking_active = False
        self.selected_waypoint = 0
        self.waypoint_direction = 1
        self.current_waypoint = load_current_waypoint()
        self.completed_waypoints = {self.current_waypoint}
        self.save_current_waypoint()
        self.axis_enabled = Axis is not None
        safety = TOPICS.get("safety", {})
        self.max_linear = float(safety.get("max_linear", 0.10))
        self.max_lateral = float(safety.get("max_lateral", self.max_linear))
        self.max_angular = float(safety.get("max_angular", 0.30))
        self.drive_publish_hz = max(10.0, min(100.0, float(safety.get("drive_publish_hz", 50.0))))
        self.latched_drive_command = None

        outgoing = TOPICS["glasses_to_robot"]
        incoming = TOPICS["robot_to_glasses"]

        self.x_vel_pub = self.create_publisher(Float32, outgoing["x_vel"], 10)
        self.z_angle_pub = self.create_publisher(Float32, outgoing["z_angle"], 10)
        self.robot_cmd_pub = self.create_publisher(Twist, outgoing["robot_cmd"], 10)
        self.robot_mode_pub = self.create_publisher(Int16, outgoing["robot_mode"], 10)
        self.light_cmd_pub = self.create_publisher(Int32, outgoing["light_cmd"], 10)
        # Keep the latest speaker state so robot_speaker can recover it after
        # that node is stopped and restarted.
        speaker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.warning_broadcast_pub = self.create_publisher(
            Int32,
            outgoing["warning_broadcast"],
            speaker_qos,
        )
        self.waypoint_goal_pub = self.create_publisher(Int32, outgoing["waypoint_goal"], 10)
        self.axis_home_pub = self.create_publisher(Bool, outgoing.get("axis_home_cmd", "/axis/home_cmd"), 10)
        self.axis_cmd_pub = (
            self.create_publisher(Axis, outgoing["axis_cmd"], 10)
            if self.axis_enabled else None
        )

        self.create_subscription(Int32, incoming["connection_status"], self.on_connection, 10)
        self.battery_array_index = int(incoming.get("battery_array_index", 0))
        self.create_subscription(
            String,
            incoming.get("battery_status", "/battery_status"),
            self.on_battery_status,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            incoming["battery_array"],
            self.on_battery_array,
            10,
        )
        self.create_subscription(BatteryState, incoming["battery_state"], self.on_battery_state, 10)
        self.create_subscription(Int32, incoming["robot_area"], self.on_robot_area, 10)
        self.create_subscription(Int32, incoming["warning_broadcast_state"], self.on_warning_broadcast_state, 10)
        self.create_subscription(Float32, incoming["robot_state"], self.on_robot_state, 10)
        self.create_subscription(
            Twist,
            incoming.get("cmd_vel", "/cmd_vel"),
            self.on_cmd_vel,
            10,
        )
        self.create_subscription(Int32, incoming["goal_success"], self.on_goal_success, 10)
        if self.axis_enabled:
            self.create_subscription(Axis, incoming["axis_state"], self.on_axis_state, 10)
        else:
            self.get_logger().warning(
                "axis_camera_msgs is unavailable; PTZ support is disabled, cmd_vel remains active"
            )
        # Republish a latched direction at 50 Hz by default.  The browser only
        # selects/changes the latch; this ROS timer provides the uninterrupted
        # command stream expected by the motor controller.
        self.create_timer(0.1, self.drive_watchdog)
        self.create_timer(1.0 / self.drive_publish_hz, self.drive_watchdog)
        self.get_logger().info(
            f"Latched /cmd_vel publish rate: {self.drive_publish_hz:.1f} Hz"
        )

    @staticmethod
    def publish_float(publisher, value):
        message = Float32()
        message.data = float(value)
        publisher.publish(message)

    @staticmethod
    def publish_int(publisher, value):
        message = Int32()
        message.data = int(value)
        publisher.publish(message)

    def stop_robot(self):
        self.latched_drive_command = None
        self.publish_float(self.x_vel_pub, 0.0)
        self.publish_float(self.z_angle_pub, 0.0)
        self.robot_cmd_pub.publish(Twist())
        self.was_moving = False

    def publish_robot_mode(self, value):
        # Robot contract: 0=manual, 1=auto.
        message = Int16()
        message.data = 1 if int(value) == 1 else 0
        self.robot_mode_pub.publish(message)

    def stop_axis(self):
        if not self.axis_enabled:
            return
        message = Axis()
        message.stamp = self.get_clock().now().to_msg()
        self.axis_cmd_pub.publish(message)
        self.ptz_moving = False

    def drive_watchdog(self):
        if self.latched_drive_command is not None:
            command = self.latched_drive_command
            self.robot_cmd_pub.publish(command)
            self.publish_float(self.x_vel_pub, command.linear.x)
            self.publish_float(self.z_angle_pub, command.angular.z)
            return
        if self.was_moving and time.monotonic() - self.last_drive_at > 0.4:
            self.get_logger().warning("Drive command timeout: publishing stop")
            self.stop_robot()
        if self.ptz_moving and time.monotonic() - self.last_ptz_at > 0.5:
            self.get_logger().warning("PTZ command timeout: publishing stop")
            self.stop_axis()

    def handle_web_message(self, data):
        message_type = str(data.get("type", ""))

        if message_type == "cmd_vel":
            x_vel = float(data.get("linear", data.get("x_vel", 0.0)))
            y_vel = float(data.get("lateral", data.get("y_vel", 0.0)))
            z_angle = float(data.get("angular", data.get("z_angle", 0.0)))
            print("[WS RX] " + json.dumps(data, ensure_ascii=False), flush=True)
            source = str(data.get("source", ""))

            # Old glasses builds also emit implicit zero-speed messages from
            # the former gesture/PTZ controller.  They must not overwrite a
            # latched v214 direction command.  Explicit v214 stop commands
            # still pass through this handler normally.
            if source in {"meta_neural_band", "meta_direction_button"}:
                self.get_logger().warning(
                    f"Ignored legacy cmd_vel source: {source}"
                )
                return

            x_vel = float(data.get("linear_x", data.get("linear", data.get("x_vel", 0.0))))
            y_vel = float(data.get("linear_y", data.get("lateral", data.get("y_vel", 0.0))))
            z_angle = float(data.get("angular_z", data.get("angular", data.get("z_angle", 0.0))))
            if not all(math.isfinite(value) for value in (x_vel, y_vel, z_angle)):
                self.get_logger().warning("Ignored non-finite drive command")
                self.stop_robot()
                return
            x_vel = max(-self.max_linear, min(self.max_linear, x_vel))
            y_vel = max(-self.max_lateral, min(self.max_lateral, y_vel))
            z_angle = max(-self.max_angular, min(self.max_angular, z_angle))
            command = Twist()
            command.linear.x = x_vel
            command.linear.y = y_vel
            command.angular.z = z_angle
            print(
                f"[CMD_VEL] linear.x={x_vel}, angular.z={z_angle}",
                flush=True,
            )
            if bool(data.get("continuous", False)) and bool(x_vel or y_vel or z_angle):
                self.latched_drive_command = command
            else:
                self.latched_drive_command = None
            self.robot_cmd_pub.publish(command)
            self.publish_float(self.x_vel_pub, x_vel)
            self.publish_float(self.z_angle_pub, z_angle)
            self.last_drive_at = time.monotonic()
            self.was_moving = bool(x_vel or y_vel or z_angle)
        elif message_type == "robot_mode":
            self.stop_robot()
            self.publish_robot_mode(data.get("value", 0))
        elif message_type == "control_mode":
            # Robot-mode UI is not implemented yet. Joystick off only stops;
            # it must not publish /server/robot_mode implicitly.
            # Entering/refreshing joystick mode must not cancel a direction
            # that is already latched. Only an explicit mode-off stops it.
            if not bool(data.get("active", False)):
                self.stop_robot()
        elif message_type == "camera_ptz":
            if not self.axis_enabled:
                self.get_logger().warning("Ignored camera_ptz: axis_camera_msgs is unavailable")
                return
            active = bool(data.get("active", False))
            pan = max(-100.0, min(100.0, float(data.get("pan", 0.0)))) if active else 0.0
            tilt = max(-100.0, min(100.0, float(data.get("tilt", 0.0)))) if active else 0.0
            zoom = float(data.get("zoom", 0.0)) if active else 0.0
            if not all(math.isfinite(value) for value in (pan, tilt, zoom)):
                self.stop_axis()
                return
            message = Axis()
            message.stamp = self.get_clock().now().to_msg()
            message.pan = pan
            message.tilt = tilt
            message.zoom = zoom
            self.axis_cmd_pub.publish(message)
            self.last_ptz_at = time.monotonic()
            self.ptz_moving = bool(pan or tilt or zoom)
        elif message_type == "camera_home":
            message = Bool()
            message.data = True
            self.axis_home_pub.publish(message)
            self.ptz_moving = False
        elif message_type == "emergency_stop":
            self.stop_robot()
        elif message_type == "headlight":
            # Unified light command: 1=front ON, 0=front OFF.
            self.publish_int(self.light_cmd_pub, 1 if data.get("active") else 0)
        elif message_type == "warning_light":
            # Unified light command: 2=top warning ON, 3=top warning OFF.
            self.publish_int(self.light_cmd_pub, 2 if data.get("active") else 3)
        elif message_type == "emergency_broadcast":
            self.publish_int(self.warning_broadcast_pub, 1 if data.get("active") else 0)
        elif message_type == "waypoint_goal":
            waypoint = int(data.get("waypoint", 0))
            if 1 <= waypoint <= 4:
                # /robot_nav/goal carries the selected destination waypoint (1..4).
                # Stop manual motion before replacing the active navigation target.
                self.stop_robot()
                self.selected_waypoint = waypoint
                self.waypoint_direction = -1 if data.get("direction") == "reverse" else 1
                self.completed_waypoints = set()
                self.goal_tracking_active = True
                self.publish_int(self.waypoint_goal_pub, waypoint)
            else:
                self.get_logger().warning(f"Ignored invalid waypoint: {waypoint}")

    def on_connection(self, message):
        names = {0: "disconnected", 1: "connected", 2: "degraded", 3: "error"}
        self.broadcast({"type": "connection_status", "value": message.data, "status": names.get(message.data, "error")})

    def broadcast_battery(self, percent):
        percent = max(0, min(100, round(float(percent))))
        self.broadcast({"type": "robot_status", "battery": percent})

    def on_battery_status(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError) as error:
            self.get_logger().warning(f"Ignored invalid /battery_status JSON: {error}")
            return

        if not isinstance(status, dict) or "soc" not in status:
            self.get_logger().warning("Ignored /battery_status message without soc")
            return

        try:
            soc = float(status["soc"])
        except (TypeError, ValueError):
            self.get_logger().warning(f"Ignored invalid battery SOC: {status.get('soc')}")
            return

        if not math.isfinite(soc) or not 0.0 <= soc <= 100.0:
            self.get_logger().warning(f"Ignored out-of-range battery SOC: {soc}")
            return

        self.broadcast_battery(soc)

    def on_battery_array(self, message):
        values = list(message.data)
        if not values:
            self.get_logger().warning("Ignored empty /robot/can/battery array")
            return
        if not 0 <= self.battery_array_index < len(values):
            self.get_logger().warning(
                f"Battery index {self.battery_array_index} is outside array length {len(values)}"
            )
            return
        value = float(values[self.battery_array_index])
        if not math.isfinite(value) or value < 0:
            self.get_logger().warning(f"Ignored invalid battery value: {value}")
            return
        # Accept either a 0.0..1.0 SOC ratio or an already converted 0..100 value.
        self.broadcast_battery(value * 100.0 if value <= 1.0 else value)

    def on_battery_state(self, message):
        # sensor_msgs/BatteryState.percentage is normally 0.0..1.0; NaN means unknown.
        if message.percentage == message.percentage:
            self.broadcast_battery(message.percentage * 100.0)

    def on_robot_area(self, message):
        if 1 <= message.data <= 4:
            self.broadcast({"type": "robot_position", "waypoint": message.data, "status": "moving"})

    def on_warning_broadcast_state(self, message):
        self.broadcast({"type": "warning_broadcast_state", "value": message.data, "active": message.data == 1})

    def on_robot_state(self, message):
        self.broadcast({"type": "robot_status", "speed": float(message.data)})

    def on_cmd_vel(self, message):
        linear_x = float(message.linear.x)
        linear_y = float(message.linear.y)
        speed_mps = (linear_x ** 2 + linear_y ** 2) ** 0.5
        self.broadcast(
            {
                "type": "robot_status",
                "speed": speed_mps,
                "speedUnit": "m/s",
                "velocity": {
                    "linearX": linear_x,
                    "linearY": linear_y,
                    "angularZ": float(message.angular.z),
                },
            }
        )

    def on_goal_success(self, message):
        # /robot_nav/goal_success carries the waypoint currently reached by the robot (1..4).
        current_waypoint = int(message.data)
        if not 1 <= current_waypoint <= 4:
            self.get_logger().warning(f"Ignored invalid goal_success waypoint: {current_waypoint}")
            return
        arrived = self.selected_waypoint in range(1, 5) and current_waypoint == self.selected_waypoint
        self.current_waypoint = current_waypoint
        self.save_current_waypoint()
        # Keep only the robot's latest reached waypoint. Passed waypoints return
        # to their neutral color instead of remaining green.
        self.completed_waypoints = {current_waypoint}
        if self.selected_waypoint in range(1, 5):
            if self.waypoint_direction > 0:
                remaining = (self.selected_waypoint - current_waypoint) % 4
            else:
                remaining = (current_waypoint - self.selected_waypoint) % 4
        else:
            remaining = 0
        self.goal_tracking_active = not arrived
        self.broadcast({
            "type": "robot_position",
            "waypoint": current_waypoint,
            "status": "arrived" if arrived else "moving",
        })
        self.broadcast({
            "type": "goal_progress",
            "waypoint": self.selected_waypoint,
            "arrived": arrived,
            "remaining": remaining,
        })
        self.broadcast(self.navigation_state())

    def save_current_waypoint(self):
        try:
            STATE_DIRECTORY.mkdir(parents=True, exist_ok=True)
            temporary = STATE_FILE.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(
                    {
                        "currentWaypoint": self.current_waypoint,
                        "waypointScheme": WAYPOINT_SCHEME,
                    },
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            temporary.replace(STATE_FILE)
        except OSError as error:
            self.get_logger().warning(f"Could not persist current waypoint: {error}")

    def navigation_state(self):
        return {
            "type": "navigation_state",
            "targetWaypoint": self.selected_waypoint or None,
            "currentWaypoint": self.current_waypoint or None,
            "completedWaypoints": sorted(self.completed_waypoints),
            "direction": "reverse" if self.waypoint_direction < 0 else "forward",
            "active": self.goal_tracking_active,
            "arrived": bool(
                self.selected_waypoint
                and self.current_waypoint == self.selected_waypoint
                and not self.goal_tracking_active
            ),
        }

    def on_axis_state(self, message):
        self.broadcast({
            "type": "ptz_state",
            "pan": float(message.pan),
            "tilt": float(message.tilt),
            "zoom": float(message.zoom),
            "connected": True,
        })


class GatewayServer:
    def __init__(self):
        self.clients = set()
        self.loop = None
        self.node = None

    def broadcast_from_ros(self, payload):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.broadcast(payload), self.loop)

    async def broadcast(self, payload):
        if not self.clients:
            return
        encoded = json.dumps(payload, ensure_ascii=False)
        await asyncio.gather(*(client.send(encoded) for client in tuple(self.clients)), return_exceptions=True)

    async def handle_client(self, websocket):
        self.clients.add(websocket)
        await websocket.send(json.dumps({"type": "connection_status", "status": "connected", "value": 1}))
        await websocket.send(json.dumps(self.node.navigation_state(), ensure_ascii=False))
        try:
            async for raw in websocket:
                try:
                    data = json.loads(raw)
                except (json.JSONDecodeError, TypeError):
                    continue
                message_type = str(data.get("type", ""))
                if message_type in {
                    "display_state", "display_state_request", "waypoint_direction",
                    "waypoint_reset",
                    "risk_alert", "alert", "safety_alert",
                }:
                    if message_type == "waypoint_reset":
                        self.node.selected_waypoint = 0
                        self.node.current_waypoint = 1
                        self.node.completed_waypoints = set()
                        self.node.goal_tracking_active = False
                        self.node.save_current_waypoint()
                    await self.broadcast(data)
                    if message_type == "waypoint_reset":
                        await self.broadcast(self.node.navigation_state())
                else:
                    self.node.handle_web_message(data)
                    if message_type == "waypoint_goal":
                        waypoint = int(data.get("waypoint", 0))
                        if 1 <= waypoint <= 4:
                            await self.broadcast({
                                "type": "waypoint_selected",
                                "waypoint": waypoint,
                                "direction": data.get("direction", "forward"),
                                "source": data.get("source", "unknown"),
                            })
        finally:
            self.clients.discard(websocket)
            if not self.clients:
                self.node.stop_robot()
                self.node.stop_axis()

    async def run(self):
        self.loop = asyncio.get_running_loop()
        rclpy.init()
        self.node = MetaGateway(self.broadcast_from_ros)
        ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        ros_thread.start()
        host = TOPICS.get("websocket", {}).get("host", "0.0.0.0")
        port = int(TOPICS.get("websocket", {}).get("port", 8765))
        self.node.get_logger().info(f"WebSocket gateway listening on ws://{host}:{port}")
        try:
            async with websockets.serve(self.handle_client, host, port):
                await asyncio.Future()
        finally:
            # rclpy's SIGINT handler may already have invalidated the context.
            # Publish the final stop only while publishers are still usable.
            if rclpy.ok():
                self.node.stop_robot()
                self.node.stop_axis()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            ros_thread.join(timeout=1.0)


def main(args=None):
    """Run the ROS 2/WebSocket gateway console entry point."""
    try:
        asyncio.run(GatewayServer().run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
