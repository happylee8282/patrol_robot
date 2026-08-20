import time
import http.client
import urllib.parse

import rclpy
from rclpy.node import Node

from axis_camera_msgs.msg import Axis


class AxisPtzNode(Node):

    def __init__(self):
        super().__init__('axis_ptz_node')

        self.declare_parameter('hostname', '192.168.0.90')
        self.declare_parameter('command_timeout', 0.5)

        self.hostname = (
            self.get_parameter('hostname')
            .get_parameter_value()
            .string_value
        )

        self.command_timeout = (
            self.get_parameter('command_timeout')
            .get_parameter_value()
            .double_value
        )

        self.last_cmd_time = time.monotonic()
        self.is_moving = False

        self.cmd_sub = self.create_subscription(
            Axis,
            '/axis/axis_cmd',
            self.cmd_callback,
            10
        )

        self.state_pub = self.create_publisher(
            Axis,
            '/axis/state',
            10
        )

        self.state_timer = self.create_timer(
            0.1,
            self.publish_state
        )

        self.watchdog_timer = self.create_timer(
            0.1,
            self.watchdog_callback
        )

        self.get_logger().info(
            f'AXIS PTZ node started: {self.hostname}'
        )

        self.get_logger().info(
            f'PTZ command timeout: '
            f'{self.command_timeout:.2f} sec'
        )

    def http_get(self, path):
        conn = http.client.HTTPConnection(
            self.hostname,
            timeout=5
        )

        try:
            conn.request('GET', path)
            response = conn.getresponse()

            body = response.read().decode(
                'latin-1',
                errors='replace'
            )

            if response.status not in (200, 204):
                raise RuntimeError(
                    f'HTTP {response.status}: {body}'
                )

            return body

        finally:
            conn.close()

    def cmd_callback(self, msg):
        pan_speed = max(
            -100.0,
            min(100.0, float(msg.pan))
        )

        tilt_speed = max(
            -100.0,
            min(100.0, float(msg.tilt))
        )

        query = {
            'continuouspantiltmove':
                f'{pan_speed:.0f},{tilt_speed:.0f}'
        }

        path = (
            '/axis-cgi/com/ptz.cgi?'
            + urllib.parse.urlencode(query)
        )

        try:
            self.http_get(path)

            self.last_cmd_time = time.monotonic()

            self.is_moving = (
                abs(pan_speed) > 0.0
                or abs(tilt_speed) > 0.0
            )

            self.get_logger().info(
                f'PTZ command: '
                f'pan={pan_speed:.0f}, '
                f'tilt={tilt_speed:.0f}'
            )

        except Exception as e:
            self.get_logger().error(
                f'PTZ command failed: {e}'
            )

    def publish_state(self):
        try:
            body = self.http_get(
                '/axis-cgi/com/ptz.cgi?query=position'
            )

            values = {}

            for line in body.splitlines():
                if '=' not in line:
                    continue

                key, value = line.split('=', 1)
                values[key.strip()] = value.strip()

            msg = Axis()

            msg.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.pan = self.to_float(values.get('pan'))
            msg.tilt = self.to_float(values.get('tilt'))
            msg.zoom = self.to_float(values.get('zoom'))
            msg.focus = self.to_float(values.get('focus'))
            msg.brightness = self.to_float(values.get('brightness'))
            msg.iris = self.to_float(values.get('iris'))

            msg.pan_r = 0.0
            msg.tilt_r = 0.0

            autofocus = (
                values.get('autofocus', 'on')
                .lower()
            )

            msg.autofocus = autofocus in (
                'on',
                'true',
                '1'
            )

            self.state_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(
                f'PTZ state query failed: {e}'
            )

    def watchdog_callback(self):
        if not self.is_moving:
            return

        elapsed = (
            time.monotonic()
            - self.last_cmd_time
        )

        if elapsed > self.command_timeout:
            self.stop_camera()

            self.get_logger().warning(
                'PTZ command timeout -> automatic stop'
            )

    def stop_camera(self):
        query = {
            'continuouspantiltmove': '0,0'
        }

        path = (
            '/axis-cgi/com/ptz.cgi?'
            + urllib.parse.urlencode(query)
        )

        try:
            self.http_get(path)
            self.is_moving = False

        except Exception as e:
            self.get_logger().warning(
                f'PTZ stop failed: {e}'
            )

    @staticmethod
    def to_float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = AxisPtzNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_camera()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
