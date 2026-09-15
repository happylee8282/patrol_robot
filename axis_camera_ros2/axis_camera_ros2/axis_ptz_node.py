import time
import http.client
import urllib.parse

import rclpy
from rclpy.node import Node

from axis_camera_msgs.msg import Axis
from std_msgs.msg import Bool


class AxisPtzNode(Node):

    def __init__(self):
        super().__init__('axis_ptz_node')

        # AXIS 카메라 주소
        self.declare_parameter('hostname', '192.168.11.90')

        # 제스처 명령이 끊겼을 때 자동 정지 시간
        self.declare_parameter('command_timeout', 0.5)

        # CCTV 초기 위치
        self.declare_parameter('home_pan', 3.95)
        self.declare_parameter('home_tilt', 5.85)
        self.declare_parameter('home_zoom', 700.0)

        self.hostname = self.get_parameter('hostname').value
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )

        self.home_pan = float(
            self.get_parameter('home_pan').value
        )
        self.home_tilt = float(
            self.get_parameter('home_tilt').value
        )
        self.home_zoom = float(
            self.get_parameter('home_zoom').value
        )

        self.last_cmd_time = time.monotonic()
        self.is_moving = False

        # 글래스 PTZ 이동 명령
        self.cmd_sub = self.create_subscription(
            Axis,
            '/axis/axis_cmd',
            self.cmd_callback,
            10
        )

        # 안스 앱 초기 위치 복귀 명령
        self.home_sub = self.create_subscription(
            Bool,
            '/axis/home_cmd',
            self.home_callback,
            10
        )

        # 현재 PTZ 상태 발행
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
            f'PTZ home position: '
            f'pan={self.home_pan}, '
            f'tilt={self.home_tilt}, '
            f'zoom={self.home_zoom}'
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

        zoom_speed = max(
            -100.0,
            min(100.0, float(msg.zoom))
        )

        query = {
            'continuouspantiltmove':
                f'{pan_speed:.0f},{tilt_speed:.0f}',
            'continuouszoommove':
                f'{zoom_speed:.0f}'
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
                or abs(zoom_speed) > 0.0
            )

            self.get_logger().info(
                f'PTZ command: '
                f'pan={pan_speed:.0f}, '
                f'tilt={tilt_speed:.0f}, '
                f'zoom={zoom_speed:.0f}'
            )

        except Exception as error:
            self.get_logger().error(
                f'PTZ command failed: {error}'
            )

    def home_callback(self, msg):
        if not msg.data:
            return

        # 지정된 절대 위치로 카메라 이동
        query = {
            'pan': f'{self.home_pan:.2f}',
            'tilt': f'{self.home_tilt:.2f}',
            'zoom': f'{self.home_zoom:.0f}',
        }

        path = (
            '/axis-cgi/com/ptz.cgi?'
            + urllib.parse.urlencode(query)
        )

        try:
            # 먼저 기존 연속 이동 정지
            self.stop_camera()

            # 초기 위치로 이동
            self.http_get(path)
            self.is_moving = False

            self.get_logger().info(
                f'PTZ home completed: '
                f'pan={self.home_pan:.2f}, '
                f'tilt={self.home_tilt:.2f}, '
                f'zoom={self.home_zoom:.0f}'
            )

        except Exception as error:
            self.get_logger().error(
                f'PTZ home failed: {error}'
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
            msg.stamp = self.get_clock().now().to_msg()

            msg.pan = self.to_float(values.get('pan'))
            msg.tilt = self.to_float(values.get('tilt'))
            msg.zoom = self.to_float(values.get('zoom'))
            msg.focus = self.to_float(values.get('focus'))
            msg.brightness = self.to_float(
                values.get('brightness')
            )
            msg.iris = self.to_float(values.get('iris'))

            msg.pan_r = 0.0
            msg.tilt_r = 0.0

            autofocus = values.get(
                'autofocus',
                'on'
            ).lower()

            msg.autofocus = autofocus in (
                'on',
                'true',
                '1'
            )

            self.state_pub.publish(msg)

        except Exception as error:
            self.get_logger().warning(
                f'PTZ state query failed: {error}'
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
            'continuouspantiltmove': '0,0',
            'continuouszoommove': '0'
        }

        path = (
            '/axis-cgi/com/ptz.cgi?'
            + urllib.parse.urlencode(query)
        )

        try:
            self.http_get(path)
            self.is_moving = False

        except Exception as error:
            self.get_logger().warning(
                f'PTZ stop failed: {error}'
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
