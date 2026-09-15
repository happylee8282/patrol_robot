import re
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class AxisCameraNode(Node):

    def __init__(self):
        super().__init__('axis_camera_node')

        self.declare_parameter('hostname', '192.168.0.90')
        self.declare_parameter('mjpeg_path', '/mjpg/video.mjpg')
        self.declare_parameter('frame_id', 'axis_camera')
        self.declare_parameter(
            'topic_name',
            '/axis/image_raw/compressed'
        )

        self.hostname = (
            self.get_parameter('hostname')
            .get_parameter_value()
            .string_value
        )

        self.mjpeg_path = (
            self.get_parameter('mjpeg_path')
            .get_parameter_value()
            .string_value
        )

        self.frame_id = (
            self.get_parameter('frame_id')
            .get_parameter_value()
            .string_value
        )

        topic_name = (
            self.get_parameter('topic_name')
            .get_parameter_value()
            .string_value
        )

        self.publisher_ = self.create_publisher(
            CompressedImage,
            topic_name,
            10
        )

        self.url = f'http://{self.hostname}{self.mjpeg_path}'

        self.stream = None
        self.boundary = None

        self.get_logger().info(
            f'Connecting to AXIS camera: {self.url}'
        )

        self.connect()

        self.timer = self.create_timer(
            0.001,
            self.read_and_publish_frame
        )

    def connect(self):
        try:
            self.stream = urllib.request.urlopen(
                self.url,
                timeout=10
            )

            content_type = self.stream.headers.get(
                'Content-Type',
                ''
            )

            match = re.search(
                r'boundary="?([^";]+)',
                content_type
            )

            if match:
                boundary = match.group(1)

                if not boundary.startswith('--'):
                    boundary = '--' + boundary

                self.boundary = boundary.encode()

            else:
                self.boundary = b'--myboundary'

            self.get_logger().info(
                f'Connected. MJPEG boundary: '
                f'{self.boundary.decode(errors="ignore")}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Camera connection failed: {e}'
            )
            self.stream = None

    def find_boundary(self):
        while rclpy.ok():
            line = self.stream.readline()

            if not line:
                raise ConnectionError(
                    'MJPEG stream closed while searching boundary'
                )

            if line.strip() == self.boundary:
                return

    def read_headers(self):
        headers = {}

        while rclpy.ok():
            line = self.stream.readline()

            if not line:
                raise ConnectionError(
                    'MJPEG stream closed while reading headers'
                )

            if line in (b'\r\n', b'\n'):
                break

            line = line.decode(
                'latin-1',
                errors='replace'
            ).strip()

            if ':' in line:
                key, value = line.split(':', 1)
                headers[key.strip().lower()] = value.strip()

        return headers

    def read_and_publish_frame(self):
        if self.stream is None:
            self.connect()
            return

        try:
            self.find_boundary()
            headers = self.read_headers()

            if 'content-length' not in headers:
                self.get_logger().warning(
                    'MJPEG frame has no Content-Length'
                )
                return

            content_length = int(
                headers['content-length']
            )

            jpeg_data = self.stream.read(
                content_length
            )

            if len(jpeg_data) != content_length:
                raise ConnectionError(
                    'Incomplete JPEG frame received'
                )

            msg = CompressedImage()

            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = jpeg_data

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warning(
                f'MJPEG read error: {e}'
            )

            try:
                if self.stream is not None:
                    self.stream.close()
            except Exception:
                pass

            self.stream = None


def main(args=None):
    rclpy.init(args=args)
    node = AxisCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.stream is not None:
            node.stream.close()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
