import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2


class RGBDetector(Node):

    def __init__(self):
        super().__init__('rgb_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/axis/image_raw/compressed',
            self.image_callback,
            10
        )

        self.get_logger().info('RGB visualization started')


    def image_callback(self, msg):

        # ROS Image -> OpenCV image
        image = self.bridge.compressed_imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        h, w, _ = image.shape

        # 중앙 픽셀
        cx = w // 2
        cy = h // 2

        # OpenCV는 BGR 순서
        b, g, r = image[cy, cx]

        # 화면에 표시할 원 표시
        cv2.circle(
            image,
            (cx, cy),
            5,
            (0, 0, 255),
            -1
        )

        # RGB 문자열
        text = f'RGB: ({r}, {g}, {b})'

        cv2.putText(
            image,
            text,
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        # 출력
        cv2.imshow(
            'AXIS RGB Viewer',
            image
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = RGBDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
