#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO

LIGHT_12V = 13      # AGX Orin J30 13번 = GPIO32 (gpio456)
LIGHT_5V  = 18     # AGX Orin J30 22번 = GPIO17 (gpio444)

ON  = GPIO.HIGH
OFF = GPIO.LOW


class LightNode(Node):
    def __init__(self):
        super().__init__('light_node')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LIGHT_12V, GPIO.OUT, initial=OFF)
        GPIO.setup(LIGHT_5V,  GPIO.OUT, initial=OFF)

        self.sub = self.create_subscription(
            Int32, '/server/light_cmd', self.on_cmd, 10)
        self.get_logger().info(
            f'라이트 노드 시작 (12V=핀{LIGHT_12V}, 5V=핀{LIGHT_5V}). /server/light_cmd 대기 중')
        self.get_logger().info('0=12V끄기, 1=12V켜기, 2=5V켜기, 3=5V끄기')

    def on_cmd(self, msg):
        if msg.data == 0:
            self.set_light(LIGHT_12V, '12V', False)
        elif msg.data == 1:
            self.set_light(LIGHT_12V, '12V', True)
        elif msg.data == 2:
            self.set_light(LIGHT_5V, '5V', True)
        elif msg.data == 3:
            self.set_light(LIGHT_5V, '5V', False)
        else:
            self.get_logger().warn(f'모르는 명령: {msg.data} (0~3만 가능)')

    def set_light(self, pin, name, on):
        GPIO.output(pin, ON if on else OFF)
        self.get_logger().info(
            f'핀 {pin} -> {"HIGH" if on else "LOW"} / {name} 라이트 {"ON" if on else "OFF"}')


def main():
    rclpy.init()
    node = LightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.output(LIGHT_12V, OFF)
        GPIO.output(LIGHT_5V, OFF)
        GPIO.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
