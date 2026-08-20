#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""키보드 입력으로 모터 제어기의 Auto/Manual 모드를 변경한다."""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int16


MODE_AUTO = 65       # ASCII 'A'
MODE_MANUAL = 77     # ASCII 'M'


class ModeKeyboardNode(Node):
    """터미널 키 입력을 mode_cmd 토픽으로 발행하는 노드."""

    def __init__(self) -> None:
        super().__init__('mode_keyboard')
        self.mode_pub = self.create_publisher(
            Int16,
            'mode_cmd',
            QoSProfile(depth=1),
        )

    def publish_mode(self, mode: int) -> None:
        msg = Int16()
        msg.data = mode
        self.mode_pub.publish(msg)

        mode_name = 'Auto' if mode == MODE_AUTO else 'Manual'
        self.get_logger().info(f'{mode_name} mode command published ({mode})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeKeyboardNode()

    if not sys.stdin.isatty():
        node.get_logger().error('A terminal (TTY) is required for keyboard input')
        node.destroy_node()
        rclpy.shutdown()
        return

    original_terminal_settings = termios.tcgetattr(sys.stdin)

    print(
        '\nMode keyboard control\n'
        '---------------------\n'
        'a : Auto mode\n'
        'm : Manual mode\n'
        'q : Manual mode and quit\n'
    )

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            # ROS 이벤트를 처리하면서 최대 0.1초 동안 키 입력을 기다린다.
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)

            if not readable:
                continue

            key = sys.stdin.read(1).lower()

            if key == 'a':
                node.publish_mode(MODE_AUTO)
            elif key == 'm':
                node.publish_mode(MODE_MANUAL)
            elif key == 'q':
                node.publish_mode(MODE_MANUAL)
                # 발행 데이터가 DDS 계층으로 전달될 시간을 준다.
                rclpy.spin_once(node, timeout_sec=0.1)
                break

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard control interrupted')
    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            original_terminal_settings,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
