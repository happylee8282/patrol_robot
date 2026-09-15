#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""키보드 또는 서버 토픽으로 모터 제어기의 Auto/Manual 모드를 변경한다."""

import select
import sys
import termios
import tty

from action_msgs.srv import CancelGoal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int16


MODE_AUTO = 65       # ASCII 'A'
MODE_MANUAL = 77     # ASCII 'M'


class ModeKeyboardNode(Node):
    """키보드와 서버 모드 입력을 mode_cmd 토픽으로 전달하는 노드."""

    def __init__(self) -> None:
        super().__init__('mode_keyboard')
        self.mode_pub = self.create_publisher(
            Int16,
            'mode_cmd',
            QoSProfile(depth=1),
        )
        self.robot_mode_sub = self.create_subscription(
            Int16,
            '/server/robot_mode',
            self.robot_mode_callback,
            QoSProfile(depth=1),
        )
        self.navigation_cancel_clients = [
            self.create_client(
                CancelGoal,
                f'{action_name}/_action/cancel_goal',
            )
            for action_name in (
                '/navigate_to_pose',
                '/navigate_through_poses',
            )
        ]

    def robot_mode_callback(self, msg: Int16) -> None:
        if msg.data == 0:
            self.publish_mode(MODE_MANUAL)
            self.cancel_navigation_goals()
        elif msg.data == 1:
            self.publish_mode(MODE_AUTO)
        else:
            self.get_logger().warning(
                f'Invalid /server/robot_mode value: {msg.data} (expected 0 or 1)'
            )

    def cancel_navigation_goals(self) -> None:
        """Cancel all active Nav2 goals when server control enters Manual mode."""
        cancel_requested = False

        for client in self.navigation_cancel_clients:
            if not client.service_is_ready():
                continue

            # 비어 있는 GoalInfo는 ROS 2 action 규약상 모든 goal 취소를 뜻한다.
            future = client.call_async(CancelGoal.Request())
            future.add_done_callback(self.navigation_cancel_callback)
            cancel_requested = True

        if cancel_requested:
            self.get_logger().info(
                'Manual mode received: requested cancellation of Nav2 goals'
            )
        else:
            self.get_logger().warning(
                'Manual mode received, but Nav2 cancel services are unavailable'
            )

    def navigation_cancel_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # ROS service 통신 실패를 노드 종료와 분리한다.
            self.get_logger().error(f'Failed to cancel Nav2 goal: {error}')
            return

        if response.goals_canceling:
            self.get_logger().info(
                f'Canceling {len(response.goals_canceling)} Nav2 goal(s)'
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

    keyboard_enabled = sys.stdin.isatty()
    original_terminal_settings = None

    if keyboard_enabled:
        original_terminal_settings = termios.tcgetattr(sys.stdin)
        print(
            '\nMode keyboard control\n'
            '---------------------\n'
            'a : Auto mode\n'
            'm : Manual mode\n'
            'q : Manual mode and quit\n'
        )
    else:
        node.get_logger().info(
            'No terminal detected; /server/robot_mode topic control only'
        )

    try:
        if keyboard_enabled:
            tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            # 토픽 콜백을 처리하면서 최대 0.1초 동안 키 입력을 기다린다.
            rclpy.spin_once(node, timeout_sec=0.0 if keyboard_enabled else 0.1)

            if not keyboard_enabled:
                continue

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
        if original_terminal_settings is not None:
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
