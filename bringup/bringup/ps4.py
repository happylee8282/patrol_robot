#!/usr/bin/env python3
"""Latch PS4 direction inputs and publish a fixed velocity until A is pressed."""

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16


class Ps4LatchedControl(Node):
    """Convert a stick direction into a continuously published fixed command."""

    def __init__(self) -> None:
        super().__init__('ps4_latched_control')

        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('auto_button', 0)
        self.declare_parameter('manual_button', 1)
        self.declare_parameter('stop_button', 2)
        self.declare_parameter('axis_threshold', 0.6)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'navigation_actions',
            ['/navigate_to_pose', '/navigate_through_poses'],
        )

        self.linear_axis = int(self.get_parameter('linear_axis').value)
        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.auto_button = int(self.get_parameter('auto_button').value)
        self.manual_button = int(self.get_parameter('manual_button').value)
        self.stop_button = int(self.get_parameter('stop_button').value)
        self.axis_threshold = float(
            self.get_parameter('axis_threshold').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value))
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.robot_mode_pub = self.create_publisher(
            Int16, '/server/robot_mode', 10)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.cancel_clients = [
            self.create_client(CancelGoal, f'{name}/_action/cancel_goal')
            for name in self.get_parameter('navigation_actions').value
        ]

        self.command = Twist()
        self.active = False
        self.previous_buttons = {
            'auto': False,
            'manual': False,
            'stop': False,
        }
        self.last_direction = 'stop'
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

        self.get_logger().info(
            'PS4 control ready: A=Auto, B=Manual, X=Stop')

    @staticmethod
    def _axis(msg: Joy, index: int) -> float:
        return float(msg.axes[index]) if 0 <= index < len(msg.axes) else 0.0

    def joy_callback(self, msg: Joy) -> None:
        auto_pressed = self._button(msg, self.auto_button)
        manual_pressed = self._button(msg, self.manual_button)
        stop_pressed = self._button(msg, self.stop_button)

        # 버튼을 누르고 있는 동안 반복 발행하지 않고, 눌린 순간 한 번만 처리한다.
        if auto_pressed and not self.previous_buttons['auto']:
            self.stop(log=False)
            self.publish_robot_mode(1, 'Auto')
        if manual_pressed and not self.previous_buttons['manual']:
            self.publish_robot_mode(0, 'Manual')
        if stop_pressed and not self.previous_buttons['stop']:
            self.stop()

        self.previous_buttons['auto'] = auto_pressed
        self.previous_buttons['manual'] = manual_pressed
        self.previous_buttons['stop'] = stop_pressed
        if auto_pressed or manual_pressed or stop_pressed:
            return

        linear = self._axis(msg, self.linear_axis)
        angular = self._axis(msg, self.angular_axis)

        # 대각 입력은 값이 더 큰 축 하나만 사용한다.
        if max(abs(linear), abs(angular)) < self.axis_threshold:
            return

        command = Twist()
        if abs(linear) >= abs(angular):
            command.linear.x = self.linear_speed if linear > 0.0 else -self.linear_speed
            direction = 'forward' if linear > 0.0 else 'backward'
        else:
            command.angular.z = self.angular_speed if angular > 0.0 else -self.angular_speed
            direction = 'left' if angular > 0.0 else 'right'

        if direction == self.last_direction and self.active:
            return

        self.command = command
        self.active = True
        self.last_direction = direction
        self.cancel_navigation()
        self.cmd_pub.publish(self.command)
        self.get_logger().info(f'Latched command: {direction}')

    @staticmethod
    def _button(msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    def publish_robot_mode(self, value: int, name: str) -> None:
        message = Int16()
        message.data = value
        self.robot_mode_pub.publish(message)
        self.get_logger().info(
            f'{name} button: /server/robot_mode = {value}')

    def cancel_navigation(self) -> None:
        """Cancel every active Nav2 goal so it cannot fight manual cmd_vel."""
        for client in self.cancel_clients:
            if not client.service_is_ready():
                continue
            client.call_async(CancelGoal.Request())

    def stop(self, log: bool = True) -> None:
        self.command = Twist()
        self.active = False
        self.last_direction = 'stop'
        # 정지 명령을 즉시 여러 번 보내 다른 cmd_vel이 남을 가능성을 줄인다.
        for _ in range(3):
            self.cmd_pub.publish(self.command)
        if log:
            self.get_logger().info('STOP: X button pressed')

    def publish_command(self) -> None:
        if self.active:
            self.cmd_pub.publish(self.command)

    def destroy_node(self) -> bool:
        self.stop(log=False)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ps4LatchedControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
