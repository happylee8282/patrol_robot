"""Configure CAN0, wait three seconds, then start the battery publisher."""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    can_up = Node(
        package='can_communication',
        executable='can_up',
        name='can_up',
        output='screen',
    )

    battery_status_publisher = Node(
        package='can_communication',
        executable='battery_status_publisher',
        name='battery_can_publisher',
        output='screen',
    )

    start_publisher_after_can = RegisterEventHandler(
        OnProcessExit(
            target_action=can_up,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[battery_status_publisher],
                ),
            ],
        )
    )

    return LaunchDescription([
        start_publisher_after_can,
        can_up,
    ])
