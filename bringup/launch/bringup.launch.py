from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    uart = Node(
        package='bringup',
        executable='uart',
        output='screen',
    )

    wheel_cmd = Node(
        package='bringup',
        executable='wheel_cmd',
        output='screen',
    )

    return LaunchDescription([
        uart,
        wheel_cmd,
    ])
