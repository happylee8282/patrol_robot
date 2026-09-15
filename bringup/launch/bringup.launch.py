import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
