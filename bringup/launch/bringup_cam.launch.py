"""Bring up an AXIS camera and optionally its OpenCV RGB viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hostname = LaunchConfiguration('hostname')
    view = LaunchConfiguration('view')

    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.11.90',
        description='AXIS camera IPv4 address',
    )
    view_arg = DeclareLaunchArgument(
        'view',
        default_value='false',
        description='Launch the OpenCV RGB viewer (true/false)',
    )

    axis_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('axis_camera_ros2'),
                'launch',
                'axis_camera.launch.py',
            )
        ),
        launch_arguments={
            'hostname': hostname,
        }.items(),
    )

    rgb_viewer = Node(
        package='axis_image_processor',
        executable='rgb_detector',
        name='rgb_detector',
        output='screen',
        condition=IfCondition(view),
    )

    return LaunchDescription([
        hostname_arg,
        view_arg,
        axis_camera_launch,
        rgb_viewer,
    ])
