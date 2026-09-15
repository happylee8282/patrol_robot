from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    yaml_file = str(
        Path(get_package_share_directory('bringup'))
        / 'config' / 'ps4' / 'ps4_teleop.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[yaml_file]
    )

    ps4_control_node = Node(
        package='bringup',
        executable='ps4_control',
        name='ps4_latched_control',
        output='screen',
        parameters=[yaml_file]
    )

    return LaunchDescription([
        joy_node,
        ps4_control_node,
    ])
