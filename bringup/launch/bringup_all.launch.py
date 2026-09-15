"""Start the requested robot subsystems at two-second intervals."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(launch_directory, filename):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_directory / filename)
        )
    )


def generate_launch_description():
    """Start each subsystem two seconds after the previous one."""
    package_directory = Path(
        get_package_share_directory('bringup')
    )
    launch_directory = package_directory / 'launch'

    bringup = _include(
        launch_directory,
        'bringup.launch.py',
    )

    light_node = Node(
        package='bringup',
        executable='light_node',
        name='light_node',
        output='screen',
    )

    mode_keyboard = Node(
        package='bringup',
        executable='mode_keyboard',
        name='mode_keyboard',
        output='screen',
    )

    can_up = Node(
        package='can_communication',
        executable='can_up',
        name='can_up',
        output='screen',
    )

    battery_status_publisher = Node(
        package='can_communication',
        executable='battery_status_publisher',
        name='battery_status_publisher',
        output='screen',
    )

    robot_speaker = Node(
        package='bringup',
        executable='robot_speaker',
        name='robot_speaker',
        output='screen',
    )

    return LaunchDescription([
        bringup,
        TimerAction(period=2.0, actions=[light_node]),
        TimerAction(period=4.0, actions=[robot_speaker]),
        TimerAction(period=6.0, actions=[mode_keyboard]),
        TimerAction(period=8.0, actions=[can_up]),
        TimerAction(period=10.0, actions=[battery_status_publisher]),
    ])
