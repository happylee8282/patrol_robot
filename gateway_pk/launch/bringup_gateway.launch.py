"""Start the gateway, AXIS camera, and MJPEG bridge in that order."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    meta_gateway = Node(
        package='gateway_pk',
        executable='meta_gateway',
        name='meta_glass_gateway',
        output='screen',
    )

    bringup_directory = Path(get_package_share_directory('bringup'))
    bringup_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_directory / 'launch' / 'bringup_cam.launch.py')
        ),
    )

    axis_mjpeg_bridge = Node(
        package='gateway_pk',
        executable='axis_mjpeg_bridge',
        name='coastal_axis_mjpeg_bridge',
        output='screen',
    )

    return LaunchDescription([
        # 0 s: WebSocket/ROS gateway
        meta_gateway,
        # 3 s: AXIS camera and PTZ bringup
        TimerAction(period=3.0, actions=[bringup_camera]),
        # 6 s: HTTP MJPEG bridge (three seconds after camera bringup)
        TimerAction(period=6.0, actions=[axis_mjpeg_bridge]),
    ])
