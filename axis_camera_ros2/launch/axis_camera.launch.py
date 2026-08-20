from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.0.90',
        description='AXIS camera IPv4 address'
    )

    command_timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value='0.5',
        description='PTZ watchdog timeout in seconds'
    )

    hostname = LaunchConfiguration('hostname')
    command_timeout = LaunchConfiguration('command_timeout')

    camera_node = Node(
        package='axis_camera_ros2',
        executable='axis_camera_node',
        name='axis_camera_node',
        output='screen',
        parameters=[
            {
                'hostname': hostname,
                'mjpeg_path': '/mjpg/video.mjpg',
                'frame_id': 'axis_camera',
                'topic_name': '/axis/image_raw/compressed',
            }
        ]
    )

    ptz_node = Node(
        package='axis_camera_ros2',
        executable='axis_ptz_node',
        name='axis_ptz_node',
        output='screen',
        parameters=[
            {
                'hostname': hostname,
                'command_timeout': command_timeout,
            }
        ]
    )

    return LaunchDescription([
        hostname_arg,
        command_timeout_arg,
        camera_node,
        ptz_node,
    ])
