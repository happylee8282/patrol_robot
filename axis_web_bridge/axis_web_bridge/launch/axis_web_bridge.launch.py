from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8766"),
        DeclareLaunchArgument("command_timeout", default_value="0.5"),
        Node(
            package="axis_web_bridge",
            executable="axis_web_bridge",
            name="axis_web_bridge",
            output="screen",
            parameters=[{
                "bind_host": LaunchConfiguration("bind_host"),
                "port": LaunchConfiguration("port"),
                "command_timeout": LaunchConfiguration("command_timeout"),
            }],
        ),
    ])
