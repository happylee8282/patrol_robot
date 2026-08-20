from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('livox_ros_driver2'),
            '/launch_ROS2/msg_MID360_launch.py',
        ])
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('fast_lio_localization'),
            '/launch/localization.launch.py',
        ]),
        launch_arguments={
            'map': '/home/unicon/nav_ws/src/bringup/map/3d/0810.pcd',
            'pcd_map_topic': '/cloud_pcd',
            'config_path': (
                '/home/unicon/nav_ws/src/fastlio_localization_ros2/config'
            ),
            'config_file': 'mid360.yaml',
            'use_sim_time': 'false',
            'rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        livox_launch,
        localization_launch,
    ])
