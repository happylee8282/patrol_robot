import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FAST_LIO_CONFIG_DIR = '/home/unicon/nav_ws/src/fastlio_localization_ros2/config'
FAST_LIO_CONFIG_FILE = 'mid360.yaml'


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    config_path = os.path.join(FAST_LIO_CONFIG_DIR, FAST_LIO_CONFIG_FILE)
    rviz_config = os.path.join(
        get_package_share_directory('bringup'), 'config', 'rviz', 'localization.rviz'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('livox_ros_driver2'),
            '/launch_ROS2/msg_MID360_launch.py',
        ]),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('myahrs_ros2_driver'),
            '/launch/myahrs_ros2_driver.launch.py',
        ]),
        launch_arguments={'use_rviz': 'false'}.items(),
    )

    tf_link = Node(
        package='bringup',
        executable='tf_link',
        name='tf_link',
        output='screen',
        parameters=[{'fast_lio_config': config_path, 'sensor_height': 0.72}],
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('fast_lio_localization'),
            '/launch/localization.launch.py',
        ]),
        launch_arguments={
            'map': '/home/unicon/nav_ws/src/bringup/map/3d/new_file.pcd',
            'pcd_map_topic': '/cloud_pcd',
            'config_path': FAST_LIO_CONFIG_DIR,
            'config_file': FAST_LIO_CONFIG_FILE,
            'use_sim_time': 'false',
            'rviz': rviz,
            'rviz_cfg': rviz_config,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        lidar_launch,
        imu_launch,
        tf_link,
        localization_launch,
    ])
