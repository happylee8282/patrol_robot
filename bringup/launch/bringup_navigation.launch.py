import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    # The odom coordinates are the FAST-LIO camera_init coordinates.  This
    # connects Nav2 to the existing map -> camera_init -> body -> base_link
    # chain without giving base_link a second TF parent.
    odom_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_odom',
        arguments=['--frame-id', 'camera_init', '--child-frame-id', 'odom'],
    )

    nav2_tf_2d = Node(
        package='bringup',
        executable='nav2_tf_2d',
        name='nav2_tf_2d',
        output='screen',
    )

    obstacle_filter = Node(
        package='slope_cloud_filter',
        executable='slope_cloud_filter',
        name='slope_cloud_filter',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('slope_cloud_filter'),
            'config', 'slope_filter.yaml',
        )],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': (
                '/home/unicon/nav_ws/src/bringup/map/new_file.yaml'
            ),
            'use_sim_time': False,
        }],
    )

    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': False,
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py',
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=(
                '/home/unicon/nav_ws/src/bringup/config/'
                'nav2_pcd_params_teb.yaml'
            ),
            description='Nav2 parameters file (TEB by default)',
        ),
        odom_frame,
        nav2_tf_2d,
        obstacle_filter,
        map_server,
        map_server_lifecycle_manager,
        navigation,
    ])
