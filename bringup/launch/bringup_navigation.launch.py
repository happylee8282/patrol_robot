from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_tf_2d = Node(
        package='bringup',
        executable='nav2_tf_2d',
        name='nav2_tf_2d',
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': (
                '/home/unicon/nav_ws/src/bringup/map/2d/0810_0.5m.yaml'
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
            'params_file': (
                '/home/unicon/nav_ws/src/bringup/config/'
                'nav2_pcd_params.yaml'
            ),
        }.items(),
    )

    return LaunchDescription([
        nav2_tf_2d,
        map_server,
        map_server_lifecycle_manager,
        navigation,
    ])
