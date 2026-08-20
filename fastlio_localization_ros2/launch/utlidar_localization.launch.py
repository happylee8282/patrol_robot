"""
FAST-LIO定位系统启动文件（支持C++和Python节点）

通过参数选择使用C++或Python节点:
- use_cpp_nodes:=true  使用C++节点（默认，高性能）
- use_cpp_nodes:=false 使用Python节点（原始版本）

使用方法:
    # 使用高性能C++节点（推荐）
    ros2 launch fast_lio_localization utlidar_localization.launch.py \
        map:=/home/unitree/go2_patrol/map/test.pcd

    # 使用Python节点（原始版本）
    ros2 launch fast_lio_localization utlidar_localization.launch.py  \
        map:=/path/to/map.pcd use_cpp_nodes:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成C++版本的FAST-LIO定位系统"""

    package_path = get_package_share_directory("fast_lio_localization")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")

    # 声明启动参数
    use_cpp_nodes = LaunchConfiguration("use_cpp_nodes")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")

    declare_use_cpp_nodes = DeclareLaunchArgument(
        "use_cpp_nodes",
        default_value="true",
        description="使用C++节点 (true) 或 Python节点 (false)"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时钟"
    )

    declare_config_path = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="配置文件目录"
    )

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value="unilidar_l1.yaml",
        description="配置文件名"
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="启用RViz可视化"
    )

    declare_rviz_cfg = DeclareLaunchArgument(
        "rviz_cfg",
        default_value=default_rviz_config_path,
        description="RViz配置文件"
    )

    declare_map_path = DeclareLaunchArgument(
        "map",
        default_value="",
        description="PCD地图文件路径"
    )

    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic",
        default_value="/cloud_pcd",
        description="PCD地图话题名"
    )

    # ========== 节点定义 ==========

    # fast_lio_node (C++, 已存在)
    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    # ========== transform_fusion 节点 (C++ vs Python) ==========

    # C++版本 transform_fusion
    transform_fusion_cpp = Node(
        package="fast_lio_localization",
        executable="transform_fusion",  # C++可执行文件
        name="transform_fusion",
        output="screen",
        condition=IfCondition(use_cpp_nodes),  # use_cpp_nodes=true 时使用
    )

    # Python版本 transform_fusion
    transform_fusion_python = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",  # Python脚本
        name="transform_fusion",
        output="screen",
        condition=UnlessCondition(use_cpp_nodes),  # use_cpp_nodes=false 时使用
    )

    # ========== global_localization 节点 (C++ vs Python) ==========

    # C++版本 global_localization
    global_localization_cpp = Node(
        package="fast_lio_localization",
        executable="global_localization",  # C++可执行文件
        name="global_localization",
        output="screen",
        parameters=[
            {
                "map_voxel_size": 0.4,
                "scan_voxel_size": 0.1,
                "freq_localization": 0.5,
                "freq_global_map": 0.25,
                "localization_threshold": 0.8,
                "fov": 6.28319,
                "fov_far": 300.0,  # 改为浮点数类型
                "pcd_map_path": pcd_map_path,
                "pcd_map_topic": pcd_map_topic,
            }
        ],
        condition=IfCondition(use_cpp_nodes),  # use_cpp_nodes=true 时使用
    )

    # Python版本 global_localization
    global_localization_python = Node(
        package="fast_lio_localization",
        executable="global_localization.py",  # Python脚本
        name="global_localization",
        output="screen",
        parameters=[
            {
                "map_voxel_size": 0.4,
                "scan_voxel_size": 0.1,
                "freq_localization": 0.5,
                "freq_global_map": 0.25,
                "localization_threshold": 0.8,
                "fov": 6.28319,
                "fov_far": 300.0,  # 改为浮点数类型
                "pcd_map_path": pcd_map_path,
                "pcd_map_topic": pcd_map_topic,
            }
        ],
        condition=UnlessCondition(use_cpp_nodes),  # use_cpp_nodes=false 时使用
    )

    # PCD地图发布器（两者共享）
    pcd_publisher_node = Node(
        package="fast_lio_localization",
        executable="pcd_to_pointclouds.py",
        name="map_publisher",
        output="screen",
        parameters=[
            {
                "file_name": pcd_map_path,
                "tf_frame": "map",
                "cloud_topic": pcd_map_topic,
                "period_ms_": 500,
            }
        ],
        remappings=[("cloud_pcd", pcd_map_topic)]
    )

    # RViz可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # 传感器矫正节点
    transform_node = Node(
        package='transform_sensors',
        executable='transform_utlidar_onboard_imu',
        name='transform_utlidar_onboard_imu',
        output='screen',
        parameters=[
            {
                # 'use_gpu': True,
                'use_gpu': False,
                'num_points': 150
            }
        ]
    )




    # ========== 组装LaunchDescription ==========

    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_cpp_nodes)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_config_path)
    ld.add_action(declare_config_file)
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_cfg)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_topic)

    # 添加节点
    ld.add_action(fast_lio_node)
    ld.add_action(transform_fusion_cpp)      # C++版本
    ld.add_action(transform_fusion_python)   # Python版本
    ld.add_action(global_localization_cpp)   # C++版本
    ld.add_action(global_localization_python)  # Python版本
    # ld.add_action(pcd_publisher_node)
    ld.add_action(rviz_node)
    
    
    ld.add_action(transform_node)

    return ld
