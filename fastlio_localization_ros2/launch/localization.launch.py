"""
FAST-LIO定位系统完整启动文件

该启动文件协调完整的FAST-LIO(Fast Tightly-Coupled LiDAR Inertial Odometry)
定位管道。它启动所有必要的节点，用于基于LiDAR数据和可选先验地图的实时定位。

系统架构概览：
    1. fastlio_mapping：核心FAST-LIO算法，执行扫描到地图的配准
    2. global_localization：使用全局点云地图进行初始位姿估计
    3. transform_fusion：融合多个源的坐标变换，提供鲁棒的位姿
    4. pcd_publisher：将先验地图发布为PointCloud2，用于定位
    5. rviz2：点云、轨迹和定位状态的可视化

使用的ROS话题：
    输入：
        /livox/lidar (或 /livox/inverted_lidar)：sensor_msgs/PointCloud2
        /livox/imu：sensor_msgs/Imu
    
    输出：
        /Odometry：nav_msgs/Odometry - 带协方差的估计位姿
        /lio_sam/mapping/cloud_registered：sensor_msgs/PointCloud2 - 配准后的扫描
        /map：sensor_msgs/PointCloud2 - 用于定位的先验地图

启动参数：
    use_sim_time (bool)：使用仿真时钟(Gazebo/ROS2 bag回放时)
    config_path (str)：YAML配置文件目录
    config_file (str)：具体配置文件(如mid360.yaml)
    rviz (bool)：启用/禁用RViz可视化
    rviz_cfg (str)：RViz配置文件路径
    map (str)：先验PCD地图文件路径
    pcd_map_topic (str)：发布先验地图的ROS话题名称

使用方法：
    ros2 launch fast_lio_localization localization.launch.py \\
        map:=/path/to/map.pcd \\
        config_file:=mid360.yaml \\
        rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """
    生成FAST-LIO定位系统的完整启动描述。
    
    配置所有节点的启动，包括正确的参数设置、依赖关系
    和基于用户参数的conditional launching(条件启动)。
    
    返回值：
        LaunchDescription：完全配置的启动描述
    """
    
    # ========== 包路径和默认值 ==========
    
    # 获取已安装包的共享目录路径
    # 使用ament索引找到包的安装位置
    package_path = get_package_share_directory("fast_lio_localization")
    
    # 默认配置目录，包含YAML参数文件
    default_config_path = os.path.join(package_path, "config")
    
    # FAST-LIO可视化的默认RViz配置文件
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")

    # ========== 启动配置 ==========
    
    # 这些是可通过启动参数覆盖的动态值
    use_sim_time = LaunchConfiguration("use_sim_time")         # 仿真时钟标志
    config_path = LaunchConfiguration("config_path")           # 配置目录路径
    config_file = LaunchConfiguration("config_file")           # 具体配置文件名
    rviz_use = LaunchConfiguration("rviz")                     # 启用RViz布尔值
    rviz_cfg = LaunchConfiguration("rviz_cfg")                 # RViz配置文件路径
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")       # 先验地图话题
    pcd_map_path = LaunchConfiguration("map")                  # 先验PCD地图文件路径

    # ========== 启动参数声明 ==========
    
    # use_sim_time：启用仿真时钟而非系统时钟
    # 与Gazebo、录制的rosbag或仿真环境配合时设为true
    # 真实机器人实时传感器操作时设为false
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="false", 
        description="使用仿真(Gazebo)时钟，如果为true"
    )
    
    # config_path：包含YAML配置文件的目录
    # 包含传感器特定参数、算法设置和坐标变换
    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path", 
        default_value=default_config_path, 
        description="YAML配置文件目录路径"
    )
    
    # config_file：具体的YAML配置文件名
    # 不同LiDAR模型对应不同文件(如mid360.yaml, avia.yaml)
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", 
        default_value="mid360.yaml", 
        description="传感器和算法参数的配置文件"
    )
    
    # rviz：启用/禁用RViz2可视化
    # 为true时启动RViz2并加载FAST-LIO特定可视化配置
    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", 
        default_value="true", 
        description="使用RViz监控结果"
    )
    
    # rviz_cfg：RViz2配置文件路径
    # 包含点云、路径和坐标系的显示设置
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", 
        default_value=default_rviz_config_path, 
        description="RViz配置文件路径"
    )
    
    # map：先验点云地图文件路径(PCD格式)
    # 由global_localization节点用于初始位姿估计
    # 留空("")则跳过全局定位，仅使用里程计
    declare_map_path = DeclareLaunchArgument(
        "map", 
        default_value="/home/unicon/nav_ws/src/bringup/map/3d/0810.pcd", 
        description="用于全局定位的PCD地图文件路径"
    )
    
    # pcd_map_topic：发布先验地图的ROS话题名称
    # 其他节点订阅该话题接收地图数据
    # 标准话题为"/map"，但可以自定义
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", 
        default_value="/cloud_pcd", 
        description="发布PCD地图的话题"
    )

    # ========== 节点定义 ==========
    
    # fast_lio_node：核心FAST-LIO建图节点
    # 使用迭代误差状态卡尔曼滤波器执行LiDAR-惯性里程计
    # 
    # 订阅：
    #   /livox/lidar (sensor_msgs/PointCloud2) - LiDAR点云扫描
    #   /livox/imu (sensor_msgs/Imu) - IMU测量数据
    #   /initialpose (geometry_msgs/PoseWithCovarianceStamped) - 初始位姿估计
    # 
    # 发布：
    #   /Odometry (nav_msgs/Odometry) - 估计的位姿和速度
    #   /lio_sam/mapping/cloud_registered (sensor_msgs/PointCloud2) - 配准后的点云
    #   /lio_sam/mapping/keyframe_cloud (sensor_msgs/PointCloud2) - 关键帧地图
    #   /lio_sam/mapping/trajectory (nav_msgs/Path) - 完整轨迹
    # 
    # 参数从YAML文件加载 + use_sim_time覆盖
    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),  # YAML配置文件
            {"use_sim_time": use_sim_time}                     # 覆盖仿真时间
        ],
        # Structure B: keep FAST-LIO's odometry origin fixed.  RViz
        # /initialpose is consumed only by global_localization.
        remappings=[
            ("/initialpose", "/fastlio_initialpose_disabled"),
        ],
        output="screen",
    )
    
    # global_localization_node：使用先验地图的初始位姿估计
    # 将当前扫描与存储的点云地图匹配以估计全局位姿
    # 使用基于体素的匹配以提高效率
    # 
    # 订阅：
    #   /livox/lidar (sensor_msgs/PointCloud2) - 当前扫描
    # 
    # 发布：
    #   /initialpose (geometry_msgs/PoseWithCovarianceStamped) - 初始位姿估计
    # 
    # 参数：
    #   map_voxel_size (0.4)：下采样先验地图的体素大小(米)
    #   scan_voxel_size (0.1)：下采样当前扫描的体素大小(米)
    #   freq_localization (0.5)：定位更新频率(Hz)
    #   freq_global_map (0.25)：全局地图更新频率(Hz)
    #   localization_threshold (0.8)：成功匹配的置信度阈值
    #   fov (6.28319)：视场角，单位弧度(2π = 全向)
    #   fov_far (300)：视场最大范围(米)
    #   pcd_map_path：先验PCD地图文件路径
    #   pcd_map_topic：地图订阅的话题名称
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization",
        name="global_localization",
        output="screen",
        parameters=[
            {
                "map_voxel_size": 0.4,          # 地图下采样分辨率
                "scan_voxel_size": 0.2,         # 扫描下采样分辨率
                "freq_localization": 0.5,       # 定位更新频率 happy - 0.5hz
                "freq_global_map": 0.25,        # 全局地图更新频率
                "localization_threshold": 0.8,  # 匹配置信度阈值
                "fov": 6.28319,                 # 视场角(360度，弧度制)
                "fov_far": 300.0,               # 最大感知范围
                "pcd_map_path": pcd_map_path,   # 先验地图文件路径
                "pcd_map_topic": pcd_map_topic, # 地图订阅话题
                "icp_coarse_max_corr_dist": 1.5,
                "icp_fine_max_corr_dist": 0.3,
                "icp_rmse_threshold": 0.2,
            }
        ],
    )

    # transform_fusion_node：坐标变换融合
    # 融合来自多个源的位姿变换
    # 通过融合里程计、IMU和地图数据提供鲁棒的位姿估计
    # 
    # 订阅：
    #   /Odometry (nav_msgs/Odometry) - LiDAR里程计
    #   /imu/data (sensor_msgs/Imu) - 原始IMU数据
    # 
    # 发布：
    #   /pose_with_covariance (geometry_msgs/PoseWithCovarianceStamped) - 融合后的位姿
    # 
    # 无需参数；使用配置文件中的默认值
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion",
        name="transform_fusion",
        output="screen",
    )

    # pcd_publisher_node：PCD地图到PointCloud2的发布器
    # 从PCD文件加载先验地图，并重新发布为ROS PointCloud2格式
    # 向global_localization节点和可视化提供地图数据
    # 
    # 订阅：无(独立发布器)
    # 
    # 发布：
    #   /map (sensor_msgs/PointCloud2) - 先验地图点云
    # 
    # 参数：
    #   file_name：PCD地图文件路径
    #   tf_frame：地图点的坐标系(默认："map")
    #   cloud_topic：地图发布的ROS话题名称
    #   period_ms_：发布周期，单位毫秒(500ms = 2Hz)
    pcd_publisher_node = Node(
        package="pcl_ros",                         # 标准PCL ROS包装器
        executable="pcd_to_pointcloud",            # PCD到PointCloud2转换器
        name="map_publisher",
        output="screen",
        parameters=[
            {
                "file_name": pcd_map_path,         # 输入PCD文件
                "tf_frame": "map",                 # 地图固定坐标系
                "cloud_topic": pcd_map_topic,      # 输出话题名称
                "period_ms_": 500,                 # 发布周期(2 Hz)
            }
        ],
        remappings=[
            ("cloud_pcd", pcd_map_topic),  # 节点接口内部重映射
        ]
    )

    # rviz_node：RViz2可视化
    # 显示点云、机器人位姿、轨迹和TF变换
    # 条件：仅当设置 rviz:=true 参数时启动
    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        arguments=["-d", rviz_cfg], 
        condition=IfCondition(rviz_use)
    )

    # ========== 启动描述组装 ==========
    
    # 创建LaunchDescription并按依赖顺序添加操作：
    # 1. 声明启动参数(必须首先添加)
    # 2. 添加节点(顺序影响启动时序但不影响依赖关系)
    ld = LaunchDescription()
    
    # 添加启动参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_topic)

    # Structure B: FAST-LIO publishes local odometry, global_localization
    # performs prior-PCD ICP, and transform_fusion publishes map -> camera_init.
    ld.add_action(fast_lio_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(pcd_publisher_node)
    ld.add_action(rviz_node)

    return ld
