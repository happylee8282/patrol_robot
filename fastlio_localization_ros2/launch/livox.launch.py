"""
Livox MID360 LiDAR ROS2驱动启动文件

该启动文件配置并启动Livox LiDAR驱动(MID360型号)的ROS2节点。
提供灵活的配置，适应不同的安装朝向和数据输出格式。

Livox驱动发布的话题：
- /livox/lidar 或 /livox/inverted_lidar：PointCloud2类型的LiDAR扫描数据
- /livox/imu 或 /livox/inverted_imu：sensor_msgs/Imu类型的IMU数据

主要功能：
- 支持倒置安装配置(LiDAR上下颠倒)
- 可配置数据传输格式(PointCloud2 vs 自定义格式)
- 可选RViz可视化，用于实时点云监控
- 通过JSON配置文件处理MID360 LiDAR特定配置

启动参数：
    inverted (bool)：LiDAR是否倒置安装
    xfer_format (int)：数据传输格式(0=PointCloud2, 1=自定义点云格式)
    rviz (bool)：是否启动RViz进行可视化

启动的节点：
    1. livox_ros_driver2_node：MID360 LiDAR主驱动节点
    2. invert_livox_scan：当LiDAR倒置时用于校正点云的节点
    3. rviz2：可视化工具(可选)

使用方法：
    ros2 launch fast_lio_localization livox.launch.py [inverted:=true] [rviz:=true]
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


################### 用户配置参数 ROS2 开始 ###################
# 这些参数控制Livox驱动的行为，请根据实际设置调整。

# multi_topic：多LiDAR输出模式
#   0 = 所有LiDAR共享同一话题(单个点云流)
#   1 = 每个LiDAR发布到独立话题(多个流)
multi_topic   = 0    

# data_src：数据源选择器
#   0 = 物理LiDAR硬件(MID360)
#   其他值 = 无效/测试数据源
data_src      = 0    

# publish_freq：发布频率，单位Hz
# 支持的频率：5.0, 10.0(默认), 20.0, 50.0 等
# 较高频率会增加带宽但可能影响系统性能
publish_freq  = 10.0 

# output_type：输出数据类型
#   0 = PointCloud2 (ROS标准格式)
#   1 = 自定义点云格式(Livox专用)
output_type   = 0    

# frame_id：点云消息的ROS坐标系标识符
# 该坐标系用于坐标变换(TF)计算和可视化
# 必须与LiDAR的实际安装坐标系一致
frame_id      = 'livox_frame'  

# lvx_file_path：LVX录制文件的路径，用于回放
# 当回放记录的LiDAR数据而非实时传感器输入时使用
# 实时LiDAR操作时设为空字符串("")
lvx_file_path = '/home/livox/livox_test.lvx'  

# cmdline_bd_code：Livox设备标识的唯一板载代码
# 应匹配实际硬件的广播码
# 格式：'livox' + 10个十六进制字符
cmdline_bd_code = 'livox0000000001'

# package_path：Livox驱动包安装的文件系统路径
# 通常由ament索引自动设置，但可以手动覆盖
# 警告：硬编码路径可能不适用于所有系统
package_path = "/home/wheelchair2/livox_ws/src/livox_ros_driver2"   # 请替换为您的实际路径

# 构建配置文件路径
cur_config_path = package_path + '/config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
# 用于可视化点云的RViz配置文件
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
################### 用户配置参数 ROS2 结束 #####################


def generate_launch_description():
    """
    生成Livox驱动系统的启动描述。
    
    该函数创建包含MID360 LiDAR驱动所有节点和配置的LaunchDescription，
    支持可选的倒置处理和可视化功能。
    
    返回值：
        LaunchDescription：包含所有操作的ROS2启动描述对象
        
    启动配置变量：
        inverted (LaunchConfiguration)：倒置安装布尔值
        xfer_format (LaunchConfiguration)：数据格式选择(0或1)
        rviz (LaunchConfiguration)：是否启用RViz
    """
    
    # ========== 启动参数声明 ==========
    
    # inverted：指定LiDAR是否倒置安装(上下颠倒)
    # 当为true时，使用倒置话题重映射并激活倒置校正节点
    inverted = LaunchConfiguration("inverted")
    declare_inverted = DeclareLaunchArgument(
        "inverted", 
        default_value="true", 
        description="指定LiDAR是否倒置安装"
    )
    
    # xfer_format：选择点云消息格式
    #   0 = sensor_msgs/PointCloud2 (标准ROS格式，兼容性最好)
    #   1 = livox_ros_driver2/CustomPointCloud (Livox专用格式，包含额外字段)
    xfer_format = LaunchConfiguration("xfer_format")
    declare_xfer_format = DeclareLaunchArgument(
        "xfer_format", 
        default_value="1", 
        description="声明Livox消息格式(0=PointCloud2, 1=自定义格式)"
    )
    
    # rviz：控制是否启动RViz2可视化
    rviz = LaunchConfiguration("rviz")
    declare_rviz = DeclareLaunchArgument(
        "rviz", 
        default_value="false", 
        description="启动rviz以显示点云"
    )
    
    # ========== LIVOX驱动参数 ==========
    
    # 传递给livox_ros_driver2_node的参数
    # 这些参数配置驱动的行为和输出
    livox_ros2_params = [
        {"xfer_format": xfer_format},          # 消息格式选择
        {"multi_topic": multi_topic},          # 多LiDAR话题模式
        {"data_src": data_src},                # 数据源(0=硬件)
        {"publish_freq": publish_freq},        # 发布频率(Hz)
        {"output_data_type": output_type},     # 输出数据类型
        {"frame_id": frame_id},                # TF坐标系
        {"lvx_file_path": lvx_file_path},      # LVX回放文件路径
        {"user_config_path": user_config_path},# MID360 JSON配置文件
        {"cmdline_input_bd_code": cmdline_bd_code}  # 设备板载代码
    ]
    
    # ========== 节点定义 ==========
    
    # livox_driver：主Livox驱动节点
    # 在标准话题上发布点云和IMU数据
    # 条件：仅在LiDAR不倒置时运行(正常安装方向)
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=UnlessCondition(inverted),  # 仅在 inverted == false 时
    )
    
    # livox_driver_remap：带话题重映射的驱动节点
    # 条件：仅在LiDAR倒置时运行(上下颠倒安装)
    # 重映射话题以指示倒置朝向，供下游处理使用
    livox_driver_remap = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        remappings=[
            ("/livox/lidar", "/livox/inverted_lidar"),   # 重映射LiDAR话题
            ("/livox/imu", "/livox/inverted_imu")         # 重映射IMU话题
        ],
        condition=IfCondition(inverted)  # 仅在 inverted == true 时
    )
    
    # invert_lidar_node：校正倒置点云的自定义节点
    # 当LiDAR上下颠倒安装时，该节点翻转Z轴
    # 以恢复SLAM/定位算法所需的正确朝向
    # 订阅：/livox/inverted_lidar (PointCloud2)
    # 发布：/livox/lidar (校正后的PointCloud2)
    invert_lidar_node = Node(
        package='fast_lio_localization',
        executable='invert_livox_scan.py',
        name='invert_livox_scan',
        output='screen',
        parameters=[{"xfer_format": xfer_format}],
        condition=IfCondition(inverted),  # 仅在倒置时激活
    )
    
    # livox_rviz：RViz2可视化节点
    # 实时显示点云、坐标变换和传感器数据
    # 使用预定义的RViz配置以获得最佳的Livox可视化效果
    livox_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path],
        condition=IfCondition(rviz),  # 仅在 rviz == true 时
    )

    # ========== 启动描述组装 ==========
    
    # 创建并返回包含所有节点和声明参数的LaunchDescription
    # 顺序很重要：参数必须在使用前声明
    return LaunchDescription([
        declare_inverted,
        declare_xfer_format,
        declare_rviz,
        livox_driver,
        livox_driver_remap,
        invert_lidar_node,
        livox_rviz,
    ])