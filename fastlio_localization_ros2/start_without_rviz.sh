#!/bin/bash

# FAST_LIO_LOCALIZATION2 启动脚本（无 RViz）
# 用于远程可视化场景：本机运行节点，远程显示

set -e

echo "=== FAST_LIO_LOCALIZATION2 启动（无 RViz）==="

# 1. 设置网络环境（与机器狗系统一致）
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

# 2. Source Livox driver
if [ -f "$HOME/ws_livox/install/setup.bash" ]; then
    echo "正在 source livox_ros_driver2..."
    source "$HOME/ws_livox/install/setup.bash"
else
    echo "警告: 未找到 ~/ws_livox/install/setup.bash"
fi

# 3. Source FAST_LIO_LOCALIZATION2
if [ -f "$HOME/FAST_LIO_LOCALIZATION2/install/setup.bash" ]; then
    echo "正在 source FAST_LIO_LOCALIZATION2..."
    source "$HOME/FAST_LIO_LOCALIZATION2/install/setup.bash"
else
    echo "错误: 未找到安装文件，请先构建: colcon build --symlink-install"
    exit 1
fi

# 4. 设置地图文件路径
MAP_FILE="${MAP_FILE:-$HOME/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd}"
CONFIG_FILE="${CONFIG_FILE:-mid360.yaml}"

if [ ! -f "$MAP_FILE" ]; then
    echo "错误: 地图文件不存在: $MAP_FILE"
    exit 1
fi

echo "地图文件: $MAP_FILE"
echo "配置文件: $CONFIG_FILE"
echo ""
echo "系统正在运行..."
echo "可通过以下方式监控："
echo "  - 查看话题: ros2 topic list"
echo "  - 查看里程计: ros2 topic echo /Odometry"
echo "  - 查看定位: ros2 topic echo /localization"
echo "  - 启动 RViz（其他机器）: rviz2 -d ~/fastlio_localization.rviz"
echo "  - 发布初始位姿: ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{\"header\":{\"frame_id\":\"map\"},\"pose\":{\"pose\":{\"position\":{\"x\":0.0,\"y\":0.0,\"z\":0.0},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}}'"
echo ""
echo "按 Ctrl+C 停止"
echo ""

# 5. 启动系统（无 RViz）
ros2 launch fast_lio_localization localization.launch.py \
    map:="$MAP_FILE" \
    config_file:="$CONFIG_FILE" \
    pcd_map_topic:=/cloud_pcd \
    rviz:=false
