#!/bin/bash

# FAST_LIO_LOCALIZATION2 快速启动脚本
# 自动 source 环境并启动系统（带 RViz）

set -e

echo "=== FAST_LIO_LOCALIZATION2 启动 ==="

# 1. 设置网络域（与机器狗系统一致）
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo ""

# 2. Source Livox driver
if [ -f "$HOME/ws_livox/install/setup.bash" ]; then
    echo "正在 source livox_ros_driver2..."
    source "$HOME/ws_livox/install/setup.bash"
else
    echo "警告: 未找到 ~/ws_livox/install/setup.bash"
fi

# 2. Source FAST_LIO_LOCALIZATION2
if [ -f "$HOME/FAST_LIO_LOCALIZATION2/install/setup.bash" ]; then
    echo "正在 source FAST_LIO_LOCALIZATION2..."
    source "$HOME/FAST_LIO_LOCALIZATION2/install/setup.bash"
else
    echo "错误: 未找到安装文件，请先构建: colcon build --symlink-install"
    exit 1
fi

# 3. 启动系统
MAP_FILE="$HOME/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd"

if [ ! -f "$MAP_FILE" ]; then
    echo "错误: 地图文件不存在: $MAP_FILE"
    exit 1
fi

echo "地图文件: $MAP_FILE"
echo "启动系统（带 RViz）..."
echo "请在 RViz2 中使用 '2D Pose Estimate' 工具提供初始位姿"
echo ""

ros2 launch fast_lio_localization localization.launch.py \
    map:="$MAP_FILE" \
    config_file:=mid360.yaml \
    pcd_map_topic:=/cloud_pcd \
    rviz:=true
