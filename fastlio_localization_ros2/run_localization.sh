#!/bin/bash

# FAST_LIO_LOCALIZATION2 一键启动脚本
# 使用方法: source setup_env.sh && ./run_localization.sh

set -e  # 遇到错误立即退出

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== FAST_LIO_LOCALIZATION2 启动脚本 ===${NC}"

# 1. 检查并 source 环境
echo -e "${GREEN}[1/5] 检查 ROS2 环境...${NC}"

if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2 环境未设置${NC}"
    echo "请先 source ROS2 安装目录下的 setup.bash"
    exit 1
fi

# Source Livox driver (如果存在)
if [ -f "$HOME/ws_livox/install/setup.bash" ]; then
    echo "  找到 livox_ros_driver2，正在 source..."
    source "$HOME/ws_livox/install/setup.bash"
else
    echo -e "${YELLOW}  警告: 未找到 livox_ros_driver2 (~/ws_livox/install/setup.bash)${NC}"
    echo "  如果使用其他激光雷达，请确保对应的 driver 已启动"
fi

# Source FAST_LIO_LOCALIZATION2
if [ -f "$HOME/FAST_LIO_LOCALIZATION2/install/setup.bash" ]; then
    source "$HOME/FAST_LIO_LOCALIZATION2/install/setup.bash"
else
    echo -e "${RED}错误: 未找到 FAST_LIO_LOCALIZATION2 安装${NC}"
    echo "请先运行: colcon build --symlink-install"
    exit 1
fi

# 2. 设置参数
echo -e "${GREEN}[2/5] 配置参数...${NC}"

# 地图文件路径 (可修改这里)
MAP_FILE="${MAP_FILE:-$HOME/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd}"
# MAP_FILE="$HOME/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd"

# 配置文件
CONFIG_FILE="${CONFIG_FILE:-mid360.yaml}"

# 是否启动 RViz
USE_RVIZ="${USE_RVIZ:-true}"

# 验证地图文件存在
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_FILE${NC}"
    echo "请检查路径或设置 MAP_FILE 环境变量"
    exit 1
fi

echo "  地图文件: $MAP_FILE"
echo "  配置文件: $CONFIG_FILE"
echo "  启动 RViz: $USE_RVIZ"

# 3. 发布初始位姿 (自动)
echo -e "${GREEN}[3/5] 准备发布初始位姿...${NC}"

# 在后台启动定位系统
echo -e "${GREEN}[4/5] 启动定位系统...${NC}"

# 根据是否启动RViz构建launch命令
if [ "$USE_RVIZ" = "true" ]; then
    ros2 launch fast_lio_localization localization.launch.py \
        map:="$MAP_FILE" \
        config_file:="$CONFIG_FILE" \
        rviz:=true &
    LAUNCH_PID=$!
else
    ros2 launch fast_lio_localization localization.launch.py \
        map:="$MAP_FILE" \
        config_file:="$CONFIG_FILE" \
        rviz:=false &
    LAUNCH_PID=$!
fi

# 等待系统初始化
echo -e "${YELLOW}  等待系统初始化 (10秒)...${NC}"
sleep 10

# 发布初始位姿
echo -e "${GREEN}[5/5] 发布初始位姿...${NC}"

# 发布到 /initialpose
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  "header": {
    "stamp": { "sec": 0, "nanosec": 0 },
    "frame_id": "map"
  },
  "pose": {
    "pose": {
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}' > /dev/null 2>&1

echo -e "${GREEN}初始位姿已发布！${NC}"
echo ""
echo -e "${YELLOW}========================================${NC}"
echo -e "${GREEN}系统已启动！${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
echo "监控命令:"
echo "  查看定位结果: ros2 topic echo /localization"
echo "  查看里程计:   ros2 topic echo /Odometry"
echo "  查看地图变换: ros2 topic echo /map_to_odom"
echo ""
echo "停止所有节点: kill $LAUNCH_PID"
echo ""

# 等待 launch 进程 (保持前台运行)
wait $LAUNCH_PID
