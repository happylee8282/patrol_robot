#!/bin/bash
# FAST-LIO完整系统启动脚本

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}FAST-LIO 完整系统启动${NC}"
echo -e "${GREEN}=========================================${NC}"

# 检查地图文件
MAP_PATH="/home/simit/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd"
if [ ! -f "$MAP_PATH" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_PATH${NC}"
    echo "请检查地图文件路径"
    exit 1
fi

echo -e "${YELLOW}步骤 1: Source工作空间${NC}"
cd /home/simit/FAST_LIO_LOCALIZATION2
source install/setup.bash
echo -e "${GREEN}✓ 工作空间已source${NC}"

echo -e "${YELLOW}步骤 2: 检查C++节点${NC}"
if [ ! -f "install/fast_lio_localization/lib/fast_lio_localization/transform_fusion" ]; then
    echo -e "${RED}错误: C++节点未编译${NC}"
    echo "请运行: ./build_and_test_cpp.sh"
    exit 1
fi
echo -e "${GREEN}✓ C++节点已编译${NC}"

echo -e "${YELLOW}步骤 3: 停止现有节点${NC}"
pkill -f livox || true
pkill -f fastlio_mapping || true
pkill -f transform_fusion || true
pkill -f global_localization || true
sleep 2
echo -e "${GREEN}✓ 现有节点已停止${NC}"

echo -e ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}步骤 4: 启动FAST-LIO系统${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo -e "${YELLOW}启动选项:${NC}"
echo "  1. 完整系统 (LiDAR + 定位)"
echo "  2. 仅定位节点 (假设LiDAR已启动)"
echo ""
read -p "请选择 (1 或 2): " choice

if [ "$choice" = "1" ]; then
    echo -e "${YELLOW}启动完整系统...${NC}"
    
    # 启动LiDAR驱动
    echo "  • 启动LiDAR驱动..."
    ros2 launch livox_ros_driver2 msg_MID360_launch.py &
    LIDOV_PID=$!
    sleep 5
    
    # 检查LiDAR是否启动
    if ! ros2 topic list | grep -q "/livox/lidar"; then
        echo -e "${RED}警告: LiDAR话题未找到，继续启动...${NC}"
    fi
    
elif [ "$choice" = "2" ]; then
    echo -e "${YELLOW}仅启动定位节点...${NC}"
    LIDOV_PID=""
else
    echo -e "${RED}无效选择，退出${NC}"
    exit 1
fi

# 启动FAST-LIO定位系统（C++版本）
echo "  • 启动FAST-LIO定位系统 (C++版本)..."
ros2 launch fast_lio_localization localization_cpp.launch.py \
    map:="$MAP_PATH" \
    rviz:=false \
    use_cpp_nodes:=true &
LAUNCH_PID=$!

sleep 5

# 检查节点是否启动
echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}系统状态检查${NC}"
echo -e "${GREEN}=========================================${NC}"

echo -e "\n${YELLOW}运行的节点:${NC}"
ros2 node list | grep -E "(fastlio|transform_fusion|global_localization)" || echo "  未找到相关节点"

echo -e "\n${YELLOW}话题列表:${NC}"
ros2 topic list | grep -E "(cloud|Odometry|localization)" | head -10

echo -e "\n${YELLOW}CPU使用率:${NC}"
ps aux | grep -E "(fastlio_mapping|transform_fusion|global_localization)" | grep -v grep | \
    awk '{printf "  %-25s CPU: %5.1f%%  MEM: %4.1f%%\n", $11, $3, $4}'

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}系统启动完成！${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo -e "${YELLOW}下一步操作:${NC}"
echo "  1. 在RViz2中设置初始位姿 (2D Pose Estimate)"
echo "     ros2 run rviz2 rviz2 -d /home/simit/FAST_LIO_LOCALIZATION2/rviz_cfg/"
echo ""
echo "  2. 监控性能"
echo "     watch -n 1 'ps aux | grep -E (fastlio|transform_fusion|global_localization)'"
echo ""
echo "  3. 查看定位结果"
echo "     ros2 topic echo /localization"
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止系统${NC}"

# 等待用户中断
wait $LAUNCH_PID

# 清理
echo ""
echo -e "${YELLOW}停止系统...${NC}"
kill $LAUNCH_PID 2>/dev/null || true
if [ -n "$LIDOV_PID" ]; then
    kill $LIDOV_PID 2>/dev/null || true
fi
pkill -f "fast_lio_localization" || true
echo -e "${GREEN}✓ 系统已停止${NC}"
