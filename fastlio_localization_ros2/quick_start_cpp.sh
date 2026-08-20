#!/bin/bash
# FAST-LIO C++节点快速启动脚本

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}FAST-LIO C++节点快速启动${NC}"
echo -e "${GREEN}=========================================${NC}"

# 检查地图文件
MAP_PATH="/home/simit/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd"
if [ ! -f "$MAP_PATH" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_PATH${NC}"
    echo "请修改脚本中的MAP_PATH或确保地图文件存在"
    exit 1
fi

# 检查C++节点是否编译
if [ ! -f "/home/simit/FAST_LIO_LOCALIZATION2/install/fast_lio_localization/lib/fast_lio_localization/transform_fusion" ]; then
    echo -e "${YELLOW}C++节点未编译，开始编译...${NC}"
    /home/simit/FAST_LIO_LOCALIZATION2/build_and_test_cpp.sh
fi

# Source工作空间
echo -e "\n${YELLOW}Source工作空间...${NC}"
source /home/simit/FAST_LIO_LOCALIZATION2/install/setup.bash

# 停止可能运行的节点
echo -e "${YELLOW}停止现有节点...${NC}"
pkill -f fastlio_mapping || true
pkill -f transform_fusion || true
pkill -f global_localization || true
sleep 2

# 启动C++版本
echo -e "\n${GREEN}启动C++版本FAST-LIO定位系统...${NC}"
echo "地图: $MAP_PATH"
echo ""

# 后台启动并记录PID
ros2 launch fast_lio_localization localization_cpp.launch.py \
  map:="$MAP_PATH" \
  rviz:=false \
  use_cpp_nodes:=true &
LAUNCH_PID=$!

# 等待启动
sleep 5

# 检查节点是否启动
if ! ps -p $LAUNCH_PID > /dev/null; then
    echo -e "${RED}启动失败，请检查日志${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 系统启动成功！PID: $LAUNCH_PID${NC}"
echo ""
echo "节点列表:"
ros2 node list | grep -E "(transform_fusion|global_localization|laser_mapping)"
echo ""
echo "CPU使用率:"
ps aux | grep -E "(fastlio|transform_fusion|global_localization)" | grep -v grep | \
  awk '{printf "  %-25s CPU: %5.1f%%\n", $11, $3}'
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止系统${NC}"

# 等待用户中断
wait $LAUNCH_PID
