#!/bin/bash
# 测试C++节点

# 停止可能运行的节点
echo "停止所有FAST-LIO节点..."
pkill -f fastlio_mapping || true
pkill -f global_localization || true
pkill -f transform_fusion || true
sleep 2

# 启动LiDAR驱动（如果需要）
echo "启动LiDAR驱动..."
# ros2 launch livox_ros_driver2 msg_MID360_launch.py &
# sleep 3

# 启动C++节点
echo "启动C++版本的fastlio_mapping..."
ros2 run fast_lio_localization fastlio_mapping --ros-args \
  --params-file /home/simit/FAST_LIO_LOCALIZATION2/install/fast_lio_localization/share/fast_lio_localization/config/mid360.yaml &
FASTLIO_PID=$!
sleep 2

echo "启动C++版本的transform_fusion..."
ros2 run fast_lio_localization transform_fusion &
FUSION_PID=$!
sleep 2

echo "启动C++版本的global_localization..."
ros2 run fast_lio_localization global_localization --ros-args \
  -r pcd_map_path:=/home/simit/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd &
LOCALIZATION_PID=$!
sleep 2

# 监控CPU使用率
echo ""
echo "========================================="
echo "C++节点性能监控 (10秒)"
echo "========================================="
for i in {1..10}; do
    clear
    echo "CPU使用率监控 - 第 $i/10 秒"
    echo "========================================="
    ps aux | grep -E "(fastlio_mapping|transform_fusion|global_localization)" | grep -v grep | \
        awk '{printf "%-25s CPU: %5.1f%%  MEM: %4.1f%%  PID: %d\n", $11, $3, $4, $2}'
    sleep 1
done

# 清理
echo ""
echo "测试完成，停止节点..."
kill $FASTLIO_PID $FUSION_PID $LOCALIZATION_PID 2>/dev/null || true

echo "测试脚本已保存到: test_cpp_nodes.sh"
