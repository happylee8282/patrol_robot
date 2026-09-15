# Core Concepts - 核心概念

## 框架概述
FAST_LIO_LOCALIZATION2是基于FAST-LIO2的实时3D全局定位框架，特点：
1. 不做点云特征提取，直接用原始点云配准
2. 增量式kd树(ikd-Tree)进行数据管理

## 系统组成
- **fastlio_mapping** (C++): FAST-LIO里程计，融合激光里程计+IMU
- **global_localization** (Python): 全局定位，负责重定位
- **transform_fusion** (Python): 坐标变换融合，发布全局一致的定位信息

## 输入话题
- /livox/lidar: LiDAR原始数据
- /livox/imu: IMU原始数据
- /initialpose: 初始位姿估计(RViz2)

## 输出话题
- /Odometry: FAST-LIO里程计输出
- /localization: 融合后的全局定位结果
- /map_to_odom: 地图到里程计的变换
