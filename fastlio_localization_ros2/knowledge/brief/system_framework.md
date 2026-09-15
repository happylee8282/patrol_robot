# 系统框架 - Brief Summary

## 目录结构
```
FAST_LIO_LOCALIZATION2-ros2/
├── src/                          # C++源代码
│   ├── laserMapping.cpp          # 主程序：FAST-LIO里程计
│   ├── IMU_Processing.hpp        # IMU处理和点云去畸变
│   ├── preprocess.cpp/h          # 点云预处理
├── fast_lio_localization/        # Python模块
│   ├── global_localization.py    # 全局定位节点
│   ├── transform_fusion.py       # 坐标变换融合
│   ├── publish_initial_pose.py   # 初始位姿发布
│   └── invert_livox_scan.py      # Livox扫描反转
├── include/                      # 头文件
│   ├── ikd-Tree/                 # 增量式KD树
│   ├── IKFoM_toolkit/            # 迭代卡尔曼滤波工具
├── config/                       # 配置文件
└── launch/                       # 启动文件
```

## 三大核心组件
1. fastlio_mapping (C++) - 负责定位，融合激光里程计+IMU
2. global_localization (Python) - 负责重定位
3. transform_fusion (Python) - 融合FAST-LIO里程计和全局定位结果
