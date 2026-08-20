# 配置参数详解

## 主要参数 (ESEKF)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| max_iteration | 3 | ESEKF最大迭代次数 |
| filter_size_surf | 0.5 | 点云降采样体素大小(m) |
| filter_size_map | 0.5 | 地图降采样体素大小(m) |
| cube_side_length | 1000.0 | 局部地图立方体边长(m) |
| det_range | 100.0 | 检测范围(m) |
| fov_degree | 360.0 | 视场角(度) |
| acc_cov | 0.1 | 加速度计噪声协方差 |
| gyr_cov | 0.1 | 陀螺仪噪声协方差 |
| b_acc_cov | 0.0001 | 加速度计bias协方差 |
| b_gyr_cov | 0.0001 | 陀螺仪bias协方差 |
| extrinsic_T | [-0.011, -0.02329, 0.04412] | LiDAR到IMU平移外参 |
| extrinsic_R | [1,0,0,0,1,0,0,0,1] | LiDAR到IMU旋转外参 |

## 全局定位参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| map_voxel_size | 0.4 | 地图降采样大小(m) |
| scan_voxel_size | 0.1 | 扫描降采样大小(m) |
| freq_localization | 0.5 | 定位频率(Hz) |
| localization_threshold | 0.8 | ICP适应度阈值 |
| fov | 6.28319 | FOV范围(弧度，约360°) |
| fov_far | 300 | FOV最远距离(m) |
