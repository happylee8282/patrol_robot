# 完整工作流程

## 数据流图

```
1. 传感器数据采集
   LiDAR → /livox/lidar
   IMU → /livox/imu

2. FAST-LIO里程计处理
   ├─ 数据同步
   ├─ IMU预积分
   ├─ 点云去畸变
   ├─ ESEKF状态估计
   ├─ ikd-Tree地图更新
   └─ 发布 /Odometry, /cloud_registered

3. 全局定位
   ├─ 加载预建地图
   ├─ 接收初始位姿 (/initialpose)
   ├─ FOV裁剪
   ├─ 多尺度ICP配准
   └─ 发布 /map_to_odom

4. 坐标变换融合
   ├─ 融合里程计和全局定位
   ├─ 发布TF: map → camera_init
   └─ 发布 /localization

5. 可视化
   └─ RViz2显示地图、轨迹、点云
```

## 启动流程

```bash
# 1. 启动定位系统
ros2 launch fast_lio_localization localization.launch.py \
    map:=/path/to/your/map.pcd \
    config_file:=mid360.yaml

# 2. 在RViz2中使用"2D Pose Estimate"提供初始位姿

# 3. 保持机器人静止直到初始化成功
```

## IMU初始化流程

```cpp
void IMU_init() {
    // 1. 计算平均加速度和角速度
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
    
    // 2. 计算协方差
    cov_acc = ...;
    cov_gyr = ...;
    
    // 3. 初始化重力方向
    init_state.grav = -mean_acc / mean_acc.norm() * G_m_s2;
    
    // 4. 初始化陀螺仪bias
    init_state.bg = mean_gyr;
}
```

## 点云去畸变原理

```cpp
void UndistortPcl() {
    // 1. IMU前向传播，记录每个时刻的位姿
    for (auto it_imu : v_imu) {
        kf_state.predict(dt, Q, in);
        IMUpose.push_back(...);
    }
    
    // 2. 反向传播，补偿每个点
    for (auto it_pcl : pcl_out.points) {
        dt = it_pcl->curvature / 1000 - head->offset_time;
        // 计算该点时刻的位姿并变换到扫描结束时刻
    }
}
```
