# 核心算法原理 - Brief Summary

## 1. ESEKF (误差状态卡尔曼滤波)
状态向量:
```
state_ikfom {
    rot          // 旋转（SO3）
    pos          // 位置（3D）
    vel          // 速度（3D）
    bg           // 陀螺仪bias（3D）
    ba           // 加速度计bias（3D）
    grav         // 重力向量（3D）
    offset_R_L_I // LiDAR到IMU旋转外参
    offset_T_L_I // LiDAR到IMU平移外参
}
```

## 2. 点到面ICP配准
- 最近邻搜索（ikd-Tree）
- 平面拟合
- 计算残差（点到面距离）
- 雅可比矩阵计算(12维)

## 3. ikd-Tree增量KD树
- 动态平衡：支持增量插入和删除
- Box删除：FOV外的点批量删除
- 降采样：体素网格降采样
- 并行搜索：OpenMP加速
