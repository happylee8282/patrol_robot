# 双树 EKF 全局定位设计文档

**日期**: 2026-04-14
**状态**: 待实现

## 问题

当前 FAST-LIO 使用单棵 ikd-Tree 从零累积局部地图。外部 global_localization 节点通过 ICP 计算 map→odom 变换，但无法反馈到 FAST-LIO 内部。LiDAR 中断后 FAST-LIO 内部状态崩坏（位置、速度、IMU bias 全部漂移），外部 ICP 无法修正，系统不可恢复。

## 方案

在 FAST-LIO 内部引入第二棵 ikd-Tree（全局地图），让 EKF 同时利用局部地图和全局地图进行 scan matching。

### 核心思路

每个扫描点产生两套测量：
- **局部测量**: 在 local_ikdtree（累积的局部地图）中搜索最近邻 → 平面拟合 → h_local, H_local
- **全局测量**: 在 global_ikdtree（预加载的先验地图，只读）中搜索最近邻 → 平面拟合 → h_global, H_global

合并为单次 EKF 更新:
```
h = [h_local; h_global]
H = [H_local; H_global]
```

EKF 通过卡尔曼增益自然决定权重。全局地图修正位置+旋转的同时，也会通过耦合的 Jacobian 修正 IMU bias。

### 架构

```
启动:
  1. 加载 PCD → global_ikdtree.Build(prior_map_points)   ← 只读
  2. local_ikdtree 从空开始
  3. 等待 /initialpose → 设置 EKF 初始位姿

每帧 (timer_callback):
  IMU 预测 (不变)
  lasermap_fov_segment() (不变，只管 local_ikdtree)
  体素降采样 (不变)
  h_share_model()  ← 核心修改
    对每个点:
      a. local_ikdtree.Nearest_Search → 平面拟合 → h_local, H_local
      b. global_ikdtree.Nearest_Search → 平面拟合 → h_global, H_global
      c. 合并有效测量
  EKF 更新 (不变，测量维度变大)
  map_incremental() (不变，只写 local_ikdtree)
  publish_odometry() (不变)

发布:
  /Odometry → 直接输出 map 坐标系下的位姿（不再需要 map→odom 补偿）
```

### 简化：不再需要的组件

| 去掉的组件 | 原因 |
|---|---|
| `global_localization.cpp` | EKF 直接对全局地图 matching |
| `transform_fusion.cpp` | FAST-LIO 直接输出 map 系位姿 |
| `/map_to_odom` 话题 | 不再需要 map→odom 变换补偿 |
| `/localization` 话题融合 | `/Odometry` 就是最终结果 |

### 保留：初始位姿

全局地图坐标系与启动位置不一致，需要通过 `/initialpose` 设置初始位姿。FAST-LIO 收到 initialpose 后设置 EKF 状态（位置、旋转）。

## 修改范围

### 文件: `src/laserMapping.cpp`

1. **新增全局 ikd-Tree 变量** (~line 100)
   - `KD_TREE<PointType> global_ikdtree;`
   - `bool global_map_loaded = false;`

2. **新增 PCD 加载参数和逻辑** (~line 995)
   - 新参数: `global_map_path` (string), `global_map_enabled` (bool)
   - 启动时: `pcl::io::loadPCDFile` → 降采样 → `global_ikdtree.Build()`

3. **修改 h_share_model()** (~line 826) — 核心修改
   - 对每个点，额外搜索 global_ikdtree
   - 合并两棵树的残差和 Jacobian
   - 无全局地图时退化为原逻辑

4. **修改首帧初始化逻辑** (~line 1170)
   - 如果全局地图已加载，local_ikdtree 为空也能靠全局地图匹配

5. **新增 /initialpose 订阅**
   - 收到初始位姿后设置 EKF 状态

### 文件: `config/` 下的参数文件

6. **新增参数**
   - `global_map_path`: PCD 文件路径
   - `global_map_enabled`: 是否启用双树模式

### 不修改

- `ikd_Tree.h/cpp`: API 足够，无需改动
- `IMU_Processing.hpp`: IMU 预测逻辑不变
- `esekfom.hpp`: EKF 更新逻辑不变

## 假设

**V1 假设：匹配质量可靠。** 即 `search_and_fit` 中的阈值筛选（近邻数、平面拟合残差、置信度 `s > 0.9`）足以过滤掉错误匹配。在此假设下，两棵树的所有有效结果直接送入 EKF，不做额外的异常值剔除。

如果后续实测中发现错误匹配导致状态被拉偏，再引入卡方检验或动态调 R。

## 现有 EKF 对异常测量的处理能力分析

### 代码现状

经过对 `esekfom.hpp` 中 `update_iterated_dyn_share_modified` 的完整分析：

**当前 EKF 没有根据残差自动降权的机制。** 具体来说：

1. **测量噪声 R 是常量**：`R = LASER_POINT_COV = 0.001`，标量，所有测量一视同仁
2. **s 分数只做硬门限**：`search_and_fit` 中 `s = 1 - 0.9 * |pd2| / sqrt(dist)` 只判断 `s > 0.9`（过/不过），从未作为权重传入 EKF
3. **没有鲁棒核函数**：残差 `h` 直接线性进入 `dx = K * h`，无 Huber/Cauchy
4. **没有卡方检验**：无 `h^T S^{-1} h` 门限
5. **P 矩阵不响应残差大小**：P 反映先验不确定性，随收敛变小会降低所有测量的增益，但不区分好坏测量

### 理论上可行的改进方向（V2+）

| 方案 | 原理 | 改动量 |
|------|------|--------|
| 自适应 R | 残差大时增大该测量的 R，降权 | 中（需改 esekfom.hpp） |
| 卡方门限 | `h^T S^{-1} h > χ²` 时拒绝 | 小（加在 h_share_model 之后） |
| s 分数作权重 | `R_effective = R / s`，用已有的 s 分数 | 小（改 laserMapping.cpp） |
| Huber 核 | 残差超过阈值时线性化 | 中（改 esekfom.hpp） |

其中 **s 分数作权重** 最简单——代码已经在计算 `s`，只差一步 `R_effective = R / s`。

## 数据流

### 树的职责

| 树 | 内容 | 写入 | 用途 |
|---|---|---|---|
| local_ikdtree | FAST-LIO 从零累积的局部地图 | `map_incremental()` 增量添加 | 局部匹配 |
| global_ikdtree | 从 PCD 文件加载的先验地图 | 启动时 `Build()` 一次，之后**只读** | 全局匹配 |

### h_share_model 两遍搜索

```
第一遍：搜局部树（保持与原版完全兼容）
  for i in 0..feats_down_size:
    变换 point_body → point_world
    sr_local = search_and_fit(ikdtree, point_world, ...)
    if sr_local.valid:
      point_selected_surf[i] = true       ← map_incremental 依赖此标记
      normvec[i] = sr_local               ← 保留给后续使用
      laserCloudOri[effct_idx] = point_body
      corr_normvect[effct_idx] = sr_local
      effct_idx++

  effct_feat_num = effct_idx   ← 局部匹配点数

第二遍：搜全局树，仅对局部未匹配的点（仅 global_map_loaded）
  for i in 0..feats_down_size:
    if point_selected_surf[i]: continue   ← 局部已匹配，跳过
    sr_global = search_and_fit(global_ikdtree, point_world, ...)
    if sr_global.valid:
      laserCloudOri[effct_feat_num + global_idx] = point_body
      corr_normvect[effct_feat_num + global_idx] = sr_global
      global_idx++

  total_feat_num = effct_feat_num + global_idx

构建 H, h（覆盖 total_feat_num 行）:
  for i in 0..total_feat_num:
    h(i)   = -(点到平面距离)
    H(i,:) = [n^T, A^T, B^T, C^T]  (12维 Jacobian)
  送入 EKF 更新（与原版完全相同）
```

### 关键约束

- `point_selected_surf[]` 和 `effct_feat_num` 仅反映局部匹配结果 → `map_incremental()` 行为不变
- `laserCloudOri` 和 `corr_normvect` 预分配大小需 ≥ `feats_down_size * 2`
- 全局树搜索仅在 `global_map_loaded && ekfom_data.converge` 时执行
- EKF 不区分测量来源，所有有效结果统一做加权最小二乘

## 理论依据

EKF 测量更新本质是加权最小二乘:
```
dx = (H^T R^{-1} H)^{-1} H^T R^{-1} (-h)
```

两棵树提供同一状态的不同观测。当两者一致时，信息叠加，协方差收敛更快。
当局部地图丢失（LiDAR 中断后）: 全局树单独接管定位。
IMU bias 通过耦合的 Jacobian H 的旋转/速度分量被修正。

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| 全局地图点少(3246点)，匹配质量差 | 地图已降采样到 0.4m，走廊环境足够 |
| 双树搜索增加延迟 | 全局树只有 3246 点，搜索极快(<0.1ms) |
| 全局地图坐标系与初始位姿不一致 | 通过 /initialpose 给初始位姿 |
| 错误匹配拉偏状态 | V1 假设匹配可靠；后续可加卡方检验或动态调 R |
| 两次 Nearest_Search 代码重复 | 提取为辅助函数 |

## 测试计划

1. 不加载全局地图时，行为与原版完全一致
2. 加载全局地图 + 给初始位姿后，正常运动时定位稳定
3. 模拟 LiDAR 中断（遮挡传感器 2 秒），恢复后能自动回到正确位置
4. CPU 占用增长不超过 20%

## 成功标准

- LiDAR 中断 2 秒后恢复，EKF 能在 1 秒内回到正确位姿
- 正常运行时定位精度不低于原版
- 无需外部重定位节点即可自恢复
- /Odometry 直接输出 map 坐标系位姿
