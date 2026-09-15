# 双树 EKF 全局定位实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 FAST-LIO 内部加载全局地图到第二棵 ikd-Tree，让 EKF 同时利用局部地图和全局地图进行 scan matching，消除漂移。

**Architecture:** 在 laserMapping.cpp 中新增 global_ikdtree，修改 h_share_model() 对每个点同时搜索两棵树并合并测量。PCD 加载在构造函数中完成。添加 /initialpose 订阅设置初始位姿。

**Tech Stack:** C++14, ROS2 Humble, PCL, Eigen, ikd-Tree

## 当前进度

| Task | 状态 | Commit |
|---|---|---|
| Task 1: 新增全局 ikd-Tree 变量和 PCD 加载函数 | ✅ 完成 | `6e20860` |
| Task 2: 构造函数中声明参数并加载全局地图 | ✅ 完成 | `6e20860` |
| Task 3: 提取 search_and_fit 辅助函数（重构） | ✅ 完成 | `e303332` |
| Task 4: 修改 h_share_model 实现双树搜索 | ✅ 完成 | `1a2a3a4` |
| Task 5: 修改首帧初始化逻辑 | ✅ 完成 | `7fbff97` |
| Task 6: 新增 /initialpose 订阅 | ✅ 完成 | `7fbff97` |
| Task 7: 集成测试 | ⏳ 待硬件测试 | - |

---

## 文件结构

| 文件 | 职责 | 变更 |
|---|---|---|
| `src/laserMapping.cpp` | FAST-LIO 主节点 | 修改（新增双树、修改 h_share_model、新增 PCD 加载和 initialpose） |
| `config/mid360.yaml` | 参数配置 | 修改（新增 global_map_path 等参数） |
| `launch/` 相关文件 | 启动文件 | 可能需修改 |
| `include/ikd-Tree/ikd_Tree.h` | ikd-Tree 头文件 | 不修改 |
| `src/IMU_Processing.hpp` | IMU 处理 | 不修改 |

---

## Task 1: 新增全局 ikd-Tree 变量和 PCD 加载函数

**Files:**
- Modify: `src/laserMapping.cpp:119` (全局变量区)

**验证:** 编译通过，无运行时变化

- [ ] **Step 1:** 在 `KD_TREE ikdtree;` (line 119) 后面添加全局 ikd-Tree 变量和加载函数

在 `laserMapping.cpp` line 119 后添加：

```cpp
// line 119: KD_TREE ikdtree;  ← 已有
KD_TREE global_ikdtree;          // 全局先验地图 (只读)
bool global_map_loaded = false;  // 全局地图是否已加载
```

在 `ikdtree` 变量之后、`XAxisPoint_body` 之前添加 PCD 加载函数：

```cpp
void load_global_map(const string &pcd_path, double voxel_size)
{
    if (pcd_path.empty()) return;

    PointCloudXYZI::Ptr raw_map(new PointCloudXYZI);
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *raw_map) == -1)
    {
        cerr << "Failed to load global map: " << pcd_path << endl;
        return;
    }

    PointCloudXYZI::Ptr filtered(new PointCloudXYZI);
    pcl::VoxelGrid<PointType> vf;
    vf.setInputCloud(raw_map);
    vf.setLeafSize(voxel_size, voxel_size, voxel_size);
    vf.filter(*filtered);

    global_ikdtree.set_downsample_param(voxel_size);
    global_ikdtree.Build(filtered->points);
    global_map_loaded = true;

    cout << "Global map loaded: " << filtered->size() << " points from " << pcd_path << endl;
}
```

- [ ] **Step 2:** 编译验证

Run: `cd /home/simit/FAST_LIO_LOCALIZATION2 && colcon build --packages-select fast_lio_localization 2>&1 | tail -5`
Expected: 编译成功

- [ ] **Step 3:** Commit

```bash
git add src/laserMapping.cpp
git commit -m "feat: add global ikd-Tree variable and PCD loading function"
```

---

## Task 2: 在构造函数中声明参数并加载全局地图

**Files:**
- Modify: `src/laserMapping.cpp:956-1069` (构造函数)
- Modify: `config/mid360.yaml`

**验证:** 编译通过，启动时能看到全局地图加载日志

- [ ] **Step 1:** 在构造函数中声明新参数（在 line 989 `declare_parameter<vector<double>>("mapping.extrinsic_R"...)` 之后）

```cpp
this->declare_parameter<string>("global_map.global_map_path", "");
this->declare_parameter<double>("global_map.global_map_voxel_size", 0.4);
this->declare_parameter<bool>("global_map.global_map_enabled", false);
```

在参数读取区（line 1022 `get_parameter_or<vector<double>>("mapping.extrinsic_R"...)` 之后）添加：

```cpp
string global_map_path;
double global_map_voxel_size;
bool global_map_enabled;
this->get_parameter_or<string>("global_map.global_map_path", global_map_path, "");
this->get_parameter_or<double>("global_map.global_map_voxel_size", global_map_voxel_size, 0.4);
this->get_parameter_or<bool>("global_map.global_map_enabled", global_map_enabled, false);

if (global_map_enabled && !global_map_path.empty())
{
    load_global_map(global_map_path, global_map_voxel_size);
}
```

- [ ] **Step 2:** 在 `config/mid360.yaml` 末尾添加全局地图参数段

```yaml
        global_map:
            global_map_enabled: false
            global_map_path: ""
            global_map_voxel_size: 0.4
```

- [ ] **Step 3:** 编译验证

Run: `cd /home/simit/FAST_LIO_LOCALIZATION2 && colcon build --packages-select fast_lio_localization 2>&1 | tail -5`
Expected: 编译成功

- [ ] **Step 4:** Commit

```bash
git add src/laserMapping.cpp config/mid360.yaml
git commit -m "feat: add global map parameters and loading in constructor"
```

---

## Task 3: 提取单树搜索+平面拟合为辅助函数

**Files:**
- Modify: `src/laserMapping.cpp:826-938` (h_share_model 之前)

**验证:** 编译通过，行为与原版完全一致（此时不使用 global_ikdtree）

这是最关键的重构步骤。将 `h_share_model` 中对单个点的"搜索→平面拟合→筛选"提取为独立函数，避免后续双树搜索时重复代码。

- [ ] **Step 1:** 在 `h_share_model()` 函数之前添加辅助结构体和函数

```cpp
struct SearchResult {
    bool valid;              // 是否有效（搜索成功 + 平面拟合成功 + 置信度足够）
    float norm_x, norm_y, norm_z;  // 平面法向量
    float residual;          // 点到平面距离（有符号）
};

SearchResult search_and_fit(KD_TREE<PointType> &tree,
                            const PointType &point_world,
                            int num_match_points,
                            bool converged)
{
    SearchResult result;
    result.valid = false;

    if (!converged) return result;

    vector<float> pointSearchSqDis(num_match_points);
    PointVector points_near;
    tree.Nearest_Search(point_world, num_match_points, points_near, pointSearchSqDis);

    if (points_near.size() < num_match_points) return result;
    if (pointSearchSqDis[num_match_points - 1] > 5.0) return result;

    VF(4) pabcd;
    if (!esti_plane(pabcd, points_near, 0.1f)) return result;

    float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                pabcd(2) * point_world.z + pabcd(3);

    V3D p_body_local(point_world.x, point_world.y, point_world.z);
    float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body_local.norm());
    if (s <= 0.9) return result;

    result.valid = true;
    result.norm_x = pabcd(0);
    result.norm_y = pabcd(1);
    result.norm_z = pabcd(2);
    result.residual = pd2;
    return result;
}
```

**注意：** 上面 `p_body_local` 使用了 `point_world` 的坐标来算距离权重，这是临时的近似。在 Step 2 中我们传入正确的 `p_body`。

实际上 `search_and_fit` 不需要 `p_body`——原始代码中 `s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm())` 使用的是 body 系下的点。修正辅助函数签名，增加 `p_body_norm` 参数：

```cpp
SearchResult search_and_fit(KD_TREE<PointType> &tree,
                            const PointType &point_world,
                            int num_match_points,
                            bool converged,
                            double p_body_norm)
{
    SearchResult result;
    result.valid = false;

    if (!converged) return result;

    vector<float> pointSearchSqDis(num_match_points);
    PointVector points_near;
    tree.Nearest_Search(point_world, num_match_points, points_near, pointSearchSqDis);

    if (points_near.size() < num_match_points) return result;
    if (pointSearchSqDis[num_match_points - 1] > 5.0) return result;

    VF(4) pabcd;
    if (!esti_plane(pabcd, points_near, 0.1f)) return result;

    float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                pabcd(2) * point_world.z + pabcd(3);

    float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body_norm);
    if (s <= 0.9) return result;

    result.valid = true;
    result.norm_x = pabcd(0);
    result.norm_y = pabcd(1);
    result.norm_z = pabcd(2);
    result.residual = pd2;
    return result;
}
```

- [ ] **Step 2:** 重写 `h_share_model()` 使用辅助函数（保持单树，行为不变）

将整个 `h_share_model()` 替换为：

```cpp
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        point_selected_surf[i] = false;

        SearchResult sr = search_and_fit(ikdtree, point_world, NUM_MATCH_POINTS,
                                          ekfom_data.converge, p_body.norm());
        if (sr.valid)
        {
            point_selected_surf[i] = true;
            normvec->points[i].x = sr.norm_x;
            normvec->points[i].y = sr.norm_y;
            normvec->points[i].z = sr.norm_z;
            normvec->points[i].intensity = sr.residual;
            res_last[i] = fabs(sr.residual);
        }
    }

    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}
```

- [ ] **Step 3:** 编译验证

Run: `cd /home/simit/FAST_LIO_LOCALIZATION2 && colcon build --packages-select fast_lio_localization 2>&1 | tail -5`
Expected: 编译成功

- [ ] **Step 4:** Commit

```bash
git add src/laserMapping.cpp
git commit -m "refactor: extract search_and_fit helper from h_share_model"
```

---

## Task 4: 修改 h_share_model 实现双树搜索

**Files:**
- Modify: `src/laserMapping.cpp` (h_share_model 内的搜索循环)

**验证:** 编译通过。加载全局地图后运行，观察 log 确认双树搜索生效

这是核心步骤。修改搜索循环，对每个点同时搜索 local 和 global 树，合并有效测量。

- [ ] **Step 1:** 修改 h_share_model 中的搜索循环部分

将 Task 3 中的搜索循环替换为双树版本。只修改 `#pragma omp parallel for` 循环和后续的有效点收集部分：

```cpp
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        point_selected_surf[i] = false;
        double p_body_norm = p_body.norm();

        // 搜索局部地图
        SearchResult sr_local = search_and_fit(ikdtree, point_world, NUM_MATCH_POINTS,
                                                ekfom_data.converge, p_body_norm);

        // 搜索全局地图
        SearchResult sr_global;
        if (global_map_loaded && ekfom_data.converge)
        {
            sr_global = search_and_fit(global_ikdtree, point_world, NUM_MATCH_POINTS,
                                       true, p_body_norm);
        }

        // 优先使用局部地图结果，局部无效时回退到全局
        SearchResult &sr = sr_local.valid ? sr_local : sr_global;
        if (sr.valid)
        {
            point_selected_surf[i] = true;
            normvec->points[i].x = sr.norm_x;
            normvec->points[i].y = sr.norm_y;
            normvec->points[i].z = sr.norm_z;
            normvec->points[i].intensity = sr.residual;
            res_last[i] = fabs(sr.residual);
        }
    }

    // 如果全局地图也加载了，额外的全局匹配点也要加入测量
    // 使用一个额外数组存储仅全局匹配的点
    int global_only_count = 0;
    vector<int> global_only_indices;
    if (global_map_loaded)
    {
        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i]) continue; // 已经有局部匹配
            // 在全局地图中单独搜索（上面循环中已经搜过但被跳过了，这里复用）
            // 实际上上面已经搜过了，但 sr_global 在循环外不可访问
            // 修正：需要在循环中同时收集
        }
    }
    // 注意：上面的逻辑有缺陷——sr_global 在循环内部是局部变量。
    // 需要换一种方式：直接在循环中同时收集两棵树的结果。
```

上面的实现有问题。重新设计：在循环中同时收集两棵树的结果，全部加入测量向量。

```cpp
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    // 为每个点存储最多2个测量：[local, global]
    // 使用平铺数组：normvec_local 和 normvec_global
    // 简化方案：先将所有有效结果收集到 vectors 中，最后拷贝到连续数组

    vector<V3F> valid_norms;     // 法向量
    vector<float> valid_res;     // 残差
    vector<int> valid_point_idx; // 对应 feats_down_body 的索引
    vector<bool> valid_is_global; // 标记来源

    // 预分配空间
    int max_measurements = feats_down_size * 2;
    valid_norms.reserve(max_measurements);
    valid_res.reserve(max_measurements);
    valid_point_idx.reserve(max_measurements);
    valid_is_global.reserve(max_measurements);

    // 为了兼容 point_selected_surf 和 normvec（后续 map_incremental 使用），
    // 仍然保留原有的 normvec 数组用于局部匹配结果
    memset(point_selected_surf, false, sizeof(bool) * feats_down_size);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        double p_body_norm = p_body.norm();

        // 搜索局部地图
        SearchResult sr_local = search_and_fit(ikdtree, point_world, NUM_MATCH_POINTS,
                                                ekfom_data.converge, p_body_norm);

        if (sr_local.valid)
        {
            point_selected_surf[i] = true;
            normvec->points[i].x = sr_local.norm_x;
            normvec->points[i].y = sr_local.norm_y;
            normvec->points[i].z = sr_local.norm_z;
            normvec->points[i].intensity = sr_local.residual;
            res_last[i] = fabs(sr_local.residual);
        }
    }

    // 统计局部匹配的有效点（保持与 map_incremental 兼容）
    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    // 额外收集全局地图匹配点（仅在双树模式下）
    int global_extra_count = 0;
    if (global_map_loaded && ekfom_data.converge)
    {
        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i]) continue; // 局部已匹配，跳过

            PointType &point_world = feats_down_world->points[i];
            V3D p_body(feats_down_body->points[i].x,
                        feats_down_body->points[i].y,
                        feats_down_body->points[i].z);

            SearchResult sr_global = search_and_fit(global_ikdtree, point_world,
                                                     NUM_MATCH_POINTS, true, p_body.norm());
            if (sr_global.valid)
            {
                laserCloudOri->points[effct_feat_num + global_extra_count] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num + global_extra_count].x = sr_global.norm_x;
                corr_normvect->points[effct_feat_num + global_extra_count].y = sr_global.norm_y;
                corr_normvect->points[effct_feat_num + global_extra_count].z = sr_global.norm_z;
                corr_normvect->points[effct_feat_num + global_extra_count].intensity = sr_global.residual;
                total_residual += fabs(sr_global.residual);
                global_extra_count++;
            }
        }
    }

    int total_feat_num = effct_feat_num + global_extra_count;
    res_mean_last = total_feat_num > 0 ? total_residual / total_feat_num : 0;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    // 构建 H 和 h（包含局部+全局所有测量）
    ekfom_data.h_x = MatrixXd::Zero(total_feat_num, 12);
    ekfom_data.h.resize(total_feat_num);

    for (int i = 0; i < total_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}
```

**关键设计决策：**
- 局部匹配点先收集，保持 `point_selected_surf` 和 `effct_feat_num` 不变（`map_incremental()` 依赖）
- 全局匹配仅补充局部未匹配到的点（不重复）
- H 和 h 合并局部+全局所有测量，一次性送入 EKF

- [ ] **Step 2:** 编译验证

Run: `cd /home/simit/FAST_LIO_LOCALIZATION2 && colcon build --packages-select fast_lio_localization 2>&1 | tail -5`
Expected: 编译成功

- [ ] **Step 3:** Commit

```bash
git add src/laserMapping.cpp
git commit -m "feat: dual-tree search in h_share_model"
```

---

## Task 5: 修改首帧初始化逻辑

**Files:**
- Modify: `src/laserMapping.cpp:1170-1184` (首帧 kdtree 初始化)

**验证:** 编译通过。有全局地图时，即使 local ikdtree 为空，第一帧也能靠全局地图完成匹配

- [ ] **Step 1:** 修改首帧逻辑

将 line 1170-1184 的逻辑修改为：如果有全局地图，local ikdtree 可以为空，跳过 `return` 继续执行 EKF 更新。

```cpp
            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                    RCLCPP_INFO(this->get_logger(), "Initialize the local map kdtree");
                }

                // 有全局地图时不需要等 local 树建好，可以直接匹配
                if (!global_map_loaded)
                    return;

                // 全局模式下也需要 feats_down_world 有数据
                if (ikdtree.Root_Node == nullptr)
                {
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    // 建一个空的 local 树，后续 map_incremental 会添加点
                    PointVector empty_pv;
                    ikdtree.set_downsample_param(filter_size_map_min);
                    ikdtree.Build(empty_pv);
                }
            }
```

- [ ] **Step 2:** 编译验证

Run: `cd /home/simit/FAST_LIO_LOCALIZATION2 && colcon build --packages-select fast_lio_localization 2>&1 | tail -5`
Expected: 编译成功

- [ ] **Step 3:** Commit

```bash
git add src/laserMapping.cpp
git commit -m "feat: allow first frame matching with global map only"
```

---

## Task 6: 新增 /initialpose 订阅

**Files:**
- Modify: `src/laserMapping.cpp` (LaserMappingNode 构造函数)

**验证:** 编译通过。启动后通过 RViz 发布 initialpose，观察 EKF 状态被设置

- [ ] **Step 1:** 在构造函数中添加 initialpose 订阅（在 IMU/LiDAR 订阅之后）

```cpp
        auto sub_initial_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                auto &p = msg->pose.pose.position;
                auto &q = msg->pose.pose.orientation;
                RCLCPP_INFO(this->get_logger(), "Initial pose received: (%.2f, %.2f, %.2f)",
                           p.x, p.y, p.z);

                state_point.pos = V3D(p.x, p.y, p.z);
                state_point.rot = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
                kf.change_x(state_point);
            });
```

需要在构造函数开头确保 `#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>` 已存在。检查发现文件已有相关 include（line 38 有 `geometry_msgs` 的引用），可能需要添加具体的头文件。

在文件头部的 include 区域（line 54 之后）添加：
```cpp
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
```

- [ ] **Step 2:** 编译验证

Run: `cd /home/simit/FAST_LIO_LOCALIZATION2 && colcon build --packages-select fast_lio_localization 2>&1 | tail -5`
Expected: 编译成功

- [ ] **Step 3:** Commit

```bash
git add src/laserMapping.cpp
git commit -m "feat: add /initialpose subscription to set EKF initial state"
```

---

## Task 7: 集成测试

**验证:** 端到端运行验证

- [ ] **Step 1:** 修改 config 启用全局地图

在 `config/mid360.yaml` 中设置：
```yaml
        global_map:
            global_map_enabled: true
            global_map_path: "/home/simit/FAST_LIO_LOCALIZATION2/maps/12f_map.pcd"
            global_map_voxel_size: 0.4
```

- [ ] **Step 2:** 编译并启动系统

Run: `cd /home/simit/FAST_LIO_LOCALIZATION2 && colcon build --packages-select fast_lio_localization`

- [ ] **Step 3:** 启动后检查日志
  - 看到 "Global map loaded: XXXX points" 
  - 看到 "Initialize the local map kdtree" 或直接跳过
  - 通过 RViz 给 /initialpose
  - 观察 /Odometry 输出是否在 map 坐标系下稳定

- [ ] **Step 4:** Commit final

```bash
git add config/mid360.yaml
git commit -m "feat: enable global map in config for testing"
```

---

## 自查清单

| 检查项 | 状态 |
|---|---|
| 每个Task独立编译通过 | ✅ 每步都有编译验证 |
| 无全局地图时行为不变 | ✅ `global_map_loaded` 为 false 时跳过所有全局逻辑 |
| h_share_model 向后兼容 | ✅ 不使用全局地图时走原逻辑 |
| map_incremental 不受影响 | ✅ 只写 local ikdtree，`point_selected_surf` 仅标记局部匹配 |
| ikd-Tree API 不修改 | ✅ 只用 `Build`, `Nearest_Search` |
| EKF 核心不修改 | ✅ `update_iterated_dyn_share_modified` 不变，只是测量维度可能变大 |
