/*
 * @file laserMapping.cpp
 * @brief FAST-LIO 激光雷达建图模块主实现文件
 *
 * 本文件实现了基于 LOAM (Lidar Odometry and Mapping) 思想的激光雷达惯性里程计
 * 和地图构建系统。使用迭代扩展卡尔曼滤波器 (Iterated EKF) 进行状态估计，
 * 结合点云特征提取和扫描到地图的匹配，实现实时高精度定位与建图。
 *
 * 核心算法：
 * - 点云特征提取（来自 preprocess.h）
 * - 基于平面特征的扫描匹配（point-to-plane ICP）
 * - 状态预测和更新（IMU 预积分 + EKF）
 * - 增量式 k-d 树地图管理（ikd-Tree）
 * - 多线程时间同步（LiDAR 和 IMU）
 *
 * 主要依赖：
 * - ROS 2 (rclcpp) - 节点、订阅/发布、定时器
 * - PCL (Point Cloud Library) - 点云处理、滤波、转换
 * - Eigen - 线性代数运算（向量、矩阵、四元数）
 * - so3_math - SO(3) 群运算（旋转矩阵、四元数转换）
 * - ikd-Tree - 增量式 k-d 树，用于地图最近邻搜索
 * - IMU_Processing - IMU 预积分和状态预测
 *
 * 版权声明：基于 LOAM 算法实现，原作者 Ji Zhang (CMU)
 * 修改者：Livox Technology
 * 保留所有权利，遵循 BSD 3-Clause 许可证
 */

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME (0.1)                // 初始化时间阈值（秒），超过此时间后 EKF 才认为收敛
#define LASER_POINT_COV (0.001)        // 激光点测量噪声协方差
#define MAXN (720000)                  // 时间日志数组最大长度
#define PUBFRAME_PERIOD (20)           // 发布帧的周期（每 N 次处理发布一次）
bool drift_log_en = false;

/*** 时间记录变量 ***/
double kdtree_incremental_time = 0.0;  // k-d 树增量更新时间
double kdtree_search_time = 0.0;       // k-d 树搜索时间
double kdtree_delete_time = 0.0;       // k-d 树删除时间
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;  // 匹配、求解、构建 H 矩阵时间
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;  // k-d 树大小变化和操作计数
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false;  // 运行日志、PCD保存、时间同步使能

/**************************/

float res_last[100000] = {0.0};       // 上次迭代的残差数组
float DET_RANGE = 300.0f;            // 检测范围（米）
const float MOV_THRESHOLD = 1.5f;    // 移动阈值（用于局部地图移动判断）
double time_diff_lidar_to_imu = 0.0; // LiDAR 到 IMU 的时间偏移

mutex mtx_buffer;                    // 缓冲区互斥锁
condition_variable sig_buffer;       // 缓冲区条件变量

string root_dir = ROOT_DIR;          // 根目录
string map_file_path, lid_topic, imu_topic;  // 地图文件路径、话题名

FILE *drift_log_fp = nullptr;
V3D prev_pos_drift = V3D::Zero();
M3D prev_rot_drift = M3D::Identity();
bool drift_log_first = true;

double res_mean_last = 0.05, total_residual = 0.0;  // 残差统计
int global_match_count = 0;
int safe_update_blocked = 0;
double diag_imu_time = 0, diag_downsample_time = 0, diag_ekf_time = 0, diag_map_time = 0, diag_pub_time = 0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;  // 最新时间戳
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;  // 协方差参数
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;  // 滤波器和 FOV 参数
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
bool point_selected_surf[100000] = {0};  // 点是否被选为平面点
bool lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited;  // 状态标志
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;  // 发布使能

vector<vector<int>> pointSearchInd_surf;   // 最近邻搜索索引
vector<BoxPointType> cub_needrm;           // 待移除的地图立方体
vector<PointVector> Nearest_Points;        // 每个点的最近邻点集合
vector<double> extrinT(3, 0.0);            // LiDAR 到 IMU 的外参平移
vector<double> extrinR(9, 0.0);            // LiDAR 到 IMU 的外参旋转
deque<double> time_buffer;                 // 时间戳缓冲队列
deque<PointCloudXYZI::Ptr> lidar_buffer;   // 激光点云缓冲队列
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;  // IMU 数据缓冲队列

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());     // 从地图中提取的特征点
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());  // 去畸变后的特征点
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  // 体素下采样后的点（body 系）
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI()); // 体素下采样后的点（world 系）
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1)); // 法向量集合
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));  // 用于匹配的原始点
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));  // 对应法向量
PointCloudXYZI::Ptr _featsArray;  // 特征点数组

pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 表面特征下采样滤波器
pcl::VoxelGrid<PointType> downSizeFilterMap;   // 地图下采样滤波器

// ===== Task 1: 新增全局 ikd-Tree 变量和 PCD 加载函数 =====
// 目的：在 FAST-LIO 内部加载全局先验地图，使 EKF 能够同时利用局部地图和全局地图
// 进行 scan matching，从根本上解决 LiDAR 中断后无法恢复的问题。
// 
// 实现方式：
// - global_ikdtree: 用于存储全局先验地图（只读，不参与增量更新）
// - global_map_loaded: 标记全局地图是否已加载
// - load_global_map(): 加载 PCD 文件，VoxelGrid 降采样后构建 ikd-Tree
KD_TREE ikdtree;  // 增量式 k-d 树，用于地图存储和最近邻搜索
KD_TREE global_ikdtree;          // 全局先验地图 (只读)
bool global_map_loaded = false;  // 全局地图是否已加载

// ===== Task 1: 加载全局先验地图 PCD 文件 =====
// 参数: pcd_path - PCD 文件路径, voxel_size - 降采样体素大小 (米)
// 流程: 加载 PCD -> 降采样 -> 构建 ikd-Tree -> 设置 global_map_loaded = true
// 注意: 此函数在构造函数中被调用，仅执行一次。全局地图是静态的，不参与 map_incremental
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

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);   // body 系 X 轴端点
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);  // world 系 X 轴端点
V3D euler_cur;                                // 当前欧拉角
V3D position_last(Zero3d);                    // 上次位置
V3D Lidar_T_wrt_IMU(Zero3d);                 // LiDAR 相对 IMU 的平移
M3D Lidar_R_wrt_IMU(Eye3d);                  // LiDAR 相对 IMU 的旋转

/*** EKF 输入输出 ***/
MeasureGroup Measures;                        // 测量数据组（LiDAR + IMU）
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;  // 迭代 EKF 滤波器
state_ikfom state_point;                      // 当前状态
vect3 pos_lid;                                // LiDAR 位置（world 系）

nav_msgs::msg::Path path;                     // 轨迹路径消息
nav_msgs::msg::Odometry odomAftMapped;        // 里程计消息
geometry_msgs::msg::Quaternion geoQuat;       // 姿态四元数
geometry_msgs::msg::PoseStamped msg_body_pose; // 姿态位姿消息

shared_ptr<Preprocess> p_pre(new Preprocess());    // 预处理对象
shared_ptr<ImuProcess> p_imu(new ImuProcess());    // IMU 处理对象

/**
 * @brief 信号处理函数
 * @param sig 信号编号
 *
 * 捕获 SIGINT 信号（Ctrl+C），设置退出标志，通知所有等待线程，关闭 ROS。
 */
void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

/**
 * @brief 将当前 LIO 状态写入日志文件
 * @param fp 文件指针
 *
 * 记录状态包括：时间戳、角度、位置、速度、陀螺仪偏置、加速度计偏置、重力向量。
 */
inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));  // 从旋转矩阵计算欧拉角
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);  // 相对时间
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // 角度 (roll, pitch, yaw)
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));  // 位置
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // 角速度（占位）
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));  // 速度
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // 加速度（占位）
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));  // 陀螺仪偏置
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));  // 加速度计偏置
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]);  // 重力向量
    fprintf(fp, "\r\n");
    fflush(fp);
}

/**
 * @brief 将点从 body 系转换到 world 系（使用给定状态）
 * @param pi 输入点（body 系）
 * @param po 输出点（world 系）
 * @param s 状态结构体（包含旋转、平移、LiDAR 外参）
 */
void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);  // body 系点向量
    // 变换公式: p_global = s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;  // 保留强度信息
}

/**
 * @brief 将点从 body 系转换到 world 系（使用全局状态 state_point）
 * @param pi 输入点（body 系）
 * @param po 输出点（world 系）
 */
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

/**
 * @brief 将 Eigen 向量从 body 系转换到 world 系（模板函数）
 * @param pi 输入向量（body 系）
 * @param po 输出向量（world 系）
 */
template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

/**
 * @brief 将点从 body 系转换到 world 系（保留 RGB 信息）
 * @param pi 输入点（body 系）
 * @param po 输出点（world 系）
 */
void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

/**
 * @brief 将点从 LiDAR 坐标系转换到 IMU 坐标系（body 系）
 * @param pi 输入点（LiDAR 系）
 * @param po 输出点（IMU body 系）
 *
 * 只应用 LiDAR 到 IMU 的外参变换，不包含全局位姿。
 */
void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    // 变换公式: p_body_imu = offset_R_L_I * p_body_lidar + offset_T_L_I
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

/**
 * @brief 从 ikd-Tree 获取已删除的点并缓存到 _featsArray
 *
 * 在局部地图移动时，需要将删除区域内的点重新加入到点云缓存中，
 * 以便后续可能的重用或保存。
 */
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);  // 从 ikd-Tree 获取已删除点
    for (int i = 0; i < points_history.size(); i++)
        _featsArray->push_back(points_history[i]);   // 缓存到全局数组
}

/**
 * @brief 局部地图体素分割
 *
 * 根据当前 LiDAR 位置判断局部地图立方体是否需要移动。
 * 如果 LiDAR 靠近某个边界，则向该方向扩展地图，并删除远离方向的旧地图。
 * 使用 ikd-Tree 的 Delete_Point_Boxes 批量删除指定立方体内的点。
 */
BoxPointType LocalMap_Points;        // 当前局部地图边界
bool Localmap_Initialized = false;   // 局部地图是否已初始化
void lasermap_fov_segment()
{
    cub_needrm.clear();                // 清空待删除立方体列表
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);  // 计算 world 系 X 轴端点
    V3D pos_LiD = pos_lid;  // LiDAR 在 world 系的位置

    // 首次进入：初始化局部地图立方体为中心在 LiDAR 位置的立方体
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    // 计算 LiDAR 到当前地图各个边界的距离
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);  // 到最小边界距离
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);  // 到最大边界距离
        // 如果距离小于阈值，需要移动地图
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;  // 无需移动

    // 计算新地图边界和待删除的旧地图区域
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    // 移动距离：保证重叠区域至少为 MOV_THRESHOLD * DET_RANGE
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));

    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        // 靠近最小边界：向负方向移动
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;  // 待删除区域：原最大边界附近
            cub_needrm.push_back(tmp_boxpoints);
        }
        // 靠近最大边界：向正方向移动
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;  // 待删除区域：原最小边界附近
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;  // 更新地图边界

    points_cache_collect();  // 缓存将被删除的点
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);  // 批量删除
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

/**
 * @brief 标准 ROS PointCloud2 消息回调函数
 * @param msg 接收到的点云消息（智能指针）
 *
 * 1. 加锁保护缓冲区
 * 2. 检查时间回环（新消息时间早于上次）
 * 3. 调用 p_pre->process 进行预处理（去畸变、特征提取）
 * 4. 将点云和时间戳加入缓冲区
 * 5. 通知条件变量（唤醒处理线程）
 */
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    mtx_buffer.lock();
    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();

    // 检测时间回环（例如 rosbag 播放时）
    if (cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);          // 预处理：去畸变、特征提取
    lidar_buffer.push_back(ptr);       // 加入点云缓冲队列
    time_buffer.push_back(cur_time);   // 加入时间戳缓冲队列
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;  // 记录预处理时间
    mtx_buffer.unlock();
    sig_buffer.notify_all();  // 通知等待的处理线程
}

/**
 * @brief Livox 自定义消息回调函数
 * @param msg Livox 自定义点云消息
 *
 * 功能与 standard_pcl_cbk 类似，但包含 Livox 特定的时间同步逻辑。
 * 如果启用时间同步（time_sync_en），会自动计算 LiDAR 和 IMU 之间的时间偏移。
 */
double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count++;

    if (cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    last_timestamp_lidar = cur_time;

    // 检查 IMU 和 LiDAR 时间是否同步（未启用同步时打印警告）
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    // 自动时间同步：计算 LiDAR 相对 IMU 的时间偏移
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);  // 使用 LiDAR 时间戳

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

/**
 * @brief IMU 消息回调函数
 * @param msg_in 接收到的 IMU 消息（角速度、加速度）
 *
 * 1. 复制消息并应用时间偏移校正
 * 2. 检查时间回环
 * 3. 加入 IMU 缓冲队列
 * 4. 通知条件变量
 *
 * 时间同步有两种模式：
 * - 手动设置 time_diff_lidar_to_imu 参数
 * - 自动同步（通过 time_sync_en 和 timediff_lidar_wrt_imu）
 */
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    // 应用手动设置的时间偏移
    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    // 如果启用自动同步且偏移较大，应用自动计算的偏移
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    // 检测时间回环
    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);  // 加入 IMU 缓冲队列
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

/**
 * @brief 同步 LiDAR 和 IMU 数据包
 * @param meas 输出的测量组（包含点云和 IMU 序列）
 * @return true 成功同步；false 数据不足或时间未到
 *
 * 同步逻辑：
 * 1. 从缓冲区取出一个 LiDAR scan（如果尚未取出）
 * 2. 计算该 scan 的结束时间（基于最后一个点的曲率/时间偏移）
 * 3. 收集所有时间戳小于 scan 结束时间的 IMU 数据
 * 4. 确保 IMU 数据覆盖整个 scan 期间
 *
 * 注意：lidar_pushed 标志确保每个 scan 只取一次。
 */
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;  // 任一缓冲区为空则无法同步
    }

    /*** 取出一个 LiDAR scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() <= 1)
        {
            lidar_buffer.pop_front();  // 点太少，丢弃
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        // scan 结束时间 = 开始时间 + 最后一个点的相对时间（曲率字段存储时间偏移，单位 ms）
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        lidar_pushed = true;
    }

    // 如果 IMU 最新时间早于 scan 结束时间，等待更多 IMU 数据
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** 收集 IMU 数据 ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());  // 加入 IMU 序列
        imu_buffer.pop_front();                  // 从队列移除
    }

    lidar_buffer.pop_front();    // 移除已处理的 LiDAR scan
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

/**
 * @brief 增量式地图更新
 *
 * 将当前帧下采样后的特征点加入地图 k-d 树。
 * 包含降采样逻辑：将点投影到网格，每个网格只保留离中心最近的点。
 *
 * 算法流程：
 * 1. 遍历所有下采样点
 * 2. 将点从 body 系转换到 world 系
 * 3. 如果 EKF 已初始化且该点有最近邻搜索记录：
 *    - 检查该点是否已在网格中有更近的点（降采样）
 *    - 检查该点是否与现有点距离过近（避免重复）
 * 4. 将需要添加的点批量加入 ikd-Tree
 */
int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;              // 需要添加到地图的点
    PointVector PointNoNeedDownsample;   // 降采样后无需下采样的点（已是最密）
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++)
    {
        /* 将点从 body 系变换到 world 系 */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));

        /* 判断是否需要加入地图 */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)  // 有最近邻且 EKF 已初始化
        {
            const PointVector &points_near = Nearest_Points[i];  // 最近邻点集
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;

            // 计算该点所在降采样网格的中心点
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;

            float dist = calc_dist(feats_down_world->points[i], mid_point);  // 到网格中心的距离

            // 如果最近邻的第一个点不在同一网格内（偏移超过半个网格），则无需降采样检查
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);  // 直接加入，不降采样
                continue;
            }

            // 检查该网格内是否已有更近的点（降采样判断）
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;  // 网格内已有更近的点，不添加
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else  // EKF 未初始化或无最近邻（初始阶段），直接添加
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    // 批量添加点（启用降采样检查），返回实际添加的点数
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    // 添加无需降采样的点（不进行降采样检查）
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();  // 总添加数
    kdtree_incremental_time = omp_get_wtime() - st_time;  // 记录时间
}

// 全局点云缓冲（用于发布和保存）
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

/**
 * @brief 发布 world 系下的激光点云（完整分辨率或下采样）
 * @param pubLaserCloudFull 发布器
 *
 * 将当前帧点云转换到 world 坐标系并发布。
 * 支持两种模式：密集发布（feats_undistort）或下采样发布（feats_down_body）。
 * 如果启用 pcd_save_en，同时将点云累积到 pcl_wait_save 供后续保存。
 */
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        // 将 body 系点转换到 world 系
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);  // 使用 scan 结束时间
        laserCloudmsg.header.frame_id = "camera_init";  // 世界坐标系（初始化坐标系）
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;  // 递减计数，控制发布频率
    }

    /**************** 保存地图 ****************/
    /* 注意事项：
     * 1. 确保有足够的内存
     * 2. PCD 保存会显著影响实时性能
     */
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;  // 累积到全局缓冲
    }
}

/**
 * @brief 发布 IMU body 系下的点云
 * @param pubLaserCloudFull_body 发布器
 *
 * 将点云转换到 IMU body 坐标系（LiDAR 外参变换）并发布。
 */
void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";  // IMU body 坐标系
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

/**
 * @brief 发布有效特征点（参与 EKF 更新的点）
 * @param pubLaserCloudEffect 发布器
 */
void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

/**
 * @brief 发布当前地图点云
 * @param pubLaserCloudMap 发布器
 */
void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudMap);
}

/**
 * @brief 将状态位姿填充到 ROS 消息中
 * @param out 输出的 Pose 或 Odometry 消息
 */
template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

/**
 * @brief 发布里程计和 TF 变换
 * @param pubOdomAftMapped 里程计发布器
 * @param tf_br TF 广播器
 *
 * 发布包含位置、姿态和协方差的里程计消息，并广播 world -> body 的 TF。
 * 协方差来自 EKF 的状态协方差矩阵 P（6x6，位置和姿态部分重排）。
 */
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
                      std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br,
                      const rclcpp::Time &stamp)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    //odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    odomAftMapped.header.stamp = stamp;
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);

    // 从 EKF 获取协方差矩阵并重排到 odometry 消息格式
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;  // 将姿态和位置部分交换（EKF 内部顺序）
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    // 广播 TF: world (camera_init) -> body
    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

/**
 * @brief 发布轨迹路径
 * @param pubPath 路径发布器
 *
 * 将当前位姿加入路径序列，每 10 帧发布一次，避免 RViz 崩溃。
 */
void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)  // 降频发布
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

// ===== Task 3: 提取单树搜索+平面拟合为辅助函数 =====
// 目的: 将 h_share_model 中的搜索->平面拟合->筛选逻辑提取为独立函数
// 原因: 避免后续双树搜索时重复代码，便于 Task 4 实现双树融合
// 说明: 此重构不改变功能，仅提取代码。Task 4 将在此基础上扩展为双树搜索

struct SearchResult {
    bool valid;
    float norm_x, norm_y, norm_z;
    float residual;
};

SearchResult search_and_fit(KD_TREE &tree,
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

/**
 * @brief EKF 测量模型（共享函数）
 * @param s 状态向量
 * @param ekfom_data EKF 数据结构（输出 H 矩阵和测量残差）
 *
 * 核心功能：对每个下采样点，在 ikd-Tree 地图中搜索最近邻，
 * 拟合平面，计算点到平面的残差和测量雅可比矩阵 H。
 *
 * 算法步骤：
 * 1. 清空输出缓冲区（laserCloudOri, corr_normvect）
 * 2. 并行循环（如果 MP_EN 启用）所有下采样点：
 *    a. 将点从 body 变换到 world
 *    b. 在地图中搜索 K 个最近邻
 *    c. 使用 esti_plane 拟合平面，计算法向量和距离
 *    d. 根据平面拟合质量筛选有效点
 * 3. 收集有效点，计算平均残差
 * 4. 构建测量雅可比矩阵 H 和测量向量 h：
 *    H 的每一行对应一个点的测量方程
 *    测量值为点到平面的有符号距离
 */
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

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

    int total_feat_num = effct_feat_num;

    if (global_map_loaded && ekfom_data.converge)
    {
        int global_idx = 0;
        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i]) continue;

            PointType &point_world = feats_down_world->points[i];
            PointType &point_body = feats_down_body->points[i];

            V3D p_body(point_body.x, point_body.y, point_body.z);
            SearchResult sr_global = search_and_fit(global_ikdtree, point_world, NUM_MATCH_POINTS,
                                                     ekfom_data.converge, p_body.norm());
            if (sr_global.valid)
            {
                int idx = total_feat_num + global_idx;
                laserCloudOri->points[idx] = feats_down_body->points[i];
                corr_normvect->points[idx].x = sr_global.norm_x;
                corr_normvect->points[idx].y = sr_global.norm_y;
                corr_normvect->points[idx].z = sr_global.norm_z;
                corr_normvect->points[idx].intensity = sr_global.residual;
                global_idx++;
            }
        }
        global_match_count = global_idx;
        total_feat_num += global_idx;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

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

/**
 * @brief 激光建图节点类
 */
class LaserMappingNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param options ROS 节点选项
     *
     * 主要步骤：
     * 1. 声明并读取 ROS 参数（话题名、滤波参数、外参等）
     * 2. 初始化 EKF 和预处理对象
     * 3. 创建订阅者（LiDAR、IMU）和发布者（点云、里程计、路径）
     * 4. 创建定时器（主处理循环）
     */
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        // 声明参数（默认值）
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<double>("preprocess.max_range", 100.f);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("drift_log_enable", false);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<string>("global_map.global_map_path", "");
        this->declare_parameter<double>("global_map.global_map_voxel_size", 0.4);
        this->declare_parameter<bool>("global_map.global_map_enabled", false);

        // 读取参数到成员变量
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min, 0.5);
        this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
        this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
        this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
        this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
        this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
        this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<double>("preprocess.max_range", p_pre->max_scan_range, 100.f);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("drift_log_enable", drift_log_en, false);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        // ===== Task 2: 读取全局地图参数并加载 =====
        // 参数来源: config/mid360.yaml 的 global_map 区段
        // - global_map_path: PCD 文件路径 (默认空)
        // - global_map_voxel_size: 降采样体素大小 (默认 0.4m)
        // - global_map_enabled: 是否启用 (默认 false，向后兼容)
        // 加载条件: global_map_enabled && !global_map_path.empty()
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

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "camera_init";

        /*** 变量定义 ***/
        int effect_feat_num = 0, frame_num = 0;
        double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        bool flg_EKF_converged, EKF_stop_flg = 0;

        // FOV 参数：视野范围（用于地图裁剪）
        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        // 设置 LiDAR 到 IMU 的外参
        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        // 初始化 EKF
        double epsi[23] = {0.001};
        fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** 调试日志文件 ***/
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");

        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

        if (drift_log_en)
        {
            string drift_log_path = root_dir + "/Log/drift_diagnostic.csv";
            drift_log_fp = fopen(drift_log_path.c_str(), "w");
            if (drift_log_fp)
            {
                fprintf(drift_log_fp, "timestamp,"
                    "pos_x,pos_y,pos_z,"
                    "rot_x,rot_y,rot_z,"
                    "vel_x,vel_y,vel_z,"
                    "bg_x,bg_y,bg_z,"
                    "ba_x,ba_y,ba_z,"
                    "grav_x,grav_y,grav_z,"
                    "delta_pos_x,delta_pos_y,delta_pos_z,"
                    "delta_rot_x,delta_rot_y,delta_rot_z,"
                    "P_d0,P_d1,P_d2,P_d3,P_d4,P_d5,P_d6,P_d7,P_d8,P_d9,"
                    "P_d10,P_d11,P_d12,P_d13,P_d14,P_d15,P_d16,P_d17,P_d18,P_d19,"
                    "P_d20,P_d21,P_d22,"
                    "effct_feat_num,feats_down_size,res_mean,icp_time,kdtree_size,global_match,"
                    "wall_time,kdtree_incre_time,rebuild_active,rebuild_queue,safe_blocked,"
                    "time_imu,time_ds,time_ekf,time_map,time_pub,"
                    "buf_lidar,buf_imu\n");
                fflush(drift_log_fp);
                RCLCPP_INFO(this->get_logger(), "Drift diagnostic log: %s", drift_log_path.c_str());
            }
        }

        /*** ROS 订阅发布初始化 ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20000, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 20000, standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);

        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
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
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 创建定时器，频率 100Hz（周期 10ms）
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(5000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
        if (drift_log_fp) fclose(drift_log_fp);
    }

private:
    /**
     * @brief 定时器回调函数（主处理循环）
     *
     * 每 10ms 调用一次，处理流程：
     * 1. 同步 LiDAR 和 IMU 数据包
     * 2. IMU 预积分和状态预测
     * 3. 局部地图 FOV 裁剪
     * 4. 点云下采样
     * 5. 初始化 k-d 树（首次）
     * 6. 迭代 EKF 更新（点到平面匹配）
     * 7. 增量式地图更新
     * 8. 发布点云、里程计、路径
     * 9. 记录调试日志
     */
    void timer_callback()
    {
        if (sync_packages(Measures))  // 成功同步一组数据
        {
            static int log_skip_count = 0;
            int buf_lidar = lidar_buffer.size();
            int buf_imu = imu_buffer.size();
            if (buf_lidar > 5 || buf_imu > 50)
            {
                log_skip_count++;
                if (log_skip_count <= 3 || buf_lidar > 20)
                    RCLCPP_WARN(this->get_logger(), "[BUFFER] lidar_queue=%d imu_queue=%d", buf_lidar, buf_imu);
            }
            else
            {
                log_skip_count = 0;
            }
            if (flg_reset)
            {
                RCLCPP_WARN(this->get_logger(), "reset when rosbag play back\n");
                p_imu->Reset();
                flg_reset = false;
                Measures.imu.clear();
                return;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();  // 开始时间

            /*** IMU 预积分和状态预测 ***/
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();  // 获取预测状态
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            diag_imu_time = omp_get_wtime() - t0;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            lasermap_fov_segment();
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            diag_downsample_time = t1 - t0;
            feats_down_size = feats_down_body->points.size();

            /*** 初始化地图 k-d 树（首次）**/
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

                if (!global_map_loaded)
                    return;

                if (ikdtree.Root_Node == nullptr)
                {
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    PointVector empty_pv;
                    ikdtree.set_downsample_param(filter_size_map_min);
                    ikdtree.Build(empty_pv);
                }
            }

            int featsFromMapNum = ikdtree.validnum();  // 地图中有效点数
            kdtree_size_st = ikdtree.size();           // 建图前大小

            /*** 迭代 EKF 更新 ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            // 记录 IMU 状态到调试文件
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            // 可选：展平地图用于可视化
            if (0)
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true;

            t2 = omp_get_wtime();

            /*** EKF 迭代更新（点到平面 ICP）***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            state_ikfom pre_update_state = kf.get_x();
            safe_update_blocked = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            {
                V3D dp = state_point.pos - pre_update_state.pos;
                M3D dR_mat = pre_update_state.rot.toRotationMatrix().transpose() * state_point.rot.toRotationMatrix();
                V3D drot = Log(dR_mat);
                double pos_err = dp.norm();
                double rot_err = drot.norm() * 57.3;
                safe_update_blocked = (pos_err > 0.3 || rot_err > 10.0) ? 1 : 0;
            }
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();
            diag_ekf_time = t_update_end - t_update_start;

            if (drift_log_en && drift_log_fp)
            {
                V3D rot_ang_drift(Log(state_point.rot.toRotationMatrix()));
                V3D delta_pos = state_point.pos - prev_pos_drift;
                M3D delta_rot_mat = prev_rot_drift.transpose() * state_point.rot.toRotationMatrix();
                V3D delta_rot_vec(Log(delta_rot_mat));

                fprintf(drift_log_fp, "%.6lf,"
                    "%.6lf,%.6lf,%.6lf,"
                    "%.6lf,%.6lf,%.6lf,"
                    "%.6lf,%.6lf,%.6lf,"
                    "%.6lf,%.6lf,%.6lf,"
                    "%.6lf,%.6lf,%.6lf,"
                    "%.6lf,%.6lf,%.6lf,"
                    "%.6lf,%.6lf,%.6lf,"
                    "%.6lf,%.6lf,%.6lf",
                    Measures.lidar_beg_time - first_lidar_time,
                    state_point.pos(0), state_point.pos(1), state_point.pos(2),
                    rot_ang_drift(0), rot_ang_drift(1), rot_ang_drift(2),
                    state_point.vel(0), state_point.vel(1), state_point.vel(2),
                    state_point.bg(0), state_point.bg(1), state_point.bg(2),
                    state_point.ba(0), state_point.ba(1), state_point.ba(2),
                    state_point.grav[0], state_point.grav[1], state_point.grav[2],
                    delta_pos(0), delta_pos(1), delta_pos(2),
                    delta_rot_vec(0), delta_rot_vec(1), delta_rot_vec(2));

                auto P_copy = kf.get_P();
                for (int pi = 0; pi < 23; pi++)
                {
                    fprintf(drift_log_fp, ",%.10lf", P_copy(pi, pi));
                }

                fprintf(drift_log_fp, ",%d,%d,%.8lf,%.6lf,%.6lf,%d,"
                    "%.6lf,%.6lf,%d,%d,%d,"
                    "%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%d,%d\n",
                    effct_feat_num, feats_down_size, res_mean_last,
                    t_update_end - t_update_start, (double)ikdtree.size(),
                    global_match_count,
                    omp_get_wtime(),
                    kdtree_incremental_time,
                    ikdtree.is_rebuilding() ? 1 : 0,
                    ikdtree.rebuild_queue_size(),
                    safe_update_blocked,
                    diag_imu_time, diag_downsample_time,
                    diag_ekf_time, diag_map_time, diag_pub_time,
                    (int)lidar_buffer.size(), (int)imu_buffer.size());

                fflush(drift_log_fp);

                prev_pos_drift = state_point.pos;
                prev_rot_drift = state_point.rot.toRotationMatrix();

                static int drift_log_frame = 0;
                drift_log_frame++;
                if (drift_log_frame % 10 == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "[dual-tree] local=%d global=%d total=%d/%d res=%.5f",
                        effct_feat_num, global_match_count, effct_feat_num + global_match_count,
                        feats_down_size, res_mean_last);
                }
            }

            /******* 发布里程计 *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_, this->now());

            /*** 将当前特征点增量式加入地图 k-d 树 ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            diag_map_time = t5 - t3;

            /******* 发布各种点云和路径 *******/
            publish_path(pubPath_);
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);
            diag_pub_time = omp_get_wtime() - t5;

            /*** 调试信息统计 ***/
            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                // 滑动平均统计各阶段耗时
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;

                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;

                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
                       t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);

                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

private:
    // ROS 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

/**
 * @brief 主函数
 * @param argc 参数计数
 * @param argv 参数数组
 *
 * 1. 初始化 ROS 2
 * 2. 注册信号处理函数
 * 3. 创建 LaserMappingNode 并 spin
 * 4. 节点退出后保存 PCD 地图和日志
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);  // 注册 Ctrl+C 处理

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();

    /**************** 保存地图 ****************/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);  // 二进制 PCD 格式
    }

    // 保存运行时性能日志
    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++)
        {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
                    T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i], int(s_plot5[i]),
                    s_plot6[i], int(s_plot7[i]), int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
