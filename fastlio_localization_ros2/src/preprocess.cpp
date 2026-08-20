/*
 * @file preprocess.cpp
 * @brief 点云预处理模块实现
 *
 * 本文件实现了 Preprocess 类，用于处理多种 LiDAR 点云格式并提取特征点。
 * 支持的 LiDAR 类型：
 * - AVIA: Livox  mid-40/70/100 系列（自定义消息格式）
 * - OUST64: Ouster OS-64（标准 PointCloud2）
 * - VELO16: Velodyne VLP-16（标准 PointCloud2）
 * - MID360: Livox MID360（类似 AVIA 但时间戳处理不同）
 * - HESAIxt16: Hesai HESAIxt16（标准 PointCloud2）
 * - UNILIDAR: Unilidar/宇树L1（标准 PointCloud2）
 *
 * 核心功能：
 * 1. 点云格式解析和坐标提取
 * 2. 去畸变（利用 IMU 数据或估算的扫描角速度）
 * 3. 特征提取：平面点（surf）和边缘点（edge/corner）
 * 4. 距离盲区过滤（blind）和最大距离过滤
 * 5. 点采样（point_filter_num）降采样
 *
 * 特征提取算法基于局部几何分析：
 * - 平面点：连续若干点在局部近似共面
 * - 边缘点：两点间距离突变（跳跃点）或平面与边缘交界
 *
 * 主要依赖：
 * - PCL：点云数据结构、转换、滤波
 * - Eigen：向量运算、特征值计算
 * - ROS 2：消息订阅和类型转换
 */

#include "preprocess.h"

#include <pcl/common/common.h>

#define RETURN0 0x00
#define RETURN0AND1 0x10

/**
 * @brief Preprocess 构造函数
 *
 * 初始化成员变量，设置默认参数：
 * - 激光线数 N_SCANS = 6（AVIA 默认）
 * - 盲区 blind = 0.01m
 * - 平面判定阈值（p2l_ratio, limit_maxmid 等）
 * - 角度阈值（cos160, jump_up_limit, jump_down_limit）
 * - 边缘跳跃阈值（edgea, edgeb）
 */
Preprocess::Preprocess() : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 6;
  SCAN_RATE = 10;
  group_size = 8;           // 平面检测的滑动窗口大小
  disA = 0.01;
  disA = 0.1; // B?（可能是笔误，但保留）
  p2l_ratio = 225;          // 点到直线距离比阈值，用于判断共面性
  limit_maxmid = 6.25;      // AVIA: 最大距离/中间距离比值阈值
  limit_midmin = 6.25;      // AVIA: 中间距离/最小距离比值阈值
  limit_maxmin = 3.24;      // 其他 LiDAR: 最大距离/最小距离比值阈值
  jump_up_limit = 170.0;    // 跳跃检测：向上跳跃角度阈值（度）
  jump_down_limit = 8.0;    // 跳跃检测：向下跳跃角度阈值（度）
  cos160 = 160.0;          // 边缘点夹角阈值（度）
  edgea = 2;               // 边缘跳跃距离比系数 A
  edgeb = 0.1;             // 边缘跳跃绝对差值阈值 B
  smallp_intersect = 172.5; // 小平面夹角阈值（度）
  smallp_ratio = 1.2;      // 小平面距离比阈值
  given_offset_time = false; // 是否已给定点时间偏移

  // 将角度转换为余弦值，便于后续比较
  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess()
{
}

/**
 * @brief 设置预处理参数
 * @param feat_en 是否启用特征提取
 * @param lid_type LiDAR 类型（AVIA/OUST64/VELO16/MID360）
 * @param bld 盲区距离（米）
 * @param pfilt_num 点采样间隔（每 N 点保留一个）
 */
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

/**
 * @brief 处理 Livox 自定义消息
 * @param msg Livox CustomMsg 消息
 * @param pcl_out 输出的特征点云（仅表面点）
 *
 * 直接调用 avia_handler 处理，输出 pl_surf。
 */
void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

/**
 * @brief 处理标准 PointCloud2 消息
 * @param msg PointCloud2 消息
 * @param pcl_out 输出的特征点云
 *
 * 根据 lidar_type 调用不同的处理器：
 * - OUST64: oust64_handler
 * - VELO16: velodyne_handler
 * - MID360: mid360_handler
 * - HESAIxt16: hesai_handler
 * - UNILIDAR: unilidar_handler
 * - 其他: default_handler
 */
void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
  case SEC:
    time_unit_scale = 1.e3f;
    break;
  case MS:
    time_unit_scale = 1.f;
    break;
  case US:
    time_unit_scale = 1.e-3f;
    break;
  case NS:
    time_unit_scale = 1.e-6f;
    break;
  default:
    time_unit_scale = 1.f;
    break;
  }


  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case MID360:
    mid360_handler(msg);
    break;

  case HESAIxt16:
    hesai_handler(msg);
    break;

  case UNILIDAR:
    unilidar_handler(msg);
    break;

  default:
    default_handler(msg);
    break;
  }
  *pcl_out = pl_surf;
}

/**
 * @brief Livox AVIA 点云处理函数
 * @param msg Livox CustomMsg 消息
 *
 * 处理流程：
 * 1. 清空输出缓冲（pl_surf, pl_corn, pl_full）
 * 2. 解析每个点，按 scan line 分组存入 pl_buff
 * 3. 如果启用特征提取（feature_enabled）：
 *    - 对每条 scan 调用 give_feature 进行平面/边缘检测
 *    - 提取 surf 点和 corner 点
 * 4. 否则仅进行简单滤波：盲区过滤 + 距离过滤 + 降采样
 *
 * 注意：
 * - AVIA 点的 tag 字段用于区分反射类型，只处理有效点（tag & 0x30 == 0x10 或 0x00）
 * - curvature 字段用于存储点的时间偏移（offset_time / 1e6，单位秒）
 */
void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // 按 scan line 分组缓冲
  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled)
  {
    // 遍历所有点，按 line 索引分组
    for (uint i = 1; i < plsize; i++)
    {
      if ((msg->points[i].line < N_SCANS) &&
          ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature =
            msg->points[i].offset_time / float(1000000); // 曲率字段存储时间偏移（秒）

        // 去重：如果与前一点坐标不同，则加入对应 scan 的缓冲区
        bool is_new = false;
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }

    static int count = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();

    // 对每条 scan 进行特征提取
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5)
        continue;  // 点数太少，跳过

      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;  // 最后一个点没有下一个点作为参考

      // 计算相邻点间距离（平方）和每个点的范围（平方）
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y;
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;  // 相邻点距离平方
      }
      types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;

      give_feature(pl, types);  // 核心特征提取函数
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else  // 不提取特征，仅降采样和过滤
  {
    for (uint i = 1; i < plsize; i++)
    {
      if ((msg->points[i].line < N_SCANS) &&
          ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num++;
        if (valid_num % point_filter_num == 0)  // 降采样
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000);

          // 检查坐标变化（去重）和盲区
          if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y > blind))
          {
            if (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y < max_scan_range * max_scan_range)
            {
              pl_surf.push_back(pl_full[i]);
            }
          }
        }
      }
    }
  }
}

/**
 * @brief Ouster OS-64 点云处理函数
 * @param msg PointCloud2 消息（Ouster 格式）
 *
 * Ouster 点类型包含 x,y,z,intensity,ring,t 字段。
 * 处理流程：
 * 1. 将 ROS 消息转换为 PCL 点云
 * 2. 如果启用特征提取：
 *    - 按 ring（scan 线）分组
 *    - 对每条 scan 调用 give_feature
 * 3. 否则：简单距离过滤和降采样
 *
 * 注意：Ouster 点的 t 字段是发射时间（纳秒），转换为秒存入 curvature。
 */
void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);  // ROS -> PCL 转换
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  if (feature_enabled)
  {
    // 初始化每条 scan 的缓冲区
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    // 遍历所有点，按 ring 分组
    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x +
                     pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < blind)
        continue;  // 盲区过滤

      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;  // 纳秒转秒

      if (pl_orig.points[i].ring < N_SCANS)  // 只处理前 N_SCANS 线
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    // 对每条 scan 进行特征提取
    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else  // 仅过滤和降采样
  {
    double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0)
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x +
                     pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;

      if (range < blind)
        continue;

      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t / 1e6;

      pl_surf.points.push_back(added_pt);
    }
  }
}

/**
 * @brief Velodyne VLP-16 点云处理函数
 * @param msg PointCloud2 消息（Velodyne 格式）
 *
 * Velodyne 点包含 x,y,z,intensity,ring,time 字段。
 * 时间处理：
 * - 如果点包含 time 字段（>0），直接使用
 * - 否则根据扫描角度估算时间偏移（假设恒定角速度 omega_l = 3.61 rad/s）
 *
 * 特征提取流程与 Ouster 类似，但时间偏移计算方式不同。
 */
void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;
  pl_surf.reserve(plsize);

  /*** 时间偏移计算相关变量（仅在没有点时间戳时使用）***/
  double omega_l = 3.61; // 扫描角速度（rad/s）
  std::vector<bool> is_first(N_SCANS, true);    // 每条 scan 是否首次遇到点
  std::vector<double> yaw_fp(N_SCANS, 0.0);     // 每条 scan 第一个点的方位角
  std::vector<float> yaw_last(N_SCANS, 0.0);    // 上次点的方位角
  std::vector<float> time_last(N_SCANS, 0.0);   // 上次点的时间偏移

  // 检查点是否包含 time 字段
  if (pl_orig.points[plsize - 1].time > 0)
  {
    given_offset_time = true;
  }
  else
  {
    given_offset_time = false;
    // 估算扫描起始和结束方位角（用于计算相对时间）
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  if (feature_enabled)
  {
    // 初始化缓冲区
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    // 遍历所有点，按 ring 分组并计算时间偏移
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS)
        continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // 单位：ms

      if (!given_offset_time)
      {
        // 计算当前点的方位角
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          yaw_fp[layer] = yaw_angle;      // 记录该 scan 的起始方位角
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // 根据方位角差计算相对时间（假设匀速旋转）
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        // 保证时间单调递增
        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    // 对每条 scan 进行特征提取
    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2)
        continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else  // 仅过滤和降采样
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0;

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // 计算时间偏移
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind)
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

/**
 * @brief Livox MID360 点云处理函数
 * @param msg PointCloud2 消息（MID360 格式）
 *
 * MID360 与 AVIA 类似，但点类型为 LivoxPointXyzitl（包含 line 和 offset_time）。
 * 该 handler 总是启用时间偏移计算（given_offset_time = false）。
 * 只输出 surf 点，不进行特征提取（除非 feature_enabled 在其他地方启用）。
 */
void Preprocess::mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<livox_ros::LivoxPointXyzitl> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;
  pl_surf.reserve(plsize);

  /*** 时间偏移计算变量 ***/
  double omega_l = 3.61;
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);
  std::vector<float> yaw_last(N_SCANS, 0.0);
  std::vector<float> time_last(N_SCANS, 0.0);

  given_offset_time = false;  // MID360 需要自己计算时间偏移

  // 计算首尾点方位角（用于验证）
  double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
  double yaw_end = yaw_first;
  int layer_first = pl_orig.points[0].line;
  for (uint i = plsize - 1; i > 0; i--)
  {
    if (pl_orig.points[i].line == layer_first)
    {
      yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
      break;
    }
  }

  // 遍历所有点，按 line 分组并计算时间偏移
  for (uint i = 0; i < plsize; ++i)
  {
    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = 0.;

    int layer = pl_orig.points[i].line;
    double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

    if (is_first[layer])
    {
      yaw_fp[layer] = yaw_angle;
      is_first[layer] = false;
      added_pt.curvature = 0.0;
      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
      continue;
    }

    // 计算时间偏移（与 Velodyne 相同逻辑）
    if (yaw_angle <= yaw_fp[layer])
    {
      added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
    }
    else
    {
      added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
    }

    if (added_pt.curvature < time_last[layer])
      added_pt.curvature += 360.0 / omega_l;

    yaw_last[layer] = yaw_angle;
    time_last[layer] = added_pt.curvature;

    // 盲区过滤
    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind)
    {
      pl_surf.push_back(std::move(added_pt));
    }
  }
}


/**
 * @brief Hesai HESAIxt16 点云处理函数
 * @param msg PointCloud2 消息（HESAIxt16 格式）
 */
void Preprocess::hesai_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg) 
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<hesai_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);
  
  /*** These variables only works when no point timestamps given ***/
  double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
  std::vector<bool> is_first(N_SCANS,true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
  std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
  std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
  /*****************************************************************/

  if (pl_orig.points[plsize - 1].timestamp > 0)
  {
    given_offset_time = true;
  }
  else
  {
    given_offset_time = false;
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end  = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  double time_head = pl_orig.points[0].timestamp;
  
  for (int i = 0; i < plsize; i++)
  {
    PointType added_pt;
    // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
    
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f; // time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;
    if (!given_offset_time)
    {
      int layer = pl_orig.points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer])
      {
        // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer]=yaw_angle;
          is_first[layer]=false;
          added_pt.curvature = 0.0;
          yaw_last[layer]=yaw_angle;
          time_last[layer]=added_pt.curvature;
          continue;
      }

      // compute offset time
      if (yaw_angle <= yaw_fp[layer])
      {
        added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
      }
      else
      {
        added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
      }

      if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

      yaw_last[layer] = yaw_angle;
      time_last[layer]=added_pt.curvature;
    }

    if (i % point_filter_num == 0)
    {
      if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
      {
        pl_surf.points.push_back(added_pt);
      }
    }
  }
  
}

/**
 * @brief Unilidar/宇树L1 点云处理函数
 * @param msg PointCloud2 消息（Unilidar/宇树L1 格式）
 */
void Preprocess::unilidar_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<unilidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    // 检测时间戳是否有效，启用运动去畸变
    if (pl_orig.points[plsize - 1].time > 0) {
      given_offset_time = true;
    }

    pl_surf.reserve(plsize);

    std::cout << "plsize = " << plsize << ", given_offset_time = " << given_offset_time << std::endl;
    int countElimnated = 0;
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;

      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      
      added_pt.intensity = pl_orig.points[i].intensity;

      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;

      // 显式过滤无回波的 (0,0,0) 占位点
      if (added_pt.x == 0.0f && added_pt.y == 0.0f && added_pt.z == 0.0f)
      {
        countElimnated++;
        continue;
      }

      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
      {
        pl_surf.points.push_back(added_pt);
      }
      else
      {
        countElimnated++;
      }
    }

    std::cout << "pl_surf.size() = " << pl_surf.size() << ", countElimnated = " << countElimnated << std::endl;
    
}



/**
 * @brief 默认点云处理函数
 * @param msg PointCloud2 消息
 *
 * 处理不认识的 LiDAR 类型，仅进行最基础的过滤：
 * - 盲区过滤
 * - 降采样（point_filter_num）
 */
void Preprocess::default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<pcl::PointXYZI> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;
  pl_surf.reserve(plsize);

  for (uint i = 0; i < plsize; ++i)
  {
    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = 0.;

    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
    {
      pl_surf.push_back(std::move(added_pt));
    }
  }
}

/**
 * @brief 特征提取核心函数
 * @param pl 单条 scan 的点云
 * @param types 点的属性数组（输入：range, dista；输出：ftype, edj, angle, intersect）
 *
 * 算法概述：
 * 1. 跳过盲区点
 * 2. 使用 plane_judge 检测平面区域（返回 1 表示平面，0 表示非平面，2 表示盲区）
 * 3. 根据连续平面的法向量变化检测 Edge_Plane（边缘与平面交界）
 * 4. 边缘点检测：通过距离跳跃（edge_jump_judge）和角度条件
 * 5. 小平面合并：对孤立的小平面区域进行合并
 * 6. 点类型标记：
 *    - Real_Plane: 确定的平面点
 *    - Poss_Plane: 可能的平面点（端点）
 *    - Edge_Jump: 跳跃边缘点
 *    - Edge_Plane: 平面-边缘交界点
 *    - Wire: 细线点
 * 7. 根据 point_filter_num 对平面点进行降采样输出
 *
 * 状态机：last_state 记录上一次是否为平面区域，用于 Edge_Plane 判断。
 */
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  // 跳过盲区点
  while (types[head].range < blind)
  {
    head++;
  }

  // Surf 点检测参数
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());  // 当前平面法向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());  // 上一个平面法向量

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;  // 0:非平面, 1:平面
  int plane_type;

  // 第一轮：遍历 scan，检测连续平面区域
  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    // 判断从 i 开始的连续点是否在同一个平面上
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1)  // 是平面区域
    {
      // 标记平面点：两端为 Poss_Plane，中间为 Real_Plane
      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }

      // 如果前一段也是平面，检查法向量变化：若夹角余弦在 (-0.707, 0.707) 之间（角度 > 45°），则为 Edge_Plane
      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;  // 跳过已处理区域
      last_state = 1;
    }
    else  // plane_type == 2（盲区）或 0（非平面）
    {
      i = i_nex;
      last_state = 0;
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  // 第二轮：检测边缘跳跃点（Edge_Jump）和细线点（Wire）
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++)
  {
    if (types[i].range < blind || types[i].ftype >= Real_Plane)
    {
      continue;  // 跳过盲区点或已标记为平面的点
    }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
    {
      continue;  // 避免除零
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++)
    {
      int m = -1;
      if (j == 1)
      {
        m = 1;
      }

      if (types[i + m].range < blind)
      {
        if (types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;  // 超出有效距离
        }
        else
        {
          types[i].edj[j] = Nr_blind;  // 盲区
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;

      // 计算向量夹角余弦
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if (types[i].angle[j] < jump_up_limit)  // 夹角大于阈值（接近180°）
      {
        types[i].edj[j] = Nr_180;
      }
      else if (types[i].angle[j] > jump_down_limit)  // 夹角很小（接近0°）
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    // 计算左右向量夹角
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

    // Edge_Jump 检测：多种几何条件组合
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor)
      {
        types[i].ftype = Wire;  // 细线点
      }
    }
  }

  // 第三轮：小平面区域合并（将相邻的 Nor 点合并为 Real_Plane）
  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < plsize2; i++)
  {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
    {
      continue;
    }

    if (types[i].ftype == Nor)
    {
      // 计算相邻距离比
      if (types[i - 1].dista > types[i].dista)
      {
        ratio = types[i - 1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i - 1].dista;
      }

      // 如果夹角小且距离比小于阈值，则三点均为平面点
      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if (types[i - 1].ftype == Nor)
        {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor)
        {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  // 第四轮：生成输出点云（pl_surf, pl_corn），并应用降采样
  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)  // 平面点
    {
      if (last_surface == -1)
      {
        last_surface = j;
      }

      // 累计 point_filter_num 个连续平面点后输出一个平均点
      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else  // 非平面点（边缘点）
    {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
      {
        pl_corn.push_back(pl[j]);  // 直接加入边缘点云
      }
      // 如果之前有未输出的平面点序列，计算平均值并输出
      if (last_surface != -1)
      {
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

/**
 * @brief 发布点云辅助函数（未使用）
 */
void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

/**
 * @brief 平面检测函数
 * @param pl 点云
 * @param types 点属性数组
 * @param i_cur 起始点索引
 * @param i_nex 输出：平面区域结束索引（包含）
 * @param curr_direct 输出：平面法向量
 * @return 0: 非平面；1: 平面；2: 包含盲区点
 *
 * 算法步骤：
 * 1. 计算初始组内距离阈值 group_dis = (disA * range + disB)^2
 * 2. 从 i_cur 开始，向后续点扩展，直到两点距离 >= group_dis
 * 3. 收集所有点对距离到 disarr
 * 4. 计算垂直距离 leng_wid（点到两点连线的垂直距离平方和的最大值）
 * 5. 判断共面性：
 *    - 如果 two_dis^2 / leng_wid < p2l_ratio，说明点离直线很近 → 非平面（返回 0）
 *    - 否则计算 disarr 中的距离分布
 * 6. 对于 AVIA：比较最大/中间 和 中间/最小 距离比
 *    对于其他 LiDAR：比较最大/最小 距离比
 * 7. 如果比值超过阈值，认为距离分布不均匀 → 非平面
 * 8. 否则认为是平面，计算法向量并归一化
 *
 * 注：disA, disB, p2l_ratio, limit_maxmid 等为经验阈值，需根据 LiDAR 特性调整。
 */
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex,
                            Eigen::Vector3d &curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;  // 距离阈值（平方）

  double two_dis;  // 首尾点距离平方
  vector<double> disarr;
  disarr.reserve(20);

  // 首先加入初始 group_size 个点对距离
  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;  // 包含盲区点
    }
    disarr.push_back(types[i_nex].dista);
  }

  // 继续向后扩展，直到两点距离超过 group_dis
  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
      break;

    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    if (two_dis >= group_dis)
    {
      break;  // 已扩展到足够远的点
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  // 计算垂直距离：遍历 i_cur+1 到 i_nex-1 的点，求它们到向量 (i_cur, i_nex) 的垂直距离平方
  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size()))
      break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    // 叉积：v2 = v1 × (pl[i_nex] - pl[i_cur])
    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid)
    {
      leng_wid = lw;
    }
  }

  // 如果所有点都紧挨着首尾连线（垂直距离很小），说明是直线（非平面）
  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;  // 非平面
  }

  // 将 disarr 从大到小排序（冒泡排序）
  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        double tmp = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = tmp;
      }
    }
  }

  // 如果次小距离接近 0，说明点分布不均匀
  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  // 根据 LiDAR 类型使用不同的距离比阈值
  if (lidar_type == AVIA)
  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;  // 距离分布太不均匀
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  // 通过平面检测：计算法向量（vx,vy,vz 是首尾点差，垂直于平面）
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

/**
 * @brief 边缘跳跃点检测函数
 * @param pl 点云
 * @param types 点属性数组
 * @param i 当前点索引
 * @param nor_dir 方向（Prev=0 向前检查，Next=1 向后检查）
 * @return true 是边缘跳跃点；false 否则
 *
 * 用于验证 edge jump 候选点。
 * 条件：
 * 1. 检查 nor_dir 方向前 1-2 个点是否在盲区内
 * 2. 取两个较远的距离 d1, d2（交换保证 d1 >= d2）
 * 3. 判断 d1 > edgea * d2 或 d1 - d2 > edgeb
 *    若成立，说明距离跳跃不够陡峭，不是边缘点（返回 false）
 *    否则认为是边缘跳跃点（返回 true）
 */
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)  // 向前检查
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind)
    {
      return false;  // 前方点不可靠
    }
  }
  else if (nor_dir == 1)  // 向后检查
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind)
    {
      return false;
    }
  }
  double d1 = types[i + nor_dir - 1].dista;  // 相邻点距离平方
  double d2 = types[i + 3 * nor_dir - 2].dista;  // 隔一个点的距离平方
  double d;

  // 保证 d1 >= d2
  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  // 如果距离比过大或差值过大，认为不是理想的边缘跳跃
  if (d1 > edgea * d2 || (d1 - d2) > edgeb)
  {
    return false;
  }

  return true;
}
