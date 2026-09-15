/**
 * @file IMU_Processing.hpp
 * @brief IMU数据处理与点云去畸变模块
 *
 * 本文件实现了ImuProcess类，负责：
 * - IMU数据的预处理和初始化
 * - 重力方向估计和IMU偏置校准
 * - 点云运动畸变补偿（基于IMU预积分）
 * - 状态预测（EKF的前向传播）
 *
 * 该模块是FAST-LIO系统中IMU-激光雷达融合的关键组件，在每帧激光扫描到达时，
 * 使用IMU数据预测传感器运动，并对点云进行运动补偿，消除高速运动造成的畸变。
 *
 * 主要算法：
 * 1. IMU初始化：在线估计重力方向、陀螺仪偏置、加速度计和陀螺仪的协方差
 * 2. 前向传播：使用一阶欧拉积分预测IMU姿态、位置和速度
 * 3. 反向去畸变：根据IMU姿态历史，将每个点云样本补偿到统一的参考时间戳
 *
 * 坐标系约定：
 * - IMU坐标系：前向(X)、左向(Y)、上向(Z)
 * - LiDAR坐标系：由外参矩阵 T_lidar_wrt_IMU 定义
 * - 世界坐标系：初始帧的IMU坐标系（Z向上）
 *
 * 作者:  FAST-LIO团队
 * 日期:  2025-04
 */

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"

/// *************预配置参数

/**
 * @brief 最大IMU初始化帧数
 *
 * 在IMU初始化阶段，至少需要收集MAX_INI_COUNT个IMU数据包来估计
 * 初始重力方向、陀螺仪偏置和协方差矩阵。如果数据不足，初始化将失败。
 */
#define MAX_INI_COUNT (20)

/**
 * @brief 点云时间戳排序比较函数
 *
 * 根据PointType结构体的curvature字段（存储时间偏移，单位毫秒）进行升序排序。
 * 用于将一帧点云按照扫描时间排序，便于后续的运动畸变补偿。
 *
 * @param x 第一个点
 * @param y 第二个点
 * @return 如果x的时间戳小于y的时间戳则返回true
 */
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU进程与点云去畸变
/**
 * @class ImuProcess
 * @brief IMU数据处理与点云运动畸变补偿类
 *
 * ImuProcess类实现了IMU数据的预处理、初始化以及基于IMU的运动畸变补偿。
 * 该类使用扩展卡尔曼滤波器（EKF）的状态来预测LiDAR点在运动过程中的位姿，
 * 并将点云补偿到统一的参考时间戳，消除高速运动造成的几何畸变。
 *
 * 工作流程：
 * 1. 初始化阶段：收集IMU数据估计重力方向、陀螺仪偏置和协方差
 * 2. IMU处理：对每个IMU数据包进行前向传播（predict）
 * 3. 点云去畸变：根据IMU姿态历史，将每个激光点反向传播到参考时间
 *
 * 关键假设：
 * - IMU数据频率高于LiDAR（通常200Hz vs 10Hz）
 * - 帧间IMU姿态变化可以用一阶欧拉积分近似
 * - 点云扫描期间IMU运动是连续的
 *
 * 线程安全：该类设计为单线程使用，不保证线程安全。
 */
class ImuProcess
{
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   /**
    * @brief 构造函数
    *
    * 初始化所有成员变量：
    * - 过程噪声协方差矩阵Q（通过process_noise_cov()计算）
    * - 加速度计和陀螺仪协方差（默认0.1）
    * - 偏置协方差（默认0.0001）
    * - 初始均值（加速度为重力向量，陀螺仪为零）
    * - 外参（初始为单位矩阵，零平移）
    * - 状态标志（首帧、需要初始化）
    */
   ImuProcess();
   ~ImuProcess();
   
   /**
    * @brief 重置IMU处理状态
    *
    * 恢复所有内部状态到初始值，包括：
    * - 加速度计和陀螺仪均值
    * - 角速度历史
    * - 初始化标志
    * - 清空IMU缓冲队列和姿态历史
    * - 重新分配点云缓冲区
    */
   void Reset();
   // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
   /**
    * @brief 带时间戳的重置
    *
    * 重置状态并设置起始时间戳和最后一条IMU消息。
    *
    * @param start_timestamp 起始时间戳（秒）
    * @param lastimu 最后一条IMU消息（用于时间同步）
    */
   void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
   /**
    * @brief 设置IMU到LiDAR的外参（平移+旋转）
    *
    * 设置LiDAR相对于IMU的位姿变换。
    *
    * @param transl 平移向量 (3x1)，单位：米
    * @param rot 旋转矩阵 (3x3)，IMU坐标系到LiDAR坐标系的旋转
    */
   void set_extrinsic(const V3D &transl, const M3D &rot);
   /**
    * @brief 设置外参（仅平移，旋转为单位阵）
    *
    * 用于LiDAR与IMU共坐标系的特殊情况。
    *
    * @param transl 平移向量 (3x1)
    */
   void set_extrinsic(const V3D &transl);
   /**
    * @brief 设置外参（4x4齐次矩阵）
    *
    * 从齐次变换矩阵中提取平移和旋转分量。
    *
    * @param T 齐次变换矩阵 (4x4)，前3x3为旋转，前3行第4列为平移
    */
   void set_extrinsic(const MD(4,4) &T);
   /**
    * @brief 设置陀螺仪协方差缩放因子
    *
    * 用于调整IMU过程噪声中陀螺仪部分的置信度。
    *
    * @param scaler 缩放向量 (3x1)，通常为对角元素
    */
   void set_gyr_cov(const V3D &scaler);
   /**
    * @brief 设置加速度计协方差缩放因子
    *
    * 用于调整IMU过程噪声中加速度计部分的置信度。
    *
    * @param scaler 缩放向量 (3x1)
    */
   void set_acc_cov(const V3D &scaler);
   /**
    * @brief 设置陀螺仪偏置协方差
    *
    * @param b_g 偏置协方差 (3x1)，通常为小值(1e-4)
    */
   void set_gyr_bias_cov(const V3D &b_g);
   /**
    * @brief 设置加速度计偏置协方差
    *
    * @param b_a 偏置协方差 (3x1)，通常为小值(1e-4)
    */
   void set_acc_bias_cov(const V3D &b_a);
   /**
    * @brief IMU过程噪声协方差矩阵 (12x12)
    *
    * 状态向量：[位置(3), 速度(3), 姿态(4), 陀螺仪偏置(3), 加速度计偏置(3)]
    * 矩阵结构：
    * - [0:2] 陀螺仪噪声（角速度随机游走）
    * - [3:5] 加速度计噪声（速度随机游走）
    * - [6:8] 陀螺仪偏置噪声
    * - [9:11] 加速度计偏置噪声
    *
    * 在构造函数中由process_noise_cov()初始化，可在运行时通过set_*_cov()调整。
    */
   Eigen::Matrix<double, 12, 12> Q;
   /**
    * @brief 处理IMU数据并执行点云去畸变
    *
    * 主要处理流程：
    * 1. 如果IMU需要初始化，调用IMU_init()进行在线标定
    * 2. 否则，调用UndistortPcl()进行点云运动补偿
    *
    * @param meas 测量数据组（包含IMU队列和LiDAR点云）
    * @param kf_state EKF状态（输入/输出，用于预测和更新）
    * @param pcl_un 输出的去畸变点云（已修改pcl_in_out内容）
    */
   void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

   ///@brief IMU调试输出文件流（可选）
   ofstream fout_imu;
   ///@brief 加速度计协方差（3x1）
   V3D cov_acc;
   ///@brief 陀螺仪协方差（3x1）
   V3D cov_gyr;
   ///@brief 加速度计协方差缩放因子（用于IMU初始化后恢复）
   V3D cov_acc_scale;
   ///@brief 陀螺仪协方差缩放因子
   V3D cov_gyr_scale;
   ///@brief 陀螺仪偏置协方差（3x1）
   V3D cov_bias_gyr;
   ///@brief 加速度计偏置协方差（3x1）
   V3D cov_bias_acc;
   ///@brief 第一帧LiDAR时间戳（用于时间同步）
   double first_lidar_time;

 private:
   /**
    * @brief IMU在线初始化
    *
    * 执行以下任务：
    * 1. 计算加速度计和陀螺仪的均值（running average）
    * 2. 计算加速度计和陀螺仪的协方差
    * 3. 估计重力方向（对加速度均值归一化并乘以重力常数）
    * 4. 设置陀螺仪偏置为陀螺仪均值
    * 5. 配置外参（LiDAR相对于IMU的位姿）
    * 6. 初始化EKF的协方差矩阵P（位置、速度、姿态、偏置的不确定性）
    *
    * 初始化策略：
    * - 首次调用(N=1)：重置所有统计量，读取第一条IMU数据
    * - 后续调用：增量更新均值和协方差（Welford在线算法）
    * - 达到MAX_INI_COUNT后：调整加速度计协方差以匹配重力大小
    *   并标记imu_need_init_=false，退出初始化阶段
    *
    * 注意：初始化期间Process()会提前返回，不执行去畸变。
    *
    * @param meas 测量数据组（至少包含1个IMU和1个LiDAR）
    * @param kf_state EKF状态对象（输出初始化后的状态）
    * @param N 输入/输出参数：当前已处理的IMU样本数（初始为1，每次调用递增）
    */
   void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
   /**
    * @brief 点云运动畸变补偿
    *
    * 核心算法：基于IMU预积分的反向传播去畸变
    *
    * 步骤详解：
    * 1. 将上一帧的最后一个IMU添加到当前IMU队列头部（保证覆盖整个点云时间范围）
    * 2. 按时间戳对点云排序（curvature字段存储相对起始时间的时间偏移）
    * 3. 前向传播：从起始IMU时刻到结束IMU时刻，逐段积分得到每个IMU时刻的姿态、位置、速度
    *    积分方法：一阶欧拉，使用IMU区间中值作为输入
    *    状态转移：p_{k+1} = p_k + v_k*dt + 0.5*a_k*dt^2
    *              v_{k+1} = v_k + a_k*dt
    *              R_{k+1} = R_k * Exp(ω_k*dt)
    * 4. 反向去畸变：从点云结束时刻向前遍历，对每个点找到对应的时间区间
    *    变换公式：P_world = R_e^T * (R_i * (R_l * P_lidar + T_l) + T_ei) - T_l
    *    其中：
    *    - R_l, T_l: LiDAR到IMU的外参旋转和平移
    *    - R_i: 从点时刻到结束时刻的旋转（Exp(ω_avg*dt)）
    *    - T_ei: 结束时刻相对于点时刻的平移（v*dt + 0.5*a*dt^2）
    *    - R_e: 结束时刻IMU旋转
    *    - 最后再逆变换回LiDAR坐标系（减去T_l并左乘R_l^T）
    *
    * 注意：当前实现为简化版本，未考虑速度和加速度对点位置的完整补偿。
    *
    * @param meas 测量数据组（包含IMU队列和LiDAR点云）
    * @param kf_state EKF状态（用于获取当前姿态和位置）
    * @param pcl_in_out 输入点云（将被修改为去畸变后的点云）
    */
   void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out);

   ///@brief 当前帧去畸变后的点云（临时缓冲区）
   PointCloudXYZI::Ptr cur_pcl_un_;
   // sensor_msgs::ImuConstPtr last_imu_;
   ///@brief 最后一条IMU消息（用于时间边界判断）
   sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
   ///@brief IMU消息缓冲队列（滑动窗口）
   deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
   ///@brief IMU姿态历史（每个IMU时刻的6DoF位姿）
   vector<Pose6D> IMUpose;
   ///@brief 每个IMU时刻的旋转矩阵历史（预计算加速）
   vector<M3D>    v_rot_pcl_;
   ///@brief LiDAR相对于IMU的旋转（外参）
   M3D Lidar_R_wrt_IMU;
   ///@brief LiDAR相对于IMU的平移（外参）
   V3D Lidar_T_wrt_IMU;
   ///@brief 加速度计均值（初始化阶段估计，用于重力对齐）
   V3D mean_acc;
   ///@brief 陀螺仪均值（初始化阶段估计，作为初始偏置）
   V3D mean_gyr;
   ///@brief 上一时刻的角速度（去畸变中保存）
   V3D angvel_last;
   ///@brief 上一时刻的比力（去畸变中保存，IMU坐标系下）
   V3D acc_s_last;
   ///@brief 起始时间戳（第一帧LiDAR的时间）
   double start_timestamp_;
   ///@brief 上一帧LiDAR的结束时间（用于IMU时间边界判断）
   double last_lidar_end_time_;
   ///@brief IMU初始化迭代次数（计数）
   int    init_iter_num = 1;
   ///@brief 是否为第一帧的标志
   bool   b_first_frame_ = true;
   ///@brief IMU是否需要初始化的标志
   bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  /**
   * @brief 初始化IMU处理器的内部状态
   *
   * 初始化顺序和默认值：
   * 1. 过程噪声协方差Q：调用process_noise_cov()计算理论值
   * 2. 加速度计/陀螺仪协方差：默认0.1（需要根据实际IMU标定调整）
   * 3. 偏置协方差：默认0.0001（假设偏置缓慢变化）
   * 4. 加速度计均值：初始化为[0, 0, -1]，对应Z向上的重力方向
   *    注意：在NED坐标系中重力为(0,0,-9.81)，这里只存储方向
   * 5. 陀螺仪均值：初始化为0，假设静止开始
   * 6. 外参：初始为单位旋转、零平移（需要后续set_extrinsic()设置）
   * 7. 标志位：标记为首帧且需要初始化
   */
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

/**
 * @brief 析构函数
 *
 * 目前为空，所有资源由智能指针自动管理。
 * 如果fout_imu已打开，需要在外部确保关闭（Reset()或Process()中处理）。
 */
ImuProcess::~ImuProcess() {}

/**
 * @brief 重置IMU处理状态
 * 
 * 恢复所有内部状态到构造函数的初始值，用于系统重启或多会话场景。
 * 具体重置内容包括：
 * - mean_acc, mean_gyr: 恢复默认均值
 * - angvel_last: 清零
 * - imu_need_init_: 设为true，下一帧将重新进入IMU_init()
 * - start_timestamp_: 设为-1（无效值）
 * - init_iter_num: 重置为1
 * - v_imu_: 清空IMU队列
 * - IMUpose: 清空姿态历史
 * - last_imu_: 重新分配一个新的Imu消息对象
 * - cur_pcl_un_: 重新分配点云缓冲区
 */
void ImuProcess::Reset() 
{
   // ROS_WARN("Reset ImuProcess");
   mean_acc      = V3D(0, 0, -1.0);
   mean_gyr      = V3D(0, 0, 0);
   angvel_last       = Zero3d;
   imu_need_init_    = true;
   start_timestamp_  = -1;
   init_iter_num     = 1;
   v_imu_.clear();
   IMUpose.clear();
   last_imu_.reset(new sensor_msgs::msg::Imu());
   cur_pcl_un_.reset(new PointCloudXYZI());
}

/**
 * @brief 设置LiDAR相对于IMU的外参（齐次矩阵形式）
 *
 * 从4x4齐次变换矩阵中提取平移和旋转分量。
 * 矩阵T的布局：
 * | R(3x3) | t(3x1) |
 * |  0 0 0 |   1   |
 *
 * @param T 齐次变换矩阵 (Eigen::Matrix<double,4,4>)
 */
void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
   Lidar_T_wrt_IMU = T.block<3,1>(0,3);
   Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

/**
 * @brief 设置外参（仅平移，旋转为单位阵）
 *
 * 用于LiDAR与IMU坐标轴对齐、仅存在平移偏移的场景。
 *
 * @param transl 平移向量，单位：米
 */
void ImuProcess::set_extrinsic(const V3D &transl)
{
   Lidar_T_wrt_IMU = transl;
   Lidar_R_wrt_IMU.setIdentity();
}

/**
 * @brief 设置外参（平移+旋转矩阵）
 *
 * @param transl 平移向量 (3x1)，单位：米
 * @param rot 旋转矩阵 (3x3)，IMU坐标系→LiDAR坐标系的旋转
 */
void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
   Lidar_T_wrt_IMU = transl;
   Lidar_R_wrt_IMU = rot;
}

/**
 * @brief 设置陀螺仪过程噪声协方差
 *
 * 该值影响EKF对陀螺仪测量值的信任度，值越大表示越不确定。
 * 典型值：0.1 ~ 1.0 rad^2/s^2（取决于IMU精度）
 *
 * @param scaler 缩放因子（3维向量，通常各向同性，三个元素相等）
 */
void ImuProcess::set_gyr_cov(const V3D &scaler)
{
   cov_gyr_scale = scaler;
}

/**
 * @brief 设置加速度计过程噪声协方差
 *
 * 该值影响EKF对加速度计测量值的信任度。
 * 典型值：0.1 ~ 1.0 m^2/s^4
 *
 * @param scaler 缩放因子（3维向量）
 */
void ImuProcess::set_acc_cov(const V3D &scaler)
{
   cov_acc_scale = scaler;
}

/**
 * @brief 设置陀螺仪偏置过程噪声协方差
 *
 * 控制偏置状态的变化速率假设。值越大，偏置被认为变化越快。
 * 典型值：1e-4 ~ 1e-6 rad^2/s^3（偏置通常缓慢漂移）
 *
 * @param b_g 协方差值（3维向量）
 */
void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
   cov_bias_gyr = b_g;
}

/**
 * @brief 设置加速度计偏置过程噪声协方差
 *
 * @param b_a 协方差值（3维向量）
 */
void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
   cov_bias_acc = b_a;
}

/**
 * @brief IMU在线初始化（计算重力、偏置和协方差）
 *
 * 本函数实现IMU的在线标定，在系统启动或重启时调用。
 * 采用Welford在线算法增量计算均值和协方差，避免数值不稳定。
 *
 * 初始化流程（分两个阶段）：
 *
 * 阶段1：首次调用（b_first_frame_=true, N=1）
 * - 调用Reset()重置所有状态
 * - 读取第一条IMU数据（线性加速度和角速度）
 * - 将mean_acc设为第一条加速度，mean_gyr设为第一条角速度
 * - 记录first_lidar_time作为时间基准
 * - b_first_frame_设为false
 *
 * 阶段2：后续调用（N从2递增到MAX_INI_COUNT）
 * - 遍历当前帧的所有IMU数据包
 * - 对每个IMU测量，使用递推公式更新均值：
 *   mean_new = mean_old + (x - mean_old) / N
 * - 同时更新协方差（使用Welford算法）：
 *   cov_new = cov_old * (N-1)/N + (x - mean_new) * (x - mean_old) / N
 * - N递增
 *
 * 阶段3：初始化完成（N > MAX_INI_COUNT）
 * - 调整加速度计协方差：cov_acc *= (G / ||mean_acc||)^2
 *   目的是将加速度均值缩放到标准重力G=9.81，使协方差具有物理意义
 * - 恢复加速度计和陀螺仪协方差到预设缩放值（cov_acc_scale, cov_gyr_scale）
 * - 设置IMU状态：
 *   grav = S2(-mean_acc / ||mean_acc|| * G_m_s2)  // 重力方向（单位四元数）
 *   bg = mean_gyr  // 陀螺仪偏置
 *   offset_T_L_I, offset_R_L_I = 外参（LiDAR相对于IMU）
 * - 初始化EKF协方差矩阵P：
 *   P(6:8,6:8)=0.00001（位置不确定性小）
 *   P(9:11,9:11)=0.00001（速度不确定性小）
 *   P(15:17,15:17)=0.0001（姿态不确定性）
 *   P(18:20,18:20)=0.001（偏置不确定性较大）
 *   P(21:23,21:23)=0.00001（其他状态）
 * - 标记imu_need_init_=false，结束初始化
 * - 打开imu调试输出文件
 *
 * 注意：
 * - 如果IMU数据不足（静止时间太短），初始化可能失败或精度低
 * - 初始化期间Process()提前返回，不执行去畸变
 *
 * @param meas 测量数据组（包含当前帧的IMU队列和LiDAR点云）
 * @param kf_state EKF状态对象（输入/输出：更新后的状态和协方差）
 * @param N 输入/输出参数：当前已累计的IMU样本数
 *           首次调用前应设为1，函数内部递增
 */
void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
   /** 1. 初始化重力、陀螺仪偏置、加速度计和陀螺仪协方差
    ** 2. 将加速度测量值归一化为单位重力方向 **/
   
   V3D cur_acc, cur_gyr;
   
   if (b_first_frame_)
   {
     Reset();
     N = 1;
     b_first_frame_ = false;
     const auto &imu_acc = meas.imu.front()->linear_acceleration;
     const auto &gyr_acc = meas.imu.front()->angular_velocity;
     mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
     mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
     first_lidar_time = meas.lidar_beg_time;
   }

   for (const auto &imu : meas.imu)
   {
     const auto &imu_acc = imu->linear_acceleration;
     const auto &gyr_acc = imu->angular_velocity;
     cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
     cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

     // Welford在线算法：增量更新均值
     mean_acc      += (cur_acc - mean_acc) / N;
     mean_gyr      += (cur_gyr - mean_gyr) / N;

     // Welford在线算法：增量更新协方差（避免存储所有历史数据）
     cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
     cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

     // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

     N ++;
   }
   state_ikfom init_state = kf_state.get_x();
   // 将加速度均值归一化并乘以重力常数，得到重力向量（世界坐标系Z向上）
   // S2()将3D向量转换为单位四元数表示的重力方向
   init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
   
   //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
   // 设置陀螺仪偏置为均值（假设静止开始，无旋转）
   init_state.bg  = mean_gyr;
   // 设置LiDAR相对于IMU的外参
   init_state.offset_T_L_I = Lidar_T_wrt_IMU;
   init_state.offset_R_L_I = Lidar_R_wrt_IMU;
   kf_state.change_x(init_state);

   // 初始化EKF协方差矩阵P
   esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
   init_P.setIdentity();
   init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;  // 位置不确定性小（已初步定位）
   init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;  // 速度不确定性小
   init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;  // 姿态不确定性（四元数）
   init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;  // 陀螺仪偏置不确定性
   init_P(21,21) = init_P(22,22) = 0.00001;  // 加速度计偏置不确定性
   kf_state.change_P(init_P);
   last_imu_ = meas.imu.back();

}

/**
 * @brief 点云运动畸变补偿（基于IMU预积分）
 *
 * 本函数实现基于IMU数据的点云去畸变算法，消除LiDAR在运动过程中扫描造成的几何畸变。
 *
 * 核心思想：
 * 一帧LiDAR扫描需要一定时间（例如100ms），在此期间传感器可能移动。
 * 每个激光点对应不同的位姿，直接拼接会产生拖影和形变。
 * 本函数将所有点补偿到统一的参考时间（通常是帧的中点或结束时刻）。
 *
 * 算法步骤：
 *
 * 步骤1：时间边界对齐
 * - 将上一帧的最后一条IMU数据(v_imu.back())添加到当前队列头部
 * - 原因：当前帧的第一个点可能对应上一帧结束后的IMU数据
 * - 计算IMU时间范围：[imu_beg_time, imu_end_time]
 * - 计算点云时间范围：[pcl_beg_time, pcl_end_time]
 *  其中 pcl_end_time = pcl_beg_time + max(curvature)/1000
 *
 * 步骤2：点云时间排序
 * - 按curvature字段（单位：毫秒）升序排序
 * - 保证后续遍历时时间单调递增
 *
 * 步骤3：前向IMU积分（Forward Propagation）
 * - 从imu_beg_time到imu_end_time，逐段积分
 * - 积分方法：一阶欧拉法（在IMU区间中值积分）
 *   ω_avg = 0.5 * (ω_head + ω_tail)
 *   a_avg  = 0.5 * (a_head + a_tail)
 *   dt = tail_stamp - head_stamp
 * - 状态预测：
 *   v_next = v_curr + a_avg * dt
 *   p_next = p_curr + v_curr*dt + 0.5*a_avg*dt^2
 *   R_next = R_curr * Exp(ω_avg * dt)
 * - 每一步将姿态(rot)、位置(pos)、速度(vel)保存到IMUpose向量
 * - 同时保存IMU坐标系下的比力acc_s_last和角速度angvel_last
 *
 * 步骤4：反向去畸变（Backward Compensation）
 * - 从点云末尾向前遍历（it_pcl从end-1到begin）
 * - 对每个点，找到对应的时间区间（it_kp，即IMUpose中相邻两个姿态）
 * - 计算点时刻相对于结束时刻的增量时间：dt = p_offset - head_offset
 * - 从点时刻旋转到结束时刻：R_i_to_e = Exp(ω_avg, dt)
 * - 从点时刻平移到结束时刻：T_ei = v_head*dt + 0.5*a_head*dt^2
 * - 完整变换链：
 *   a) 将点从LiDAR系变换到IMU系：P_imu_lidar = R_l * P_lidar + T_l
 *   b) 从点时刻旋转到结束时刻：P_imu_e = R_i_to_e * P_imu_lidar + T_ei
 *   c) 从IMU系旋转到世界系（结束时刻IMU旋转的逆）：P_world = R_e^T * P_imu_e
 *   d) 反向变换回LiDAR系（补偿后的LiDAR坐标）：P_comp = R_l^T * (P_world - T_l)
 * - 将补偿后的坐标写回点云
 *
 * 数学公式（精简版）：
 * P_comp = R_l^T * (R_e^T * (R_i * (R_l * P + T_l) + T_ei) - T_l)
 * 其中：
 *   R_l = offset_R_L_I (LiDAR→IMU外参旋转)
 *   T_l = offset_T_L_I (LiDAR→IMU外参平移)
 *   R_i = R_imu * Exp(ω_avg, dt)  (点时刻到结束时刻的旋转)
 *   R_e = imu_state.rot (结束时刻IMU旋转)
 *   T_ei = pos_imu + vel_imu*dt + 0.5*acc_imu*dt^2 - imu_state.pos
 *
 * 注意：
 * - 当前实现使用了简化的补偿公式（注释中标记"not accurate!"）
 * - 完整实现应考虑速度和加速度对点位置的贡献
 * - 假设IMU频率足够高，区间内加速度和角速度近似恒定
 *
 * @param meas 测量数据组（IMU队列 + 一帧点云）
 * @param kf_state EKF当前状态（用于获取初始位姿）
 * @param pcl_out 输入/输出点云（原地修改，存储去畸变后的点）
 */
void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
   /*** 步骤1：扩展IMU队列，将上一帧的最后一条IMU加入头部 ***/
   auto v_imu = meas.imu;
   v_imu.push_front(last_imu_);
   const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
   const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
   const double &pcl_beg_time = meas.lidar_beg_time;
   
   /*** 步骤2：按时间戳对点云排序 ***/
   pcl_out = *(meas.lidar);
   sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
   const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
   // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
   //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

   /*** 步骤3：前向传播IMU姿态（Forward Propagation）***/
   state_ikfom imu_state = kf_state.get_x();
   IMUpose.clear();
   // 初始姿态（t=0，对应pcl_beg_time之前的last_imu_时刻）
   IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

   // 遍历每个IMU区间 [head, tail]，积分得到tail时刻的姿态
   V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
   M3D R_imu;

   double dt = 0;

   input_ikfom in;
   for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
   {
     auto &&head = *(it_imu);
     auto &&tail = *(it_imu + 1);
     
     double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
     double head_stamp = rclcpp::Time(head->header.stamp).seconds();

     // 跳过早于上一帧LiDAR结束时间的IMU数据（避免重复处理）
     if (tail_stamp < last_lidar_end_time_)    continue;
     
     // 计算角速度和加速度的平均值（中值积分）
     angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                 0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                 0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
     acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                 0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

     // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

     /**
      * @brief 加速度计标定
      *
      * 将加速度测量值缩放到标准重力G_m_s2（9.81 m/s²）。
      * 假设IMU初始静止，加速度均值应等于重力大小。
      * 缩放因子：G_m_s2 / ||mean_acc||
      */
     acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

     // 计算积分时间间隔
     if(head_stamp < last_lidar_end_time_)
     {
       dt = tail_stamp - last_lidar_end_time_;
       // dt = tail->header.stamp.toSec() - pcl_beg_time;
     }
     else
     {
       dt = tail_stamp - head_stamp;
     }
     
     in.acc = acc_avr;
     in.gyro = angvel_avr;
     // 设置过程噪声协方差Q（对角阵）
     Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
     Q.block<3, 3>(3, 3).diagonal() = cov_acc;
     Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
     Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
     // EKF前向预测
     kf_state.predict(dt, Q, in);

     /* 保存当前IMU区间的末端姿态，供点云去畸变使用 */
     imu_state = kf_state.get_x();
     // 保存去偏置后的角速度（body系）
     angvel_last = angvel_avr - imu_state.bg;
     // 保存IMU坐标系下的比力（扣除偏置并旋转到body系）
     acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
     for(int i=0; i<3; i++)
     {
       acc_s_last[i] += imu_state.grav[i];  // 加上重力（grav已包含方向和大小）
     }
     double &&offs_t = tail_stamp - pcl_beg_time;
     IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
   }

   /*** 步骤4：计算帧结束时刻的IMU姿态（可能超出最后一个IMU时间）***/
   double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
   dt = note * (pcl_end_time - imu_end_time);
   kf_state.predict(dt, Q, in);
   
   imu_state = kf_state.get_x();
   last_imu_ = meas.imu.back();
   last_lidar_end_time_ = pcl_end_time;

   /*** 步骤5：反向去畸变（Backward Propagation）***/
   if (pcl_out.points.begin() == pcl_out.points.end()) return;
   auto it_pcl = pcl_out.points.end() - 1;
   // 从最后一个IMU姿态向前遍历到第一个
   for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
   {
     auto head = it_kp - 1;   // 较早的IMU姿态
     auto tail = it_kp;       // 较晚的IMU姿态（接近当前点）
     
     R_imu<<MAT_FROM_ARRAY(head->rot);
     // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
     vel_imu<<VEC_FROM_ARRAY(head->vel);
     pos_imu<<VEC_FROM_ARRAY(head->pos);
     acc_imu<<VEC_FROM_ARRAY(tail->acc);
     angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

     /**
      * 遍历当前IMU区间内的所有点
      * 条件：点的时间偏移(curvature/1000) > head->offset_time
      * 即点时刻在head时刻之后，在tail时刻之前（或等于）
      */
     for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
     {
       dt = it_pcl->curvature / double(1000) - head->offset_time;

       /**
        * 变换公式解释：
        * 目标：将LiDAR坐标系下的点P_i补偿到帧结束时刻的LiDAR坐标系
        * 
        * 步骤分解：
        * 1) P_imu_lidar = R_l * P_i + T_l
        *    将点从LiDAR系变换到IMU系（使用外参）
        *
        * 2) R_i = R_imu * Exp(ω_avg, dt)
        *    从点时刻(head)旋转到结束时刻(tail)
        *    Exp(ω_avg, dt)是旋转矩阵的指数映射（Rodrigues公式）
        *
        * 3) T_ei = pos_imu + vel_imu*dt + 0.5*acc_imu*dt^2 - imu_state.pos
        *    从head位置到tail位置的平移增量（相对于世界系原点）
        *    pos_imu是head时刻的位置，imu_state.pos是tail时刻的位置
        *    因此 T_ei = (p_head + v*dt + 0.5*a*dt^2) - p_tail
        *
        * 4) P_imu_e = R_i * P_imu_lidar + T_ei
        *    将点从head时刻的IMU系变换到tail时刻的IMU系
        *
        * 5) P_world = R_e^T * P_imu_e
        *    从tail时刻的IMU系变换到世界系（使用结束姿态的逆旋转）
        *
        * 6) P_comp = R_l^T * (P_world - T_l)
        *    从世界系反向变换回LiDAR系，得到补偿后的LiDAR坐标
        *
        * 注意：当前公式(327行)可能存在误差，完整形式应考虑：
        * - 点时刻的旋转R_i应使用head->rot的指数形式
        * - 平移T_ei应包含速度和加速度的完整积分
        */
       M3D R_i(R_imu * Exp(angvel_avr, dt));
       
       V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
       V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
       V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
       
       // 保存去畸变后的点坐标
       it_pcl->x = P_compensate(0);
       it_pcl->y = P_compensate(1);
       it_pcl->z = P_compensate(2);

       if (it_pcl == pcl_out.points.begin()) break;
     }
   }
}

/**
 * @brief IMU处理主函数（Process的完整实现）
 *
 * 这是ImuProcess类的主要入口点，由LaserMappingNode在每帧定时器中调用。
 * 处理流程：
 *
 * 1. 输入检查：
 *    - 如果IMU队列为空，直接返回（跳过当前帧）
 *    - 断言LiDAR点云非空（保证数据完整性）
 *
 * 2. 初始化阶段（imu_need_init_=true）：
 *    - 调用IMU_init()进行IMU在线标定
 *    - 记录最后一条IMU消息
 *    - 如果init_iter_num > MAX_INI_COUNT：
 *      * 调整加速度计协方差（缩放到标准重力）
 *      * 恢复加速度计/陀螺仪协方差到预设值
 *      * 输出"IMU Initial Done"提示
 *      * 打开imu调试文件
 *    - 提前返回，不执行去畸变（等待初始化完成）
 *
 * 3. 正常处理阶段（imu_need_init_=false）：
 *    - 调用UndistortPcl()执行点云去畸变
 *    - 输出时间统计（t1, t2, t3）
 *
 * 时间统计：
 * - t1: 函数入口时间
 * - t2: IMU初始化完成时间（仅在初始化阶段有效）
 * - t3: 去畸变完成时间（仅在正常阶段有效）
 *
 * @param meas 测量数据组（IMU队列 + 一帧LiDAR点云）
 * @param kf_state EKF状态（输入/输出：IMU_init()或UndistortPcl()会修改）
 * @param cur_pcl_un_ 输出的去畸变点云（已修改内容）
 */
void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// 第一帧LiDAR到达，进行IMU初始化
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;  // 注意：这里仍为true，直到init_iter_num>MAX_INI_COUNT才置false
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      /**
       * @brief 初始化完成后的最终调整
       *
       * 缩放加速度计协方差：
       * cov_acc *= (G_m_s2 / ||mean_acc||)^2
       *
       * 目的：将在线估计的加速度均值（可能不等于标准重力）
       * 调整为标准重力，使协方差具有物理一致性。
       * 例如，如果||mean_acc||=9.5，缩放因子=(9.81/9.5)^2≈1.07
       */
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;  // 初始化完成，下一帧将执行去畸变

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      std::cout << "IMU Initial Done" << std::endl;
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  // 正常处理：执行点云去畸变
  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
