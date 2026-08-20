/**
 * @file common_lib.h
 * @brief FAST_LIO定位的公共定义、类型和工具函数
 * @author FAST_LIO localization team
 * @date 2026
 * @details 本头文件提供FAST_LIO（快速鲁棒LiDAR惯性里程计）系统中
 *          共享的类型定义、常量和辅助函数。
 *          包括状态表示、平面拟合工具和ROS消息转换。
 *
 * @section Dependencies
 * - so3_math.h（SO(3)数学操作）
 * - Eigen3（线性代数）
 * - PCL（点云库）
 * - ROS 2消息（rclcpp、geometry_msgs、sensor_msgs、nav_msgs）
 */

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fast_lio_localization/msg/pose6_d.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std;
using namespace Eigen;

/**
 * @def USE_IKFOM
 * @brief 启用/禁用 IKFOM（流形上的迭代卡尔曼滤波）特性
 * @details 定义时使用IKFOM特定的优化和状态表示。
 */
#define USE_IKFOM

/**
 * @def PI_M
 * @brief 数学常数 π（pi），高精度（3.14159265358）
 */
#define PI_M (3.14159265358)

/**
 * @def G_m_s2
 * @brief 重力加速度常数（9.81 m/s²）
 * @details 适用于广东/中国地区的值。随纬度可能有轻微变化。
 */
#define G_m_s2 (9.81)   // 广东/中国地区重力常数

/**
 * @def DIM_STATE
 * @brief 状态向量总维度（18）
 * @details 状态向量布局：[3(旋转SO3), 3(位置), 3(速度),
 *          3(陀螺仪偏置), 3(加速度计偏置), 3(重力)] = 18维。
 *          SO(3)流形贡献3个自由度。
 */
#define DIM_STATE (18)  // 状态维度（设Dim(SO(3)) = 3）

/**
 * @def DIM_PROC_N
 * @brief 过程噪声向量维度（12）
 * @details 过程噪声影响：速度（3）、偏置（6）、重力（3）。
 *          在此模型中假设旋转无噪声。
 */
#define DIM_PROC_N (12) // 过程噪声维度（设Dim(SO(3)) = 3）

/**
 * @def CUBE_LEN
 * @brief 局部地图状态立方体的边长（6.0米）
 * @details 用于点云管理中定义局部地图边界。
 */
#define CUBE_LEN (6.0)

/**
 * @def LIDAR_SP_LEN
 * @brief LiDAR扫描线数量（2）- 可能是占位符/未使用
 */
#define LIDAR_SP_LEN (2)

/**
 * @def INIT_COV
 * @brief 初始协方差标量值（1）
 * @details 初始协方差矩阵 = 单位矩阵 * INIT_COV。
 */
#define INIT_COV (1)

/**
 * @def NUM_MATCH_POINTS
 * @brief 平面拟合的最近邻点数量（5）
 * @details 用于通过PCA（主成分分析）进行平面估计。
 */
#define NUM_MATCH_POINTS (5)

/**
 * @def MAX_MEAS_DIM
 * @brief 最大测量维度（10000）
 * @details EKF更新时测量向量大小的上限。
 */

/**
 * @def VEC_FROM_ARRAY(v)
 * @brief 将数组v扩展为三个逗号分隔元素：v[0], v[1], v[2]
 * @details 对于期望三个独立参数的函数调用很有用。
 * @warning 无边界检查；v必须至少有3个元素。
 */
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]

/**
 * @def MAT_FROM_ARRAY(v)
 * @brief 将存储在数组中的3x3矩阵扩展为9个逗号分隔元素
 * @details 行优先顺序：v[0]..v[8] → m00,m01,m02,m10,...,m22
 * @warning 无边界检查；v必须至少有9个元素。
 */
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

/**
 * @def CONSTRAIN(v, min, max)
 * @brief 将值v限制在范围[min, max]内
 * @return 如果v<min返回min，如果v>max返回max，否则返回v
 * @details 用于快速边界 enforcing 的三元运算符链。
 */
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)

/**
 * @def ARRAY_FROM_EIGEN(mat)
 * @brief 从Eigen矩阵获取原始数据数组指针
 * @param mat Eigen矩阵（Matrix或Array）
 * @return T* 指向连续数据的指针（mat.data()）
 * @details Eigen矩阵连续存储数据。用于与期望原始指针的C API接口。
 */
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()

/**
 * @def STD_VEC_FROM_EIGEN(mat)
 * @brief 从Eigen矩阵数据构造std::vector
 * @param mat Eigen矩阵
 * @return 包含相同元素的std::vector<T>（复制）
 * @details 通过复制矩阵元素创建向量。对PCL转换很有用。
 */
#define STD_VEC_FROM_EIGEN(mat) vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())

/**
 * @def DEBUG_FILE_DIR(name)
 * @brief 构造调试日志文件的完整路径
 * @param name 文件名（不含路径）
 * @return 包含 ROOT_DIR/Log/name 的 std::string
 * @details ROOT_DIR 必须在别处定义。用于记录调试信息。
 */
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

/**
 * @typedef Pose6D
 * @brief 6D位姿消息类型（位置+方向）
 * @details 来自 fast_lio_localization::msg::Pose6D。
 *          包含位置[x,y,z]、旋转矩阵[3x3]、速度[vx,vy,vz]、
 *          加速度[ax,ay,az]、角速度[wx,wy,wz]和时间戳偏移 offset_time。
 */
typedef fast_lio_localization::msg::Pose6D Pose6D;

/**
 * @typedef PointType
 * @brief PCL点的别名，包含位置、强度、法线、曲率
 * @details 基于 pcl::PointXYZINormal：x,y,z, intensity, normal_x,y,z, curvature。
 *          本代码库中LiDAR处理的标准点类型。
 */
typedef pcl::PointXYZINormal PointType;

/**
 * @typedef PointCloudXYZI
 * @brief PointType点的PCL点云
 * @details 点云的共享指针类型：PointCloud<PointType>::Ptr
 */
typedef pcl::PointCloud<PointType> PointCloudXYZI;

/**
 * @typedef PointVector
 * @brief 使用Eigen对齐分配器的PointType的std::vector
 * @details 支持点数据上的向量化Eigen操作。要求STL容器中
 *          Eigen类型正确对齐。
 */
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

/**
 * @typedef V3D
 * @brief 三维双精度向量（Eigen::Vector3d）
 */
typedef Vector3d V3D;

/**
 * @typedef M3D
 * @brief 3x3双精度矩阵（Eigen::Matrix3d）
 */
typedef Matrix3d M3D;

/**
 * @typedef V3F
 * @brief 三维单精度向量（Eigen::Vector3f）
 */
typedef Vector3f V3F;

/**
 * @typedef M3F
 * @brief 3x3单精度矩阵（Eigen::Matrix3f）
 */
typedef Matrix3f M3F;

/**
 * @def MD(a, b)
 * @brief Matrix<double, a, b>的简写
 * @example MD(3, 3) → Matrix<double, 3, 3>
 */
#define MD(a, b) Matrix<double, (a), (b)>

/**
 * @def VD(a)
 * @brief Matrix<double, a, 1>（列向量）的简写
 * @example VD(3) → Matrix<double, 3, 1>
 */
#define VD(a) Matrix<double, (a), 1>

/**
 * @def MF(a, b)
 * @brief Matrix<float, a, b>的简写
 */
#define MF(a, b) Matrix<float, (a), (b)>

/**
 * @def VF(a)
 * @brief Matrix<float, a, 1>的简写
 */
#define VF(a) Matrix<float, (a), 1>

/**
 * @var Eye3d
 * @brief 全局3x3单位矩阵（双精度）
 * @details M3D::Identity()在静态初始化时求值。
 */
M3D Eye3d(M3D::Identity());

/**
 * @var Eye3f
 * @brief 全局3x3单位矩阵（单精度）
 * @details M3F::Identity()在静态初始化时求值。
 */
M3F Eye3f(M3F::Identity());

/**
 * @var Zero3d
 * @brief 全局3D零向量（双精度）
 * @details V3D(0,0,0)在静态初始化时求值。
 */
V3D Zero3d(0, 0, 0);

/**
 * @var Zero3f
 * @brief 全局3D零向量（单精度）
 * @details V3F(0,0,0)在静态初始化时求值。
 */
V3F Zero3f(0, 0, 0);

/**
 * @struct MeasureGroup
 * @brief 用于处理的LiDAR和IMU测量数据组
 * @details 持同步的数据批次：一个LiDAR点云和
 *          IMU消息的deque。作为状态估计的输入。
 * @note 当前未存储LiDAR结束时间（已注释）。
 */
struct MeasureGroup
{
    /**
     * @brief 默认构造函数
     * @details 将 lidar_beg_time 初始化为0.0并创建空的PointCloudXYZI。
     */
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };

    double lidar_beg_time;           /**< LiDAR帧开始时间戳（秒） */
    // double lidar_end_time;        /**< [已注释] LiDAR帧结束时间戳 */
    PointCloudXYZI::Ptr lidar;       /**< LiDAR点云的共享指针 */
    deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;  /**< IMU消息缓冲区 */
};

/**
 * @struct StatesGroup
 * @brief EKF/IKF估计的完整系统状态
 * @details 包含FAST_LIO的所有状态变量：位姿、速度、偏置、
 *          重力和协方差矩阵。支持SO(3)流形上的代数运算
 *          用于状态传播和误差计算。
 *
 * @section State_Vector_Layout (18维):
 * - rot_end:    世界到身体的旋转矩阵R_wb（3x3）
 * - pos_end:    世界坐标系中的位置（3x1）
 * - vel_end:    世界坐标系中的速度（3x1）
 * - bias_g:     陀螺仪偏置（3x1）
 * - bias_a:    加速度计偏置（3x1）
 * - gravity:   重力向量（3x1）
 * - cov:        18x18协方差矩阵（P）
 *
 * @section Operations
 * - operator+  : 添加状态增量（使用Exp()处理旋转）
 * - operator+= : 就地添加增量
 * - operator-  : 状态差（使用Log()处理旋转）
 * - resetpose() : 将位置、速度、旋转重置为初始值
 */
struct StatesGroup
{
    /**
     * @brief 默认构造函数
     * @details 初始化所有状态变量为单位矩阵/零向量：
     *          - rot_end：单位旋转
     *          - pos_end、vel_end、bias_g、bias_a、gravity：零向量
     *          - cov：前9x9块为INIT_COV的对角矩阵，
     *                 偏置/重力块为很小值（1e-5）。
     */
    StatesGroup()
    {
        this->rot_end = M3D::Identity();
        this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g = Zero3d;
        this->bias_a = Zero3d;
        this->gravity = Zero3d;
        this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
        this->cov.block<9, 9>(9, 9) = MD(9, 9)::Identity() * 0.00001;
    };

    /**
     * @brief 拷贝构造函数
     * @param b 要拷贝的源StatesGroup
     * @details 深度拷贝所有状态变量和协方差矩阵。
     */
    StatesGroup(const StatesGroup &b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
        this->gravity = b.gravity;
        this->cov = b.cov;
    };

    /**
     * @brief 拷贝赋值运算符
     * @param b 要赋值的源StatesGroup
     * @return 对 *this 的引用
     * @details 深度拷贝所有状态变量。
     */
    StatesGroup &operator=(const StatesGroup &b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
        this->gravity = b.gravity;
        this->cov = b.cov;
        return *this;
    };

    /**
     * @brief 添加状态增量（非成员加法更新）
     * @param state_add 18x1增量向量 [dR(3), dp(3), dv(3), dbg(3), dba(3), dg(3)]
     * @return 增量后的新StatesGroup
     * @details 旋转增量dR使用Exp()将so(3)向量转换为SO(3)：
     *          R_new = R_old * Exp(dR)。其他状态线性相加。
     *          协方差保持不变（应通过EKF单独更新）。
     */
    StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
    {
        StatesGroup a;
        a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
        a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
        a.cov = this->cov;
        return a;
    };

    /**
     * @brief 就地添加状态增量
     * @param state_add 18x1增量向量
     * @return 修改后对 *this 的引用
     * @details 与 operator+ 类似但修改当前对象。
     */
    StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        this->pos_end += state_add.block<3, 1>(3, 0);
        this->vel_end += state_add.block<3, 1>(6, 0);
        this->bias_g += state_add.block<3, 1>(9, 0);
        this->bias_a += state_add.block<3, 1>(12, 0);
        this->gravity += state_add.block<3, 1>(15, 0);
        return *this;
    };

    /**
     * @brief 计算状态差（误差状态）
     * @param b 要减去的另一个StatesGroup
     * @return 切空间中的18x1误差状态向量
     * @details 计算：ΔR = Log(R_b^T * R_this)，Δp, Δv, Δbias, Δg 为向量。
     *          旋转差使用矩阵对数将SO(3)映射到so(3)。
     *          结果是右不变误差状态表示。
     */
    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
    {
        Matrix<double, DIM_STATE, 1> a;
        M3D rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3, 1>(0, 0) = Log(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
        a.block<3, 1>(15, 0) = this->gravity - b.gravity;
        return a;
    };

    /**
     * @brief 重置位置、速度和旋转为初始值
     * @details 将 rot_end 设为单位矩阵，pos_end 和 vel_end 设为零。
     *          不重置偏置或重力。
     */
    void resetpose()
    {
        this->rot_end = M3D::Identity();
        this->pos_end = Zero3d;
        this->vel_end = Zero3d;
    }

    M3D rot_end;                              /**< 估计的姿态：LiDAR点结束时的世界到身体旋转矩阵 */
    V3D pos_end;                              /**< LiDAR点结束时的世界坐标系估计位置 */
    V3D vel_end;                              /**< LiDAR点结束时的世界坐标系估计速度 */
    V3D bias_g;                               /**< 估计的陀螺仪偏置（rad/s） */
    V3D bias_a;                               /**< 估计的加速度计偏置（m/s²） */
    V3D gravity;                              /**< 估计的重力向量（m/s²） */
    Matrix<double, DIM_STATE, DIM_STATE> cov; /**< 状态协方差矩阵（18x18） */
};

/**
 * @brief 将弧度转换为度
 * @tparam T 数值类型（float、double等）
 * @param radians 弧度值
 * @return 角度值（度）
 * @details 使用转换因子180/π。模板适用于任何数值类型。
 */
template <typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / PI_M;
}

/**
 * @brief 将度转换为弧度
 * @tparam T 数值类型（float、double等）
 * @param degrees 角度值（度）
 * @return 弧度值
 * @details 使用转换因子π/180。
 */
template <typename T>
T deg2rad(T degrees)
{
  return degrees * PI_M / 180.0;
}

/**
 * @brief 从组件构造Pose6D消息
 * @tparam T 标量类型（float或double）
 * @param t 时间戳偏移
 * @param a 加速度向量（3x1）
 * @param g 角速度（陀螺仪）向量（3x1）
 * @param v 速度向量（3x1）
 * @param p 位置向量（3x1）
 * @param R 旋转矩阵（3x3）
 * @return 填充了数据的Pose6D消息
 * @details 将所有状态组件打包到Pose6D ROS消息中。
 *          旋转矩阵按行优先顺序存储在 rot[9] 中。
 */
template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g,
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    return move(rot_kp);
}

/**
 * @brief 从点云估计平面法向量
 * @tparam T 数值类型（float/double）
 * @param normvec 输出：归一化的平面法向量（3x1单位向量）
 * @param point 输入：点云（至少NUM_MATCH_POINTS个点）
 * @param threshold 内点最大距离阈值
 * @param point_num 使用的点数（应为NUM_MATCH_POINTS）
 * @return 如果平面拟合在阈值内返回true，否则false
 * @details 求解 Ax=b，其中 A=[x y z]矩阵，b=[-1 ... -1]^T。
 *          解 x = [A/D, B/D, C/D]^T 给出平面法向量（A,B,C）
 *          通过D归一化。检查所有点满足 |Ax+By+Cz+1| < threshold。
 *          使用Eigen的colPivHouseholderQr()进行QR分解。
 *
 * @note 平面方程：Ax + By + Cz + D = 0 → (A/D)x + (B/D)y + (C/D)z = -1
 *       求解后，normvec归一化为单位长度。
 */
template <typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);
    
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

/**
 * @brief 计算两点间的平方欧氏距离
 * @param p1 第一个点
 * @param p2 第二个点
 * @return 平方距离（dx² + dy² + dz²）
 * @details 不取平方根以提高效率。用于最近邻比较，
 *          其中相对距离已足够。
 */
float calc_dist(PointType p1, PointType p2)
{
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

/**
 * @brief 使用PCA估计平面参数（基于NUM_MATCH_POINTS）
 * @tparam T 数值类型
 * @param pca_result 输出：4x1平面参数 [A, B, C, D] 已归一化
 * @param point 输入：点云（至少NUM_MATCH_POINTS个点）
 * @param threshold 内点距离阈值
 * @return 如果平面拟合所有点都在阈值内返回true
 * @details 类似于 esti_normvector 但返回完整的平面参数，
 *          D已包含在内。解法求法向量，然后计算
 *          D = 1/||normal||，给出平面方程：(A)x + (B)y + (C)z + D = 0
 *          其中 [A,B,C] = normvec * D。
 */
template <typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 将ROS Time消息转换为秒（double）
 * @param time ROS builtin_interfaces::Time消息
 * @return 以秒为单位的double时间
 * @details 使用 rclcpp::Time 进行转换。处理纳秒精度。
 */
double get_time_sec(const builtin_interfaces::msg::Time &time)
{
    return rclcpp::Time(time).seconds();
}

/**
 * @brief 将时间戳（double秒）转换为ROS Time
 * @param timestamp 时间（秒，可能包含小数部分）
 * @return rclcpp::Time对象
 * @details 拆分为整数秒和纳秒余数。
 *          用于发布具有正确时间戳的消息。
 */
rclcpp::Time get_ros_time(double timestamp)
{
    int32_t sec = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    uint32_t nanosec = nanosec_d;
    return rclcpp::Time(sec, nanosec);
}

#endif