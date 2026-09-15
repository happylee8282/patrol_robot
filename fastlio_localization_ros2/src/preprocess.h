/**
 * @file preprocess.h
 * @brief FAST_LIO的LiDAR点云预处理
 * @author FAST_LIO localization team
 * @date 2026
 * @details Preprocess类处理多个制造商（Velodyne、Ouster、Livox）的原始LiDAR数据
 *          并转换为统一的PCL格式。
 *          执行特征提取：地面点移除、边缘检测和点分类。
 *          专为实时SLAM应用设计。
 *
 * @section Supported_Sensors
 * - Livox Avia (MID40/MID70) 通过 livox_ros_driver2
 * - Velodyne VLP-16, VLP-32 通过 velodyne_ros
 * - Ouster OS-1, OS-2 通过 ouster_ros
 * - Hesai HESAIxt16 通过 hesai_ros
 * - 宇树L1 通过 unilidar_ros
 * - 通用 PointCloud2（默认处理器）
 *
 * @section Output_Point_Types
 * - pl_full: 所有点（过滤后）
 * - pl_corn: 角点/边缘特征
 * - pl_surf: 表面/平面特征
 */

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using namespace std;

/**
 * @def IS_VALID(a)
 * @brief 检查值是否被视为无效/超出范围
 * @param a 待测试的值
 * @return 如果 |a| > 1e8 返回 true，否则返回 false
 * @details 用于过滤无效的LiDAR回波（无回波、最大量程）。
 *          非常大的值表示测量失败。
 */
#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

/**
 * @enum LID_TYPE
 * @brief LiDAR传感器类型标识符
 * @details 用于选择适当的点云处理器。
 *          每个制造商都有自定义点格式，需要专门解包。
 */
enum LID_TYPE
{
   AVIA = 1,    /**< Livox Avia/MID系列 */
   VELO16,      /**< Velodyne VLP-16/32 */
   OUST64,      /**< Ouster OS-1/OS-2 */
   MID360,       /**< Livox MID360 */
   HESAIxt16,    /**< Hesai HESAIxt16 */
   UNILIDAR,     /**< UNILIDAR/宇树LiDAR */
}; //{1, 2, 3, 4, 5, 6}

enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};


/**
 * @enum Feature
 * @brief 点云分割的特征分类
 * @details 在预处理中对点进行分类，用于SLAM：
 * - Nor: 普通/未知点
 * - Poss_Plane: 可能的平面点（候选）
 * - Real_Plane: 确认的平面点
 * - Edge_Jump: 尖锐边缘或深度不连续
 * - Edge_Plane: 平面上的边缘点
 * - Wire: 细线/边缘特征
 * - ZeroPoint: 无效/零点
 */
enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};

/**
 * @enum Surround
 * @brief 邻域查询的方向指示符
 * @details 在边缘/平面检测中用于检查扫描线或有序点云中的
 *          前一个或后一个点。
 */
enum Surround
{
   Prev,  /**< 前一个/左/较低索引 */
   Next   /**< 后一个/右/较高索引 */
};

/**
 * @enum E_jump
 * @brief 边缘跳变分类
 * @details 描述在某点检测到的不连续性类型：
 * - Nr_nor: 正常（无跳变）
 * - Nr_zero: 遇到零值/NaN值
 * - Nr_180: 180度方向反转
 * - Nr_inf: 无限跳变（无有效邻域）
 * - Nr_blind: 盲区/数据缺失区域
 */
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

/**
 * @struct orgtype
 * @brief LiDAR点的每点特征信息
 * @details 存储用于分割的每个点的计算特征。
 *          由 give_feature() 方法填充。
 */
struct orgtype
{
   double range;           /**< 距传感器原点的距离 */
   double dista;           /**< 到最近邻的距离？ */
   double angle[2];        /**< 角坐标（方位角、俯仰角？） */
   double intersect;       /**< 平面拟合的交点参数 */
   E_jump edj[2];          /**< Prev和Next方向上的跳变类型 */
   Feature ftype;          /**< 分类的特征类型 */

   /**
    * @brief 默认构造函数
    * @details 初始化所有成员为默认值。
    */
   orgtype()
   {
     range = 0;
     edj[Prev] = Nr_nor;
     edj[Next] = Nr_nor;
     ftype = Nor;
     intersect = 2;
   }
};

/**
 * @namespace velodyne_ros
 * @brief Velodyne特有的点类型定义
 * @details 扩展PCL点以包含Velodyne特有字段：时间戳和环号。
 */
namespace velodyne_ros {
   /**
    * @struct Point
    * @brief 带时间和环信息的Velodyne点
    * @details 包含标准XYZI加上时间戳和激光通道ID。
    *          对齐到16字节以支持SIMD（EIGEN_ALIGN16）。
    */
   struct EIGEN_ALIGN16 Point {
       PCL_ADD_POINT4D;           /**< 添加x,y,z和填充以达到16字节 */
       float intensity;           /**< 回波强度 */
       float time;                /**< 扫描内时间戳（微秒） */
       uint16_t ring;             /**< 激光通道/环号（0-63） */
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW  /**< 启用EIGEN_MAKE_ALIGNED_OPERATOR_NEW以支持Eigen对齐 */
   };
}  // namespace velodyne_ros

/**
 * @brief 注册 velodyne_ros::Point 为PCL点类型
 * @details 宏展开为ROS消息转换所需的typedef和序列化代码。
 *          使PCL能够理解 velodyne_ros::Point 的结构布局。
 */
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

/**
 * @namespace ouster_ros
 * @brief Ouster特有的点类型定义
 * @details Ouster点包含时间戳（t）、反射率、环境光、距离。
 */
namespace ouster_ros {
   struct EIGEN_ALIGN16 Point {
       PCL_ADD_POINT4D;           /**< 添加x,y,z和填充 */
       float intensity;           /**< 强度（反射强度） */
       uint32_t t;                /**< 时间戳（纳秒？） */
       uint16_t reflectivity;     /**< 反射率测量值 */
       uint8_t  ring;             /**< 激光通道 */
       uint16_t ambient;          /**< 环境光测量值 */
       uint32_t range;            /**< 距离测量值 */
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

/**
 * @namespace livox_ros
 * @brief Livox特有的点类型定义
 * @details Livox使用非重复扫描模式；点包含标签和线号。
 */
namespace livox_ros
{
/**
 * @struct LivoxPointXyzrtl
 * @brief Livox点，包含反射率、标签和线ID
 */
typedef struct {
  float x;            /**< X轴，单位：米 */
  float y;            /**< Y轴，单位：米 */
  float z;            /**< Z轴，单位：米 */
  float reflectivity; /**< 反射率值 */
  uint8_t tag;        /**< Livox点标签（例如弱回波/强回波） */
  uint8_t line;       /**< 激光线ID */
} LivoxPointXyzrtl;

/**
 * @struct LivoxPointXyzitl
 * @brief Livox点，使用强度而非反射率
 */
typedef struct {
  float x;            /**< X轴，单位：米 */
  float y;            /**< Y轴，单位：米 */
  float z;            /**< Z轴，单位：米 */
  float intensity;    /**< 强度值 */
  uint8_t tag;        /**< Livox点标签 */
  uint8_t line;       /**< 激光线ID */
} LivoxPointXyzitl;
}


POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzrtl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzitl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)


/**
 * @namespace hesai_ros
 * @brief Hesai特有的点类型定义
 * @details Hesai点包含时间戳、强度、环号。
 */
namespace hesai_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;           /**< 添加x,y,z和填充 */
      float intensity;           /**< 强度（反射强度） */
      double timestamp;           /**< 时间戳（秒） */
      uint16_t ring;             /**< 激光通道/环号（0-15） */
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace hesai_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
   (float, x, x)
   (float, y, y)
   (float, z, z)
   (float, intensity, intensity)
   (double, timestamp, timestamp)
   (std::uint16_t, ring, ring)
)


/**
 * @namespace  Unilidar 
 * @brief  Unilidar 特有的点类型定义
 * @details Unilidar
 */
namespace unilidar_ros {
  struct Point
  {
    PCL_ADD_POINT4D;           /**< 添加x,y,z和填充 */
    PCL_ADD_INTENSITY;         /**< 添加强度 */
    std::uint16_t ring;
    float time;                /**< 时间戳（秒） */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  } EIGEN_ALIGN16;
  }
  POINT_CLOUD_REGISTER_POINT_STRUCT(unilidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
  )




/**
 * @class Preprocess
 * @brief LiDAR点云的主预处理类
 * @details 处理多种LiDAR格式，执行特征提取（地面、边缘、表面），
 *          并为SLAM前端输出有序点云。
 *          支持多条扫描线的并行处理。
 *
 * @section Processing_Flow
 * 1. 接收原始点云消息（CustomMsg或PointCloud2）
 * 2. 将点解包为PointCloudXYZI格式
 * 3. 按扫描线组织点（如果适用）
 * 4. 对每个点计算局部几何（边缘/平面）
 * 5. 分类点并分离到pl_full、pl_corn、pl_surf
 * 6. 发布或返回处理后的点云
 *
 * @section Configuration
 * 使用 set() 配置：特征提取开关、LiDAR类型、
 * 盲区半径、点过滤数量。
 */
class Preprocess
{
  public:
    //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // commented out - may be needed for Eigen vector storage

     /**
      * @brief 构造函数
      * @details 初始化所有成员变量为默认值。
      *          根据 lidar_type（如果已知）设置 N_SCANS，分配缓冲区。
      */
     Preprocess();
     
     /**
      * @brief 析构函数
      * @details 释放任何动态分配的内存（如果有）。
      */
     ~Preprocess();
     
     /**
      * @brief 处理 Livox CustomMsg 为输出点云
      * @param msg Livox自定义消息的UniquePtr
      * @param pcl_out 输出点云（PointCloudXYZI::Ptr）
      * @details 解包Livox特定格式，按线组织，调用 give_feature()。
      */
     void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
     
     /**
      * @brief 处理标准 PointCloud2 消息
      * @param msg PointCloud2消息的UniquePtr
      * @param pcl_out 输出点云
      * @details 根据 lidar_type 处理 Velodyne、Ouster 和通用点云。
      */
     void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
     
     /**
      * @brief 配置预处理参数
      * @param feat_en 启用/禁用特征提取（true=提取边缘/表面）
      * @param lid_type LiDAR类型枚举（AVIA, VELO16, OUST64, MID360, HESAIxt16, UNILIDAR）
      * @param bld 盲区半径（米）- 忽略比该值更接近传感器原点的点
      * @param pfilt_num 点过滤数量 - 降采样因子？（每N个点保留一个）
      * @details 设置 lidar_type、blind、point_filter_num、feature_enabled。
      *          在首次 process() 调用前调用。
      */
     void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    // sensor_msgs::PointCloud2::ConstPtr pointcloud;

     PointCloudXYZI pl_full;        /**< 输出：完整点云（过滤后） */
     PointCloudXYZI pl_corn;        /**< 输出：角点/边缘特征点 */
     PointCloudXYZI pl_surf;        /**< 输出：表面/平面特征点 */
     PointCloudXYZI pl_buff[128];   /**< 有序点存储缓冲区（每扫描线） */
                                /**< 支持最多128条扫描线（高分辨率LiDAR） */
     vector<orgtype> typess[128];   /**< 每扫描线每点的特征信息 */
     int lidar_type;                /**< 当前LiDAR类型（来自LID_TYPE枚举） */
     int point_filter_num;          /**< 点过滤器：每N个点保留一个（下采样） */
     int N_SCANS;                   /**< 扫描线/通道数量 */
     int SCAN_RATE;
     double blind;                  /**< 盲区半径：忽略比该值更近的点（米） */
     double max_scan_range;         /**< 最大有效量程（米） */
     bool feature_enabled;          /**< 特征提取主开关 */
     bool given_offset_time;        /**< 如果为true，使用提供的的时间戳偏移 */
     float time_unit_scale;
     int time_unit;
    // ros::Publisher pub_full, pub_surf, pub_corn;  // Publishers (ROS 1 style, commented)
  
   private:
     /**
      * @brief 处理 Livox CustomMsg
      * @param msg Livox消息
      * @details 解包Livox点，按线组织，调用 give_feature()。
      */
     void avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg);
     
     /**
      * @brief 处理 Ouster PointCloud2
      * @param msg Ouster消息
      * @details 解包Ouster特定字段（t、reflectivity、ambient、range）。
      */
     void oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
     
     /**
      * @brief 处理 Velodyne PointCloud2
      * @param msg Velodyne消息
      * @details 解包标准Velodyne数据包，按环组织。
      */
     void velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
     
     /**
      * @brief 处理 Livox MID360
      * @param msg MID360消息
      * @details 类似于 avia_handler，但MID360有不同的点格式。
      */
     void mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);

     /**
      * @brief 处理 Hesai HESAIxt16
      * @param msg HESAIxt16消息
      * @details 类似于 avia_handler，但HESAIxt16有不同的点格式。
      */
     void hesai_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);

     /**
      * @brief 处理 UNILIDAR/宇树LiDAR
      * @param msg UNILIDAR/宇树LiDAR消息
      * @details 类似于 avia_handler，但UNILIDAR/宇树LiDAR有不同的点格式。
      */
     void unilidar_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
    
     
     /**
      * @brief 默认/未处理传感器处理器
      * @param msg 通用 PointCloud2
      * @details 未知传感器类型的后备处理。使用标准PCL转换。
      */
     void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
     
     /**
      * @brief 从有序点云提取特征
      * @param pl 输入/输出点云
      * @param types 每点特征信息向量
      * @details 主要特征提取例程。遍历所有点，
      *          使用邻域点计算局部曲率和法线，
      *          基于阈值分类为平面/边缘/未知。
      */
     void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
     
     /**
      * @brief 发布处理后的点云（存根）
      * @param pl 要发布点云
      * @param ct 时间戳
      * @details 如果启用发布器会发布到ROS话题。
      *          当前可能未使用或用于调试。
      */
     void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
     
     /**
      * @brief 判断点是否属于平面表面
      * @param pl 输入点云（有序）
      * @param types 特征类型数组（通过引用输出）
      * @param i 当前点索引
      * @param i_nex 下一个点索引（输出）
      * @param curr_direct 当前平面法线方向（输出）
      * @return 平面类型：0=无，1=可能，2=确认
      * @details 使用邻域局部PCA估计平面。
      *          检查点分布和曲率以决定平面归属。
      *          如果找到平面则填充 curr_direct。
      */
     int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
     
     /**
      * @brief 检查小平面子区域
      * @param pl 输入点云
      * @param types 特征类型数组
      * @param i_cur 当前点索引
      * @param i_nex 下一个索引（输出）
      * @param curr_direct 当前方向向量（输出）
      * @return 如果检测到小平面的返回 true
      * @details 更小尺度的平面检测，使用更严格的阈值。
      *          用于详细表面分割。
      */
     bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
     
     /**
      * @brief 检测点云中的边缘/不连续性
      * @param pl 输入点云
      * @param types 特征类型数组
      * @param i 当前点索引
      * @param nor_dir 检查方向（Prev或Next）
      * @return 如果检测到边缘跳变返回 true
      * @details 检查点i和nor_dir方向上的邻域是否形成
      *          边缘（大深度不连续或法线变化）。
      */
     bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

     // ============ 配置参数（针对传感器调优） ============

     int group_size;          /**< 特征计算中邻域点的分组大小 */
     double disA, disB;       /**< 边缘检测的距离阈值A和B */
     double inf_bound;        /**< 无效测量的无穷界值 */
     double limit_maxmid;     /**< 限制：最大-中间差值阈值 */
     double limit_midmin;     /**< 限制：中间-最小差值阈值 */
     double limit_maxmin;     /**< 限制：最大-最小差值阈值 */
     double p2l_ratio;        /**< 点到直线距离比率阈值 */
     double jump_up_limit;    /**< 边缘检测的上跳变限制 */
     double jump_down_limit;  /**< 边缘检测的下跳变限制 */
     double cos160;           /**< 160度的余弦值（用于角度测试） */
     double edgea, edgeb;     /**< 边缘检测参数a和b */
     double smallp_intersect; /**< 小平面交点阈值 */
     double smallp_ratio;     /**< 小平面比率阈值 */
     double vx, vy, vz;       /**< 速度/平移分量？（可能用于运动补偿） */
};
