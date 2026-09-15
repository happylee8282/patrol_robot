#!/usr/bin/env python3
"""
Livox激光雷达坐标校正节点

此节点用于校正Livox激光雷达传感器安装方向不正导致的坐标偏差。
如果传感器倒装或非标准安装，此节点会对点云数据和IMU测量值
进行坐标轴翻转，使其与机器人基坐标系对齐。

功能原理:
    Livox传感器可能以不同于机器人基坐标系的方向安装
    (例如绕X轴旋转180°)。此节点应用必要的坐标变换来对齐
    传感器数据与机器人坐标系。
    
    具体而言，如果传感器倒装，校正为: y → -y 和 z → -z
    (绕X轴旋转180°的变换)。
    
    对应的旋转矩阵:
        R_x(π) = [1   0    0]
                 [0  -1    0]
                 [0   0   -1]

订阅:
    /livox/inverted_lidar: Y和Z轴被反转的点云(PointCloud2或CustomMsg)
    /livox/inverted_imu: Y和Z轴角速度被反转的IMU数据

发布:
    /livox/lidar: 坐标校正后的点云(标准方向)
    /livox/imu: 坐标校正后的IMU数据(标准方向)

参数:
    xfer_format (int): 消息格式(0=PointCloud2, 1=CustomMsg)

作者: FAST-LIO团队
日期: 2024
许可证: MIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

# 定义QoS配置文件，确保可靠通信并配备足够缓冲区
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # 确保可靠的消息传递
    history=HistoryPolicy.KEEP_LAST,        # 保留最近的消息
    depth=10                                # 增加缓冲区大小(10条消息)
)


class LivoxLaserToPointcloud(Node):
    """
    校正Livox传感器坐标方向的节点。
    
    此节点订阅Livox点云和IMU数据(已被反转，可能是由于驱动或
    硬件级变换)，然后重新应用正确的变换以标准化坐标系。
    
    Livox传感器数据结构(二进制格式):
        偏移量  类型      名称
        0      float32  x      (X坐标)
        4      float32  y      (Y坐标)
        8      float32  z      (Z坐标)
        12     float32  intensity (信号强度/反射率)
        16     uint8    tag     (点标签/分类)
        17     uint8    line    (激光线号)
        18     float64  timestamp (点时间戳)
    
    属性:
        LIVOX_DTYPE (numpy.dtype): 匹配Livox点格式的结构化NumPy数据类型
        xfer_format (int): 消息格式(0=PointCloud2, 1=CustomMsg)
    """
    
    # 定义NumPy结构化数组数据类型，匹配Livox点格式
    # 这允许从二进制缓冲区到数组的零拷贝映射
    LIVOX_DTYPE = np.dtype([
        ('x', 'f4'),         # 偏移量 0:   X坐标(float32)
        ('y', 'f4'),         # 偏移量 4:   Y坐标(float32)
        ('z', 'f4'),         # 偏移量 8:   Z坐标(float32)
        ('intensity', 'f4'), # 偏移量 12:  强度/反射率(float32)
        ('tag', 'u1'),       # 偏移量 16:  点标签/类型(uint8)
        ('line', 'u1'),      # 偏移量 17:  激光线号(uint8)
        ('timestamp', 'f8')  # 偏移量 18:  时间戳(float64, 微秒)
        ])
    
    def __init__(self):
        """初始化Livox校正节点，配置订阅和发布。"""
        super().__init__("Invert_Livox_Scan")

        # 加载传输格式参数(决定消息类型)
        xfer_format = self.declare_parameter("xfer_format", 0).value

        if xfer_format == 0:
            # 格式0: 标准ROS PointCloud2消息
            self.pub_scan = self.create_publisher(PointCloud2, "/livox/lidar", qos_profile=qos_profile)
            self.sub_scan = self.create_subscription(
                PointCloud2, "/livox/inverted_lidar", self.pointcloud2_callback, 
                qos_profile=qos_profile
            )

        elif xfer_format == 1:
            # 格式1: Livox CustomMsg(更高效，包含额外字段)
            self.pub_scan = self.create_publisher(CustomMsg, "/livox/lidar", qos_profile=qos_profile)
            self.sub_scan = self.create_subscription(
                CustomMsg, "/livox/inverted_lidar", self.custom_msg_callback, 
                qos_profile=qos_profile
            )

        else:
            # 无效格式: 记录错误并关闭节点
            self.get_logger().error(f"方法未定义，xfer_format = {xfer_format}")
            self.destroy_node()
            return

        # 创建IMU发布器和订阅器
        self.pub_imu = self.create_publisher(Imu, "/livox/imu", qos_profile=qos_profile)
        self.sub_imu = self.create_subscription(
            Imu, "/livox/inverted_imu", self.imu_callback, 
            qos_profile=qos_profile
        )

    def pointcloud2_callback(self, msg: PointCloud2) -> None:
        """
        PointCloud2格式消息的回调函数。
        
        此方法:
        1. 将原始消息数据缓冲区映射到结构化NumPy数组
        2. 应用坐标反转(y → -y, z → -z)
        3. 序列化回字节并重新发布
        
        该变换是向量化的，以实现高性能(无循环)。
        
        参数:
            msg (sensor_msgs.msg.PointCloud2): 反转的点云消息
        """
        # 步骤1: 将消息数据缓冲区映射到结构化数据类型
        # np.frombuffer创建零拷贝视图; .copy()使其可写
        data = np.frombuffer(msg.data, dtype=self.LIVOX_DTYPE).copy()

        # 步骤2: 应用坐标反转(向量化运算)
        # 这些操作同时修改所有点
        data['y'] = -data['y']  # 反转Y轴
        data['z'] = -data['z']  # 反转Z轴
        # X轴保持不变

        # 步骤3: 使用修改后的数据重建消息
        # 复制原始消息以保留元数据(头部、字段等)
        out_msg = msg 
        out_msg.data = data.tobytes()  # 将结构化数组转换回字节
        
        # 发布校正后的点云
        self.pub_scan.publish(out_msg)

    def custom_msg_callback(self, msg: CustomMsg) -> None:
        """
        Livox CustomMsg格式的回调函数。
        
        Livox CustomMsg包含Point对象列表。此方法遍历每个点
        并反转y和z坐标。
        
        参数:
            msg (livox_ros_driver2.msg.CustomMsg): 反转的自定义消息
        """
        # 遍历消息中的每个点
        for p in msg.points:
            p.y = -p.y  # 反转Y坐标
            p.z = -p.z  # 反转Z坐标
            
        # 可选: 更新时间戳(已注释)
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.timebase = int(str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec))
        
        # 发布校正后的消息
        self.pub_scan.publish(msg)

    def imu_callback(self, msg: Imu) -> None:
        """
        IMU数据的回调函数。
        
        反转角速度的Y和Z分量以匹配校正后的点云坐标系。
        
        注意:
            线性加速度的Z分量不被反转，因为重力方向应保持
            在世界坐标系中一致。注释行显示了备选方案。
        
        参数:
            msg (sensor_msgs.msg.Imu): 反转的IMU测量值
        """
        # 反转角速度分量(Y和Z)
        msg.angular_velocity.y = -msg.angular_velocity.y
        msg.angular_velocity.z = -msg.angular_velocity.z
        
        # 注意: 线性加速度Z通常不被反转，因为重力在标准机器人坐标系
        # 中沿-Z方向作用。如果您的应用需要则取消注释:
        # msg.linear_acceleration.z = -msg.linear_acceleration.z
        
        # 可选: 更新时间戳(已注释)
        # msg.header.stamp = self.get_clock().now().to_msg()

        # 发布校正后的IMU数据
        self.pub_imu.publish(msg)


def main(args=None):
    """Livox反转节点的主入口点。"""
    rclpy.init(args=args)
    node = LivoxLaserToPointcloud()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()