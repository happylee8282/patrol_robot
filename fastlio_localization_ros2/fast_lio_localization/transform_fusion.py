#!/usr/bin/env python3
"""
Transform Fusion Node for FAST-LIO Localization

此节点融合坐标变换以提供统一的全局定位估计。它结合：
- odometry数据（baselink到odom帧）
- map到odom的变换（来自全局定位）
- 发布TF变换和odometry消息

订阅:
    /Odometry: 来自里程计传感器的odometry (odom -> baselink)
    /map_to_odom: 来自全局定位的估计变换 (map -> odom)

发布:
    /localization: map坐标系下的融合odometry (map -> body)
    TF广播: map -> camera_init变换

作者: FAST-LIO团队
日期: 2024
许可证: MIT
"""

import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import rclpy.timer
import tf_transformations
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Header


class TransformFusion(Node):
    """
    ROS2节点，用于融合odometry和map变换。
    
    此节点维护当前的odometry (odom -> baselink) 和
    map到odom (map -> odom) 变换，然后计算并
    发布融合后的变换 (map -> baselink/body)。
    
    融合逻辑:
    1. 接收来自轮式编码器+IMU的odometry (odom -> baselink)
    2. 接收来自全局定位(ICP)的map -> odom
    3. 计算 map -> baselink = (map -> odom) * (odom -> baselink)
    4. 广播TF变换 (map -> camera_init) 用于可视化
    5. 发布Odometry消息 (map -> body) 用于导航
    
    属性:
        cur_odom_to_baselink (Odometry): 最新的odometry消息 (odom -> baselink)
        cur_map_to_odom (Odometry): 最新的map -> odom变换
        tf_broadcaster (tf2_ros.TransformBroadcaster): TF变换广播器
        pub_localization (rclpy.publisher.Publisher): 融合定位的发布器
        freq_pub_localization (float): 发布频率，单位Hz (默认10Hz)
        timer (rclpy.timer.Timer): 定时器，周期触发融合更新
    """
    
    def __init__(self):
        """初始化TransformFusion节点，配置订阅和发布。"""
        super().__init__("transform_fusion")

        # 初始化变换存储为None（尚无数据）
        self.cur_odom_to_baselink = None  # 存储最新的odometry (odom -> baselink)
        self.cur_map_to_odom = None       # 存储最新的map -> odom变换

        # 创建TF广播器，用于发布坐标系变换
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 创建发布器，用于发布融合后的定位odometry (map -> body)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        # 订阅odometry话题（通常来自LIO-SAM或类似系统）
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 1)
        
        # 订阅map -> odom变换话题（来自全局定位节点）
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)

        # 设置融合频率为10Hz（对大多数定位任务足够）
        self.freq_pub_localization = 10
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)
        # 备选方案: threading.Thread(target=self.transform_fusion, daemon=True).start()

    def pose_to_mat(self, pose_msg) -> np.ndarray:
        """
        将ROS Pose消息转换为4x4齐次变换矩阵。
        
        将位置(x, y, z)和方向(四元数: x, y, z, w)
        转换为齐次变换矩阵：
        
            T = [R(3x3)  t(3x1)]
                [0 0 0       1   ]
        
        其中R是由四元数导出的旋转矩阵，
        使用tf_transformations.quaternion_matrix()。
        
        参数:
            pose_msg (geometry_msgs.msg.Pose): 包含位置和方向(四元数)的输入位姿
            
        返回:
            numpy.ndarray: 4x4齐次变换矩阵
            
        示例:
            >>> T = pose_to_mat(pose_msg)
            >>> print(T.shape)
            (4, 4)
        """
        # 创建4x4单位矩阵
        trans = np.eye(4)
        
        # 设置平移分量（右上角3x1）
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        
        # 将四元数转换为旋转矩阵
        # 四元数格式: [x, y, z, w]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, 
                pose_msg.orientation.z, pose_msg.orientation.w]
        
        # tf_transformations.quaternion_matrix返回4x4矩阵
        # 我们提取左上角3x3旋转部分
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        
        return trans

    def transform_fusion(self) -> None:
        """
        主融合回调函数：计算并发布融合后的变换。
        
        此方法由定时器周期调用（10Hz）。它:
        1. 检查所需的变换是否可用
        2. 构建map -> odom变换矩阵
        3. 广播map -> camera_init TF变换
        4. 通过矩阵乘法计算map -> body变换
        5. 发布融合后的odometry消息
        
        数学运算:
        
        令:
            T_mo = map -> odom变换 (来自全局定位)
            T_ob = odom -> baselink变换 (来自odometry)
            
        则:
            T_mb = T_mo * T_ob  (map -> baselink/body)
            
        TF链: map -> odom -> baselink -> body
        
        注意:
            如果未提供map -> odom，则假设为单位矩阵（map = odom）。
            如果未提供odometry，则提前退出（数据不足）。
        """
        # 如果odometry数据尚未接收则提前退出
        if self.cur_odom_to_baselink is None:
            return

        # 确定map -> odom变换
        # 如果全局定位提供估计则使用它；否则假设为单位矩阵
        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        # --------------------------------------------------------------------
        # 步骤1: 广播map -> camera_init TF变换
        # --------------------------------------------------------------------
        transform_msg = Transform()
        
        # 从变换矩阵中提取平移分量
        transform_msg.translation.x = T_map_to_odom[0, 3]
        transform_msg.translation.y = T_map_to_odom[1, 3]
        transform_msg.translation.z = T_map_to_odom[2, 3]
        
        # 将旋转矩阵转换为四元数 [x, y, z, w]
        quat = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        transform_msg.rotation.x = quat[0]
        transform_msg.rotation.y = quat[1]
        transform_msg.rotation.z = quat[2]
        transform_msg.rotation.w = quat[3]
        
        # 创建带当前时间戳的头部
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.cur_odom_to_baselink.header.frame_id
        
        # 构建用于TF广播的TransformStamped消息
        transform_stamped_msg = tf2_ros.TransformStamped(
                header=self.cur_odom_to_baselink.header,
                child_frame_id="camera_init",
                transform=transform_msg
            )
        # 覆盖frame_id为"map"（这是父坐标系）
        transform_stamped_msg.header.frame_id = "map"
        
        # 广播变换，使所有ROS节点都能使用它
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        # --------------------------------------------------------------------
        # 步骤2: 计算并发布融合后的odometry (map -> body)
        # --------------------------------------------------------------------
        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            # 将odometry (odom -> baselink) 转换为矩阵
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            
            # 矩阵乘法: T_mb = T_mo * T_ob
            # 这给出了从map到body的完整变换
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

            # 从矩阵中提取平移分量
            xyz = tf_transformations.translation_from_matrix(T_map_to_base_link)
            
            # 从矩阵中提取四元数
            quat = tf_transformations.quaternion_from_matrix(T_map_to_base_link)

            # 构建Odometry消息
            localization = Odometry()
            localization.pose.pose = Pose(
                position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            )
            
            # 保留原始odometry中的速度/扭曲信息
            localization.twist = cur_odom.twist

            # 设置头部元数据
            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = "map"
            localization.child_frame_id = "body"
            
            # 发布融合后的定位估计
            self.pub_localization.publish(localization)

    def cb_save_cur_odom(self, msg):
        """
        /Odometry订阅的回调函数。
        
        存储最新的odometry消息 (odom -> baselink变换)。
        这通常来自轮式编码器+IMU积分。
        
        参数:
            msg (nav_msgs.msg.Odometry): odom坐标系下的odometry消息
        """
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        """
        /map_to_odom订阅的回调函数。
        
        存储最新的map -> odom变换估计。
        这通常来自全局定位(ICP匹配)。
        
        参数:
            msg (nav_msgs.msg.Odometry): map到odom坐标系的变换
        """
        self.cur_map_to_odom = msg


def main(args=None):
    """transform_fusion节点的主入口点。"""
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
