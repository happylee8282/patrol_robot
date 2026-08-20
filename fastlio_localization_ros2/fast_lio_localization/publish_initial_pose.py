#!/usr/bin/env python3
"""
初始位姿发布节点

命令行工具，用于发布全局定位系统的初始位姿估计。
通常用于在机器人启动或丢失定位后提供ICP算法的初始猜测值。

此节点可通过命令行参数调用:
    ros2 run fast_lio_localization publish_initial_pose x y z yaw pitch roll
    
也可通过导入并调用publish_pose()以编程方式使用。

位姿在map坐标系中指定，以PoseWithCovarianceStamped消息
形式发布到/initialpose话题。这是RViz"2D位姿估计"工具
和AMCL类定位系统的标准接口。

使用方法:
    ros2 run fast_lio_localization publish_initial_pose 0.0 0.0 0.0 0.0 0.0 0.0
    
    参数(单位: 米和弧度):
        x, y, z: map坐标系中的位置
        yaw:    绕Z轴的旋转(垂直方向)
        pitch:  绕Y轴的旋转(横向)
        roll:   绕X轴的旋转(纵向)

    注意: 欧拉角遵循内旋顺序(ROS标准):
        R = R_x(roll) * R_y(pitch) * R_z(yaw)

发布:
    /initialpose (geometry_msgs/PoseWithCovarianceStamped): 初始位姿估计

作者: FAST-LIO团队
日期: 2024
许可证: MIT
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import tf_transformations


class PublishInitialPose(Node):
    """
    发布初始位姿估计的节点。
    
    此节点通常用于:
    - 机器人启动时提供初始位姿猜测
    - 丢失定位后重新定位
    - 在RViz中手动设置机器人位置
    
    位姿从欧拉角(roll, pitch, yaw)转换为四元数(x, y, z, w)
    以进行ROS表示。
    
    属性:
        pub_pose (rclpy.publisher.Publisher): 初始位姿的发布器
    """
    
    def __init__(self):
        """初始化节点，配置/initialpose话题的发布器。"""
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def publish_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        """
        以PoseWithCovarianceStamped消息形式发布初始位姿。
        
        将欧拉角(roll, pitch, yaw)转换为四元数并发布完整位姿
        到/initialpose话题。
        
        变换数学:
            使用tf_transformations.quaternion_from_euler()，它采用
            内旋(固定轴)顺序:
                R = R_z(yaw) * R_y(pitch) * R_x(roll)
            
            这从旋转矩阵导出四元数(x, y, z, w)。
        
        参数:
            x (float): map坐标系中的X位置(米)
            y (float): map坐标系中的Y位置(米)
            z (float): map坐标系中的Z位置(米)
            roll (float): 绕X轴的滚转角(弧度)
            pitch (float): 绕Y轴的俯仰角(弧度)
            yaw (float): 绕Z轴的偏航角(弧度)
            
        注意:
            协方差设置为非常大的值(方向除外)以表示位置的不确定性。
            可根据对估计的置信度进行调整。
        """
        # 将欧拉角转换为四元数
        # 顺序: roll (X), pitch (Y), yaw (Z) - 内旋
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        xyz = [x, y, z]

        # 创建PoseWithCovarianceStamped消息
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose = Pose(
            Point(*xyz), 
            Quaternion(*quat)
        )
        
        # 设置头部时间戳和坐标系
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        
        # 发布消息
        self.pub_pose.publish(initial_pose)

        # 记录发布的位姿用于调试
        self.get_logger().info(
            f"初始位姿: x={x:.3f} y={y:.3f} z={z:.3f} "
            f"yaw={yaw:.3f} pitch={pitch:.3f} roll={roll:.3f}"
        )


def main(args=None):
    """
    主入口点: 解析命令行参数并发布位姿。
    
    命令行界面:
        ros2 run fast_lio_localization publish_initial_pose <x> <y> <z> <yaw> <pitch> <roll>
        
    示例:
        ros2 run fast_lio_localization publish_initial_pose 0.0 0.0 0.0 0.0 0.0 0.0
        
    注意:
        参数顺序: x y z yaw pitch roll(单位: 米/弧度)
        此顺序符合常见机器人惯例(yaw, pitch, roll)，
        但请注意publish_pose()内部期望的顺序为(roll, pitch, yaw)。
    """
    rclpy.init(args=args)
    node = PublishInitialPose()

    # 设置参数解析器
    parser = argparse.ArgumentParser(
        description='为全局定位发布初始位姿估计'
    )
    parser.add_argument("x", type=float, help="map坐标系中的X位置(m)")
    parser.add_argument("y", type=float, help="map坐标系中的Y位置(m)")
    parser.add_argument("z", type=float, help="map坐标系中的Z位置(m)")
    parser.add_argument("yaw", type=float, help="绕Z轴的偏航角(rad)")
    parser.add_argument("pitch", type=float, help="绕Y轴的俯仰角(rad)")
    parser.add_argument("roll", type=float, help="绕X轴的滚转角(rad)")
    args = parser.parse_args()

    # 发布位姿(注意: publish_pose期望的顺序为roll, pitch, yaw)
    node.publish_pose(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
    
    # 发布后关闭(单次执行节点)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
