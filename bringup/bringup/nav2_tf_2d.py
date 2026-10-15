#!/usr/bin/env python3
"""Publish planar odometry for Nav2 from FAST-LIO data."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(quaternion):
    """Return Z-axis yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def set_yaw(quaternion, yaw):
    """Set a geometry_msgs Quaternion to a yaw-only rotation."""
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)


class Nav2Tf2D(Node):
    """Publish /odom_2d; TF is provided by FAST-LIO and the bringup launches."""

    def __init__(self):
        super().__init__('nav2_tf_2d')
        self.declare_parameter('lio_odom_topic', '/Odometry')
        self.declare_parameter('nav_odom_topic', '/odom_2d')

        self._odom_publisher = self.create_publisher(
            Odometry,
            self.get_parameter('nav_odom_topic').value,
            20,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('lio_odom_topic').value,
            self._lio_odom_callback,
            20,
        )

    def _lio_odom_callback(self, message):
        stamp = message.header.stamp
        yaw = yaw_from_quaternion(message.pose.pose.orientation)

        odom_2d = Odometry()
        odom_2d.header.stamp = stamp
        odom_2d.header.frame_id = 'odom'
        odom_2d.child_frame_id = 'base_link'
        odom_2d.pose.pose.position.x = message.pose.pose.position.x
        odom_2d.pose.pose.position.y = message.pose.pose.position.y
        set_yaw(odom_2d.pose.pose.orientation, yaw)
        odom_2d.pose.covariance = message.pose.covariance
        odom_2d.twist = message.twist
        odom_2d.twist.twist.linear.y = 0.0
        odom_2d.twist.twist.linear.z = 0.0
        odom_2d.twist.twist.angular.x = 0.0
        odom_2d.twist.twist.angular.y = 0.0
        self._odom_publisher.publish(odom_2d)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Tf2D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
