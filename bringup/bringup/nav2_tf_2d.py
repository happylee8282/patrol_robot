#!/usr/bin/env python3
"""Publish a planar TF and odometry branch for Nav2 from FAST-LIO data."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


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
    """Create map->odom->base_link as a planar copy of the FAST-LIO chain."""

    def __init__(self):
        super().__init__('nav2_tf_2d')
        self.declare_parameter('lio_odom_topic', '/Odometry')
        self.declare_parameter('map_correction_topic', '/map_to_odom')
        self.declare_parameter('nav_odom_topic', '/odom_2d')

        self._map_correction = None
        self._broadcaster = TransformBroadcaster(self)
        self._odom_publisher = self.create_publisher(
            Odometry,
            self.get_parameter('nav_odom_topic').value,
            20,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('map_correction_topic').value,
            self._map_correction_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('lio_odom_topic').value,
            self._lio_odom_callback,
            20,
        )

    def _map_correction_callback(self, message):
        self._map_correction = message

    def _lio_odom_callback(self, message):
        stamp = message.header.stamp

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        if self._map_correction is not None:
            pose = self._map_correction.pose.pose
            map_to_odom.transform.translation.x = pose.position.x
            map_to_odom.transform.translation.y = pose.position.y
            set_yaw(
                map_to_odom.transform.rotation,
                yaw_from_quaternion(pose.orientation),
            )
        else:
            map_to_odom.transform.rotation.w = 1.0

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        odom_to_base.transform.translation.x = message.pose.pose.position.x
        odom_to_base.transform.translation.y = message.pose.pose.position.y
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        set_yaw(odom_to_base.transform.rotation, yaw)

        self._broadcaster.sendTransform([map_to_odom, odom_to_base])

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
