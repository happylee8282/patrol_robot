#!/usr/bin/env python3
"""Publish the IMU/LiDAR mounting TF and a PointCloud2 RViz preview."""

import math
import os

from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import StaticTransformBroadcaster
import yaml


DEFAULT_FAST_LIO_CONFIG = (
    '/home/unicon/nav_ws/src/fastlio_localization_ros2/config/mid360.yaml'
)


def rotation_matrix_to_quaternion(values):
    """Convert a row-major 3x3 LiDAR-to-IMU rotation to (x, y, z, w)."""
    if len(values) != 9:
        raise ValueError('mapping.extrinsic_R must contain nine values')
    m = [float(value) for value in values]
    trace = m[0] + m[4] + m[8]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((m[7] - m[5]) / s, (m[2] - m[6]) / s,
             (m[3] - m[1]) / s, 0.25 * s)
    elif m[0] > m[4] and m[0] > m[8]:
        s = math.sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0
        q = (0.25 * s, (m[1] + m[3]) / s,
             (m[2] + m[6]) / s, (m[7] - m[5]) / s)
    elif m[4] > m[8]:
        s = math.sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0
        q = ((m[1] + m[3]) / s, 0.25 * s,
             (m[5] + m[7]) / s, (m[2] - m[6]) / s)
    else:
        s = math.sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0
        q = ((m[2] + m[6]) / s, (m[5] + m[7]) / s,
             0.25 * s, (m[3] - m[1]) / s)
    norm = math.sqrt(sum(component * component for component in q))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError('mapping.extrinsic_R is not a valid rotation')
    return tuple(component / norm for component in q)


def transform(parent, child, translation=(0.0, 0.0, 0.0),
              quaternion=(0.0, 0.0, 0.0, 1.0)):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message


class TfLink(Node):
    """Keep the sensor TF aligned with FAST-LIO's input extrinsics."""

    def __init__(self):
        super().__init__('tf_link')
        self.declare_parameter('fast_lio_config', DEFAULT_FAST_LIO_CONFIG)
        self.declare_parameter('sensor_height', 0.72)
        self.declare_parameter('preview_stride', 4)

        config_path = os.path.expanduser(
            self.get_parameter('fast_lio_config').value
        )
        with open(config_path, encoding='utf-8') as config_stream:
            mapping = yaml.safe_load(config_stream)['/**']['ros__parameters']['mapping']
        lidar_translation = mapping['extrinsic_T']
        if len(lidar_translation) != 3:
            raise ValueError('mapping.extrinsic_T must contain three values')
        lidar_quaternion = rotation_matrix_to_quaternion(mapping['extrinsic_R'])
        sensor_height = float(self.get_parameter('sensor_height').value)
        self._preview_stride = int(self.get_parameter('preview_stride').value)
        if sensor_height < 0 or self._preview_stride < 1:
            raise ValueError('sensor_height must be >= 0 and preview_stride >= 1')

        self._static_tf = StaticTransformBroadcaster(self)
        transforms = [
            # FAST-LIO's body origin is the IMU center.  The IMU axes are
            # assumed aligned with the robot's x-forward/y-left/z-up axes.
            transform('body', 'base_link'),
            transform('base_link', 'imu_link'),
            transform('base_link', 'livox_frame', lidar_translation,
                      lidar_quaternion),
            # The floor reference is 72 cm below both sensor origins.
            transform('base_link', 'base_footprint',
                      (0.0, 0.0, -sensor_height)),
        ]
        stamp = self.get_clock().now().to_msg()
        for item in transforms:
            item.header.stamp = stamp
        self._static_tf.sendTransform(transforms)

        self._fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        self._preview = self.create_publisher(
            PointCloud2, '/livox/points',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            CustomMsg, '/livox/lidar', self._publish_preview,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'LiDAR relative to IMU: x={lidar_translation[0]:.3f} m, '
            f'y={lidar_translation[1]:.3f} m, '
            f'z={lidar_translation[2]:.3f} m; '
            f'floor reference {sensor_height:.3f} m below IMU'
        )

    def _publish_preview(self, message):
        # Avoid conversion cost when RViz (or another preview subscriber) is
        # closed. FAST-LIO still receives the full CustomMsg independently.
        if self._preview.get_subscription_count() == 0:
            return
        points = (
            (point.x, point.y, point.z, float(point.reflectivity))
            for point in message.points[::self._preview_stride]
        )
        cloud = point_cloud2.create_cloud(message.header, self._fields, points)
        self._preview.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = TfLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
