#!/usr/bin/env python3

import os
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PcdToPointCloud(Node):
    def __init__(self):
        super().__init__('pcd_to_pointcloud')

        self.declare_parameter('file_name', '')
        self.declare_parameter('tf_frame', 'map')
        self.declare_parameter('cloud_topic', 'cloud_pcd')
        self.declare_parameter('period_ms_', 1000)

        file_name   = self.get_parameter('file_name').get_parameter_value().string_value
        tf_frame    = self.get_parameter('tf_frame').get_parameter_value().string_value
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        period_ms   = self.get_parameter('period_ms_').get_parameter_value().integer_value

        self.tf_frame_   = tf_frame
        self.cloud_msg_  = None

        # Latched QoS：新订阅者连接时也能立即收到地图
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_ = self.create_publisher(PointCloud2, cloud_topic, qos)
        self.get_logger().info(
            f"Publishing on topic '{cloud_topic}' with frame_id '{tf_frame}', "
            f"period {period_ms} ms."
        )

        if not file_name:
            self.get_logger().error("Parameter 'file_name' is empty. Node will not publish.")
            return

        if not self._load_pcd(file_name):
            return

        period_sec = period_ms / 1000.0
        self.timer_ = self.create_timer(period_sec, self._timer_callback)

    # ------------------------------------------------------------------
    def _load_pcd(self, file_name: str) -> bool:
        """加载 PCD 文件并构建 PointCloud2 消息。优先使用 open3d，否则退回手写解析器。"""
        if not os.path.isfile(file_name):
            self.get_logger().error(f"PCD file not found: {file_name}")
            return False

        try:
            import open3d as o3d
            pcd       = o3d.io.read_point_cloud(file_name)
            pts       = np.asarray(pcd.points, dtype=np.float32)
            has_color = pcd.has_colors()
            colors    = (np.asarray(pcd.colors) * 255).astype(np.uint8) if has_color else None
            self.get_logger().info("Using open3d to load PCD.")
        except ImportError:
            self.get_logger().warn("open3d not found, falling back to built-in ASCII PCD parser.")
            pts, has_color, colors = self._parse_pcd_ascii(file_name)
            if pts is None:
                return False

        n = len(pts)
        if n == 0:
            self.get_logger().error("PCD file loaded but contains 0 points.")
            return False

        self.get_logger().info(f"Loaded {n} points from '{file_name}'.")

        if has_color and colors is not None:
            fields = [
                PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 16
            data = bytearray(n * point_step)
            for i, (pt, col) in enumerate(zip(pts, colors)):
                rgb_int = (int(col[0]) << 16) | (int(col[1]) << 8) | int(col[2])
                rgb_f   = struct.unpack('f', struct.pack('I', rgb_int))[0]
                struct.pack_into('ffff', data, i * point_step,
                                 float(pt[0]), float(pt[1]), float(pt[2]), rgb_f)
        else:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 12
            data = pts[:, :3].astype(np.float32).tobytes()

        msg = PointCloud2()
        msg.header.frame_id = self.tf_frame_
        msg.height      = 1
        msg.width       = n
        msg.fields      = fields
        msg.is_bigendian = False
        msg.point_step  = point_step
        msg.row_step    = point_step * n
        msg.data        = bytes(data)
        msg.is_dense    = True

        self.cloud_msg_ = msg
        return True

    def _parse_pcd_ascii(self, file_name: str):
        """内置的 ASCII 格式 PCD 解析器（不依赖任何第三方库）"""
        points = []
        try:
            with open(file_name, 'rb') as f:
                data_type = 'ascii'
                for raw_line in f:
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    if line.upper().startswith('DATA'):
                        data_type = line.split()[1].lower()
                        if data_type != 'ascii':
                            self.get_logger().error(
                                f"PCD data type '{data_type}' is not supported without open3d. "
                                "Please install open3d: pip install open3d"
                            )
                            return None, False, None
                        # 读取 ASCII 点数据
                        for row in f:
                            vals = row.decode('utf-8', errors='ignore').strip().split()
                            if len(vals) >= 3:
                                points.append([float(vals[0]), float(vals[1]), float(vals[2])])
                        break
        except Exception as e:
            self.get_logger().error(f"PCD parse error: {e}")
            return None, False, None

        if not points:
            self.get_logger().error("No points parsed from PCD file.")
            return None, False, None

        return np.array(points, dtype=np.float32), False, None

    # ------------------------------------------------------------------
    def _timer_callback(self):
        if self.cloud_msg_ is None:
            return
        # 与 C++ 版本一致：无订阅者时跳过发布
        if self.pub_.get_subscription_count() == 0:
            return
        self.cloud_msg_.header.stamp = self.get_clock().now().to_msg()
        self.pub_.publish(self.cloud_msg_)


def main(args=None):
    rclpy.init(args=args)
    node = PcdToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
