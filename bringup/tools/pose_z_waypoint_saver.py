#!/usr/bin/env python3
"""RViz의 Initial/Goal Pose를 3D PCD 맵의 지면 Z와 함께 JSON으로 저장."""

import json
import os
from datetime import datetime
from pathlib import Path
from threading import Lock
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PoseZWaypointSaver(Node):
    def __init__(self):
        super().__init__('pose_z_waypoint_saver')

        # 저장된 PCD 맵 토픽을 기본값으로 사용한다.
        self.declare_parameter('map_cloud_topic', '/cloud_pcd')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter(
            'output_file',
            '~/nav_ws/src/bringup/config/point_map/final.json',
        )

        # 정적 PCD 맵은 보통 TRANSIENT_LOCAL QoS로 한 번 발행된다.
        self.declare_parameter('cloud_transient_local', False)

        # map과 camera_init의 TF가 identity임을 확인했으므로 같은 좌표로 취급한다.
        self.declare_parameter(
            'equivalent_frames',
            ['map', 'camera_init'],
        )

        self.declare_parameter('search_radius_m', 0.50)
        self.declare_parameter('minimum_points', 10)
        self.declare_parameter('ground_percentile', 20.0)
        self.declare_parameter('ground_band_below_m', 0.05)
        self.declare_parameter('ground_band_above_m', 0.10)

        gp = self.get_parameter
        self.map_cloud_topic = str(gp('map_cloud_topic').value)
        self.initialpose_topic = str(gp('initialpose_topic').value)
        self.goal_topic = str(gp('goal_topic').value)
        self.output_file = Path(gp('output_file').value).expanduser().resolve()
        self.cloud_transient_local = bool(gp('cloud_transient_local').value)
        self.equivalent_frames = {
            self.normalize_frame(frame)
            for frame in gp('equivalent_frames').value
            if str(frame).strip()
        }
        self.search_radius = float(gp('search_radius_m').value)
        self.minimum_points = int(gp('minimum_points').value)
        self.ground_percentile = float(gp('ground_percentile').value)
        self.band_below = float(gp('ground_band_below_m').value)
        self.band_above = float(gp('ground_band_above_m').value)

        self.cloud_xyz = None
        self.cloud_frame = ''
        self.lock = Lock()

        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if self.cloud_transient_local
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if self.cloud_transient_local
                else DurabilityPolicy.VOLATILE
            ),
        )

        self.create_subscription(
            PointCloud2,
            self.map_cloud_topic,
            self.cloud_callback,
            cloud_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.get_logger().info(f'Map cloud : {self.map_cloud_topic}')
        self.get_logger().info(f'Initial   : {self.initialpose_topic}')
        self.get_logger().info(f'Goal      : {self.goal_topic}')
        self.get_logger().info(f'Save file : {self.output_file}')
        self.get_logger().info(
            'Equivalent frames: ' + ', '.join(sorted(self.equivalent_frames))
        )

    @staticmethod
    def normalize_frame(frame_id: str) -> str:
        """'/map'과 'map'을 같은 프레임 이름으로 처리."""
        return str(frame_id).strip().lstrip('/')

    def frames_are_compatible(self, pose_frame: str, cloud_frame: str) -> bool:
        pose_frame = self.normalize_frame(pose_frame)
        cloud_frame = self.normalize_frame(cloud_frame)

        if not pose_frame or not cloud_frame:
            return True
        if pose_frame == cloud_frame:
            return True
        return (
            pose_frame in self.equivalent_frames
            and cloud_frame in self.equivalent_frames
        )

    @staticmethod
    def current_time() -> str:
        return datetime.now(ZoneInfo('Asia/Seoul')).isoformat(timespec='seconds')

    def cloud_callback(self, msg: PointCloud2):
        try:
            points = point_cloud2.read_points_numpy(
                msg,
                field_names=['x', 'y', 'z'],
                skip_nans=True,
            )
            xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        except Exception as exc:
            self.get_logger().error(f'PointCloud 변환 실패: {exc}')
            return

        if xyz.size == 0:
            self.get_logger().warning('수신한 PointCloud에 유효한 점이 없습니다.')
            return

        with self.lock:
            self.cloud_xyz = xyz
            self.cloud_frame = self.normalize_frame(msg.header.frame_id)

        self.get_logger().info(
            f'3D map 수신: {len(xyz):,} points, frame={self.cloud_frame}'
        )

    def estimate_ground_z(self, x: float, y: float, pose_frame: str):
        with self.lock:
            if self.cloud_xyz is None:
                self.get_logger().warning(
                    '아직 3D map PointCloud를 받지 못했습니다.'
                )
                return None

            xyz = self.cloud_xyz
            cloud_frame = self.cloud_frame

        pose_frame = self.normalize_frame(pose_frame)

        if not self.frames_are_compatible(pose_frame, cloud_frame):
            self.get_logger().error(
                f'좌표계 불일치: pose={pose_frame}, cloud={cloud_frame}. '
                f'동일 좌표로 허용된 프레임은 '
                f'{sorted(self.equivalent_frames)}입니다.'
            )
            return None

        if pose_frame != cloud_frame:
            self.get_logger().info(
                f'동일 좌표계로 처리: pose={pose_frame}, cloud={cloud_frame}'
            )

        dx = xyz[:, 0] - x
        dy = xyz[:, 1] - y
        mask = dx * dx + dy * dy <= self.search_radius * self.search_radius
        nearby_z = xyz[mask, 2].astype(np.float64)

        if nearby_z.size < self.minimum_points:
            self.get_logger().warning(
                f'({x:.2f}, {y:.2f}) 반경 {self.search_radius:.2f} m 안의 '
                f'포인트가 부족합니다: {nearby_z.size}/{self.minimum_points}'
            )
            return None

        low_reference = float(
            np.percentile(nearby_z, self.ground_percentile)
        )
        ground_mask = (
            (nearby_z >= low_reference - self.band_below)
            & (nearby_z <= low_reference + self.band_above)
        )
        ground_z_values = nearby_z[ground_mask]

        if ground_z_values.size == 0:
            self.get_logger().warning(
                f'({x:.2f}, {y:.2f}) 주변에서 지면 후보를 찾지 못했습니다.'
            )
            return None

        return (
            float(np.median(ground_z_values)),
            int(nearby_z.size),
            int(ground_z_values.size),
        )

    def load_records(self):
        if not self.output_file.exists():
            return {
                'created_at': self.current_time(),
                'initial_pose': [],
                'coordinate': [],
                'goal_pose': [],
            }

        try:
            with self.output_file.open('r', encoding='utf-8') as stream:
                data = json.load(stream)

            if not isinstance(data, dict):
                raise ValueError('JSON 최상위 데이터가 객체가 아닙니다.')

            data.setdefault('created_at', self.current_time())
            data.setdefault('initial_pose', [])
            data.setdefault('coordinate', [])
            data.setdefault('goal_pose', [])
            return data
        except Exception as exc:
            self.get_logger().error(f'기존 JSON 읽기 실패: {exc}')
            return None

    def save_record(self, category: str, pose, frame_id: str, covariance=None):
        result = self.estimate_ground_z(
            float(pose.position.x),
            float(pose.position.y),
            frame_id,
        )
        if result is None:
            self.get_logger().warning(
                '지면 Z를 찾지 못해 Pose를 저장하지 않았습니다.'
            )
            return

        ground_z, nearby_count, ground_count = result
        data = self.load_records()
        if data is None:
            return

        saved_frame = self.normalize_frame(frame_id) or self.cloud_frame
        record = {
            'position': {
                'x': round(float(pose.position.x), 4),
                'y': round(float(pose.position.y), 4),
                'z': round(float(ground_z), 4),
            },
            'orientation': {
                'x': float(pose.orientation.x),
                'y': float(pose.orientation.y),
                'z': float(pose.orientation.z),
                'w': float(pose.orientation.w),
            },
        }

        if covariance is not None:
            record['covariance'] = [float(value) for value in covariance]

        record['saved_at'] = self.current_time()
        record['frame_id'] = saved_frame
        data[category].append(record)

        try:
            self.output_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.output_file.with_suffix(
                self.output_file.suffix + '.tmp'
            )
            with temporary.open('w', encoding='utf-8') as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)
                stream.write('\n')
            os.replace(temporary, self.output_file)
        except Exception as exc:
            self.get_logger().error(f'JSON 저장 실패: {exc}')
            return

        label = 'Initial Pose' if category == 'initial_pose' else 'Goal Pose'
        self.get_logger().info(
            f'{label} 저장: '
            f'x={pose.position.x:.3f}, '
            f'y={pose.position.y:.3f}, '
            f'z={ground_z:.3f}, '
            f'nearby={nearby_count}, ground={ground_count}, '
            f'file={self.output_file}'
        )

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        self.save_record(
            category='initial_pose',
            pose=msg.pose.pose,
            frame_id=msg.header.frame_id,
            covariance=msg.pose.covariance,
        )

    def goal_callback(self, msg: PoseStamped):
        self.save_record(
            category='goal_pose',
            pose=msg.pose,
            frame_id=msg.header.frame_id,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseZWaypointSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
