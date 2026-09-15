#!/usr/bin/env python3
"""Record RViz initial poses, clicked points, and Nav2 goals to a new JSON."""

import json
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class PoseCoordinateRecorder(Node):
    """Persist RViz navigation inputs as they arrive."""

    def __init__(self):
        super().__init__('pose_coordinate_recorder')
        default_output_dir = (
            Path(__file__).resolve().parents[1] / 'config' / 'point_map'
        )
        self.declare_parameter('output_directory', str(default_output_dir))
        output_dir = Path(self.get_parameter('output_directory').value).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        self.output_path = output_dir / f'navigation_coordinates_{timestamp}.json'
        self.data = {
            'created_at': datetime.now().astimezone().isoformat(timespec='seconds'),
            'initial_pose': [],
            'coordinate': [],
            'goal_pose': [],
        }
        self._write_json()

        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initial_pose_cb, 10)
        self.create_subscription(
            PointStamped, '/clicked_point', self._coordinate_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_cb, 10)
        self.get_logger().info(f'Recording coordinates to {self.output_path}')

    @staticmethod
    def _time_now():
        return datetime.now().astimezone().isoformat(timespec='milliseconds')

    @staticmethod
    def _pose_dict(message):
        return {
            'position': {
                'x': message.position.x,
                'y': message.position.y,
                'z': message.position.z,
            },
            'orientation': {
                'x': message.orientation.x,
                'y': message.orientation.y,
                'z': message.orientation.z,
                'w': message.orientation.w,
            },
        }

    def _append(self, section, frame_id, value):
        value['saved_at'] = self._time_now()
        value['frame_id'] = frame_id or 'map'
        self.data[section].append(value)
        try:
            self._write_json()
        except OSError as error:
            self.get_logger().error(f'Failed to write JSON: {error}')
            return
        self.get_logger().info(
            f'Saved {section} #{len(self.data[section])}')

    def _write_json(self):
        temporary = self.output_path.with_suffix('.json.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(self.data, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        temporary.replace(self.output_path)

    def _initial_pose_cb(self, message):
        value = self._pose_dict(message.pose.pose)
        value['covariance'] = list(message.pose.covariance)
        self._append('initial_pose', message.header.frame_id, value)

    def _coordinate_cb(self, message):
        self._append('coordinate', message.header.frame_id, {
            'x': message.point.x,
            'y': message.point.y,
            'z': message.point.z,
        })

    def _goal_pose_cb(self, message):
        self._append(
            'goal_pose', message.header.frame_id, self._pose_dict(message.pose))


def main(args=None):
    rclpy.init(args=args)
    node = PoseCoordinateRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
