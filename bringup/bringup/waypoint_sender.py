#!/usr/bin/env python3
"""Send a sequence of map-frame poses to Nav2's waypoint follower."""

import json
import math
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import FollowWaypoints

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


# Waypoints are (x [m], y [m], yaw [deg]).
# Replace these entries with the coordinates shown by RViz.
WAYPOINTS = [
    # (1.0, 2.0, 0.0),
    # (3.0, 2.0, 90.0),
]


class WaypointSender(Node):
    """Send the waypoints declared above once Nav2 is available."""

    def __init__(self):
        """Create the Nav2 action client."""
        super().__init__('waypoint_sender')
        self.declare_parameter('history_file', 'waypoint_history.json')
        history_file = self.get_parameter('history_file').value
        self._history_file = Path(history_file).expanduser().resolve()
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def save_history(self):
        """Append this run's time and coordinates to one JSON file."""
        record = {
            'saved_at': datetime.now().astimezone().isoformat(
                timespec='seconds'
            ),
            'frame_id': 'map',
            'waypoints': [
                {'x': x, 'y': y, 'yaw_deg': yaw_deg}
                for x, y, yaw_deg in WAYPOINTS
            ],
        }

        self._history_file.parent.mkdir(parents=True, exist_ok=True)
        history = []
        if self._history_file.exists():
            try:
                with self._history_file.open(encoding='utf-8') as file:
                    loaded = json.load(file)
                if isinstance(loaded, list):
                    history = loaded
                else:
                    raise ValueError('JSON root must be a list')
            except (json.JSONDecodeError, OSError, ValueError) as error:
                self.get_logger().error(
                    f'Cannot read history file: {error}'
                )
                return False

        history.append(record)
        temporary_file = self._history_file.with_suffix('.json.tmp')
        try:
            with temporary_file.open('w', encoding='utf-8') as file:
                json.dump(history, file, ensure_ascii=False, indent=2)
                file.write('\n')
            temporary_file.replace(self._history_file)
        except OSError as error:
            self.get_logger().error(f'Cannot save waypoint history: {error}')
            return False

        self.get_logger().info(f'Saved history to {self._history_file}')
        return True

    @staticmethod
    def make_pose(x, y, yaw_deg, stamp):
        """Convert an x, y, yaw tuple into a map-frame pose."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        yaw = math.radians(float(yaw_deg))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def send(self):
        """Wait for Nav2 and send every configured waypoint."""
        if not WAYPOINTS:
            self.get_logger().error(
                'WAYPOINTS is empty. Add (x, y, yaw_deg) coordinates first.'
            )
            return False

        self.get_logger().info('Waiting for Nav2 follow_waypoints action...')
        self._client.wait_for_server()

        stamp = self.get_clock().now().to_msg()
        goal = FollowWaypoints.Goal()
        goal.poses = [
            self.make_pose(x, y, yaw_deg, stamp)
            for x, y, yaw_deg in WAYPOINTS
        ]

        future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'Sending {len(goal.poses)} waypoint(s).')
        return True

    def goal_response_callback(self, future):
        """Handle Nav2 accepting or rejecting the waypoint goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal was rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Waypoint goal accepted.')
        self.save_history()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Log the index of the waypoint currently being followed."""
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(
            f'Moving to waypoint {current + 1}/{len(WAYPOINTS)}'
        )

    def result_callback(self, future):
        """Report completion and stop the node."""
        result = future.result().result
        missed = list(result.missed_waypoints)
        if missed:
            self.get_logger().warning(
                f'Finished; missed waypoint indices: {missed}'
            )
        else:
            self.get_logger().info('All waypoints completed.')
        rclpy.shutdown()


def main(args=None):
    """Run the waypoint sender node."""
    rclpy.init(args=args)
    node = WaypointSender()
    if node.send():
        rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
