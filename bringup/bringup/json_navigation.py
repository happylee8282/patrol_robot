#!/usr/bin/env python3
"""Load recorded poses and execute indexed Nav2 routes."""

import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32


class JsonNavigation(Node):
    """Publish the first initial pose and navigate JSON goals in sequence."""

    def __init__(self):
        super().__init__('json_navigation')
        default_json_file = Path(
            '/home/unicon/nav_ws/src/bringup/config/point_map/final.json'
        )
        self.declare_parameter('json_file', str(default_json_file))
        json_file = self.get_parameter('json_file').value
        self._data = self._load_json(Path(json_file).expanduser())
        self._goals = self._data.get('goal_pose', [])
        if not self._goals:
            raise RuntimeError('JSON has no goal_pose entries')

        initial_qos = QoSProfile(depth=1)
        initial_qos.reliability = ReliabilityPolicy.RELIABLE
        initial_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', initial_qos)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._busy = False
        self._route = []
        # Logical waypoint number. Zero means the robot has not visited one yet.
        self._current_waypoint = 0
        self._active_goal_handle = None
        self._pending_waypoint = None
        self._retry_timer = None
        self._last_progress_log_ns = 0
        self._success_pub = self.create_publisher(
            Int32, '/robot_nav/goal_success', 10)
        self.create_subscription(Int32, '/robot_nav/goal', self._command_cb, 10)
        self._publish_initial_pose()
        self.get_logger().info(
            f'Loaded {len(self._goals)} goals; waiting on /robot_nav/goal')

    @staticmethod
    def _load_json(path):
        with path.open(encoding='utf-8') as stream:
            data = json.load(stream)
        if not isinstance(data, dict):
            raise RuntimeError('JSON root must be an object')
        return data

    @staticmethod
    def _fill_pose(target, source):
        position = source['position']
        orientation = source['orientation']
        target.position.x = float(position.get('x', 0.0))
        target.position.y = float(position.get('y', 0.0))
        target.position.z = float(position.get('z', 0.0))
        target.orientation.x = float(orientation.get('x', 0.0))
        target.orientation.y = float(orientation.get('y', 0.0))
        target.orientation.z = float(orientation.get('z', 0.0))
        target.orientation.w = float(orientation.get('w', 1.0))

    def _publish_initial_pose(self):
        poses = self._data.get('initial_pose', [])
        if not poses:
            self.get_logger().warning('JSON has no initial_pose entry')
            return
        source = poses[0]
        message = PoseWithCovarianceStamped()
        message.header.frame_id = source.get('frame_id', 'map')
        message.header.stamp = self.get_clock().now().to_msg()
        self._fill_pose(message.pose.pose, source)
        covariance = source.get('covariance', [])
        if len(covariance) == 36:
            message.pose.covariance = [float(value) for value in covariance]
        self._initial_pub.publish(message)
        self.get_logger().info('Published initial_pose #1')

    def _command_cb(self, message):
        requested = int(message.data)
        if requested < 1 or requested > len(self._goals):
            self.get_logger().error(
                f'Goal must be between 1 and {len(self._goals)}')
            return
        if self._busy:
            self._pending_waypoint = requested
            self._route = []
            self.get_logger().info(
                f'New waypoint {requested} received; stopping current goal')
            if self._active_goal_handle is not None:
                self._active_goal_handle.cancel_goal_async()
            return

        self._start_route(requested)

    def _start_route(self, requested):
        if requested == self._current_waypoint:
            self.get_logger().info(
                f'Robot is already at waypoint {requested}')
            return

        if requested > self._current_waypoint:
            waypoint_numbers = range(
                self._current_waypoint + 1, requested + 1)
        else:
            waypoint_numbers = range(
                self._current_waypoint - 1, requested - 1, -1)

        # JSON goal_pose uses zero-based list indices internally.
        self._route = [number - 1 for number in waypoint_numbers]
        self._busy = True
        self.get_logger().info(
            'Starting route: ' + ', '.join(str(i + 1) for i in self._route))
        self._send_next()

    def _send_next(self):
        if not self._route:
            self._busy = False
            if self._start_pending_route():
                return
            self.get_logger().info('Route complete; waiting for next command')
            return
        if not self._nav_client.server_is_ready():
            self.get_logger().info('Waiting for navigate_to_pose action...')
            self._nav_client.wait_for_server(timeout_sec=1.0)
            if not self._nav_client.server_is_ready():
                if self._retry_timer is None:
                    self._retry_timer = self.create_timer(1.0, self._retry_once)
                return
        index = self._route.pop(0)
        source = self._goals[index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = source.get('frame_id', 'map')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self._fill_pose(goal.pose.pose, source)
        self._current_index = index
        self._active_goal_handle = None
        self._last_progress_log_ns = 0
        self.get_logger().info(f'{index + 1} waypoint moving')
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_progress_log_ns < 2_000_000_000:
            return
        self._last_progress_log_ns = now_ns
        feedback = feedback_message.feedback
        self.get_logger().info(
            f'{self._current_index + 1} waypoint '
            f'{feedback.distance_remaining:.2f} m remaining')

    def _retry_once(self):
        self._retry_timer.cancel()
        self.destroy_timer(self._retry_timer)
        self._retry_timer = None
        self._send_next()

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'Goal {self._current_index + 1} rejected')
            self._busy = False
            self._route = []
            self._start_pending_route()
            return
        self._active_goal_handle = handle
        self.get_logger().info(f'{self._current_index + 1} waypoint accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)
        if self._pending_waypoint is not None:
            handle.cancel_goal_async()

    def _start_pending_route(self):
        if self._pending_waypoint is None:
            return False
        requested = self._pending_waypoint
        self._pending_waypoint = None
        self._busy = False
        self._start_route(requested)
        return True

    def _result_cb(self, future):
        status = future.result().status
        self._active_goal_handle = None
        if self._pending_waypoint is not None:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._waypoint_arrived()
            else:
                self.get_logger().info('Current goal stopped')
            self._start_pending_route()
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Goal {self._current_index + 1} failed (status={status})')
            self._busy = False
            self._route = []
            return
        self._waypoint_arrived()
        self._send_next()

    def _waypoint_arrived(self):
        self._current_waypoint = self._current_index + 1
        self.get_logger().info(f'{self._current_index + 1} waypoint arrived')
        message = Int32()
        message.data = self._current_waypoint
        self._success_pub.publish(message)
        self.get_logger().info(
            f'Published {message.data} on /robot_nav/goal_success')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JsonNavigation()
    except Exception as error:
        rclpy.logging.get_logger('json_navigation').fatal(str(error))
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
