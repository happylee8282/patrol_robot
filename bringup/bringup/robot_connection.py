#!/usr/bin/env python3
"""Poll the robot connection service and launch bringup when connected."""

import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class RobotConnection(Node):
    """Send true once per second until the server confirms connection."""

    def __init__(self):
        super().__init__('robot_connection')
        self._client = self.create_client(SetBool, '/robot/connection')
        self._request_pending = False
        self._launched = False
        self._launch_process = None
        self._timer = self.create_timer(1.0, self._poll)
        self.get_logger().info('Polling /robot/connection every second')

    def _poll(self):
        if self._launched or self._request_pending:
            return
        if not self._client.service_is_ready():
            self.get_logger().warning('/robot/connection is not available')
            return
        request = SetBool.Request()
        request.data = True
        self._request_pending = True
        future = self._client.call_async(request)
        future.add_done_callback(self._response_cb)

    def _response_cb(self, future):
        self._request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Connection service failed: {error}')
            return
        if not response.success:
            self.get_logger().info('Robot returned 0; retrying in one second')
            return
        self._launched = True
        self._timer.cancel()
        self.get_logger().info(
            'Robot returned 1; launching bringup_all.launch.py')
        try:
            self._launch_process = subprocess.Popen(
                ['ros2', 'launch', 'bringup', 'bringup_all.launch.py'])
        except OSError as error:
            self._launched = False
            self.get_logger().error(f'Could not launch bringup: {error}')

    def destroy_node(self):
        if self._launch_process and self._launch_process.poll() is None:
            self._launch_process.terminate()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotConnection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
