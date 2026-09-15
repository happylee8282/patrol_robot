#!/usr/bin/env python3
"""Loop or stop an audio file from /server/speaker Int32 commands."""

from pathlib import Path
import os
import shutil
import subprocess

# CoastalPatrol robot-wide ROS 2 DDS domain. This must be set before rclpy is
# imported/initialized so the speaker always joins the gateway's domain.
os.environ['ROS_DOMAIN_ID'] = '84'

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32


class RobotSpeaker(Node):
    """Play on command 1 and stop on command 0."""

    def __init__(self):
        super().__init__('robot_speaker')
        default_audio_file = (
            Path(get_package_share_directory('bringup'))
            / 'tools'
            / 'screaming.mp3'
        )
        self.declare_parameter('audio_file', str(default_audio_file))
        self._audio_file = Path(
            self.get_parameter('audio_file').value).expanduser()
        self._process = None
        speaker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscription = self.create_subscription(
            Int32,
            '/server/speaker',
            self._command_cb,
            speaker_qos,
        )
        self.get_logger().info('Using ROS_DOMAIN_ID=84')
        self.get_logger().info('Waiting for 0/1 commands on /server/speaker')

    def _command_cb(self, message):
        if message.data == 1:
            self._play()
        elif message.data == 0:
            self._stop()
        else:
            self.get_logger().warning('Speaker command must be 0 or 1')

    def _play(self):
        if self._process and self._process.poll() is None:
            return
        if not self._audio_file.is_file():
            self.get_logger().error(
                f'Audio file does not exist: {self._audio_file}')
            return
        if shutil.which('ffplay'):
            command = [
                'ffplay', '-nodisp', '-loglevel', 'error', '-loop', '0',
                str(self._audio_file),
            ]
        elif shutil.which('mpg123'):
            command = ['mpg123', '--loop', '-1', str(self._audio_file)]
        elif shutil.which('aplay') and self._audio_file.suffix.lower() == '.wav':
            command = ['aplay', '--quiet', '--loop=0', str(self._audio_file)]
        else:
            self.get_logger().error(
                'Install ffplay/mpg123, or use a WAV file with aplay')
            return
        try:
            self._process = subprocess.Popen(command)
            self.get_logger().info('Speaker playback started')
        except OSError as error:
            self.get_logger().error(f'Could not start player: {error}')

    def _stop(self):
        if self._process and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self.get_logger().info('Speaker playback stopped')
        self._process = None

    def destroy_node(self):
        self._stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSpeaker()
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
