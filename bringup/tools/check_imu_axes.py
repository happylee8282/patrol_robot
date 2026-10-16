#!/usr/bin/env python3
"""Compare myAHRS and Livox gyro axes over a 30-second motion test."""

import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


AXES = ('x', 'y', 'z')
MOTIONS = ('롤: 좌우로 기울이기', '피치: 앞뒤로 기울이기', '요: 수평으로 돌리기')
SECONDS_PER_AXIS = 10
MIN_MOTION_STD = 0.03  # rad/s; ignore stationary gyro noise


def rates(message):
    return tuple(getattr(message.angular_velocity, axis) for axis in AXES)


def correlation(first, second):
    first_mean = statistics.mean(first)
    second_mean = statistics.mean(second)
    first_centered = [value - first_mean for value in first]
    second_centered = [value - second_mean for value in second]
    denominator = math.sqrt(
        sum(value * value for value in first_centered)
        * sum(value * value for value in second_centered)
    )
    if denominator == 0:
        return 0.0
    return sum(a * b for a, b in zip(first_centered, second_centered)) / denominator


def summarize(samples, expected_axis):
    if len(samples) < 30:
        return '데이터 부족: 두 IMU 토픽을 확인하세요'
    ahrs = [row[0][expected_axis] for row in samples]
    if statistics.pstdev(ahrs) < MIN_MOTION_STD:
        return '움직임 부족: 해당 축으로 더 크게 돌려 다시 측정하세요'
    scores = [
        correlation(ahrs, [row[1][axis] for row in samples])
        for axis in range(3)
    ]
    best_axis = max(range(3), key=lambda axis: abs(scores[axis]))
    score = scores[best_axis]
    if abs(score) < 0.65:
        result = '불확실: 한 축씩 움직여 다시 측정하세요'
    elif best_axis != expected_axis:
        result = '축 다름'
    elif score < 0:
        result = '부호 반대'
    else:
        result = '방향 일치'
    return (
        f'myAHRS {AXES[expected_axis]} → Livox {AXES[best_axis]}, '
        f'상관계수 {score:+.2f}: {result}'
    )


class ImuAxisCheck(Node):
    def __init__(self):
        super().__init__('check_imu_axes')
        self.latest = {}
        self.samples = [[], [], []]
        self.started_at = None
        self.phase = -1
        self.done = False
        self.create_subscription(
            Imu, '/imu/data',
            lambda msg: self.save('myAHRS', msg), qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, '/livox/imu',
            lambda msg: self.save('Livox', msg), qos_profile_sensor_data
        )
        self.create_timer(0.05, self.collect)

    def save(self, name, message):
        self.latest[name] = (time.monotonic(), rates(message))

    def collect(self):
        now = time.monotonic()
        ready = all(
            name in self.latest and now - self.latest[name][0] < 0.15
            for name in ('myAHRS', 'Livox')
        )
        if self.started_at is None:
            if not ready:
                return
            self.started_at = now
            print('두 IMU 연결됨. 30초 측정을 시작합니다.', flush=True)

        elapsed = now - self.started_at
        if elapsed >= 3 * SECONDS_PER_AXIS:
            self.done = True
            print('\n30초 분석 결과', flush=True)
            for axis in range(3):
                print(
                    f'{AXES[axis]}축 ({MOTIONS[axis]}): '
                    f'{summarize(self.samples[axis], axis)} '
                    f'[{len(self.samples[axis])}쌍]',
                    flush=True,
                )
            return

        phase = int(elapsed // SECONDS_PER_AXIS)
        if phase != self.phase:
            self.phase = phase
            print(f'지금부터 10초: {MOTIONS[phase]} ({AXES[phase]}축)', flush=True)
        if ready:
            self.samples[phase].append(
                (self.latest['myAHRS'][1], self.latest['Livox'][1])
            )


def main():
    rclpy.init()
    node = ImuAxisCheck()
    print('두 IMU 토픽을 기다리는 중...')
    print('연결되면 0~10초 롤, 10~20초 피치, 20~30초 요 방향으로 움직이세요.')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n측정이 중단됐습니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
