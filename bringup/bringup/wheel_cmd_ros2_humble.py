#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# math는 목표 방향, yaw 오차, quaternion 변환 계산에 사용합니다.
import math

# rclpy는 ROS2 Python 클라이언트 라이브러리입니다.
import rclpy

# Node는 ROS2 노드 클래스를 만들기 위한 기본 클래스입니다.
from rclpy.node import Node

# ROS2 제어 토픽에 사용할 QoS 정책을 가져옵니다.
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# 기존 코드와 동일하게 상태와 휠 명령은 Int16 계열 메시지를 사용합니다.
from std_msgs.msg import Int16, Int16MultiArray

# Nav2 속도 명령과 목표/현재 자세 메시지를 사용합니다.
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

# UART 프로토콜에서 사용하는 방향 명령값입니다.
DIR_FORWARD = 0x43       # 67, ASCII 'C': 전진
DIR_BACKWARD = 0x57      # 87, ASCII 'W': 후진
DIR_STOP = 0x53          # 83, ASCII 'S': 정지

# UART 프로토콜에서 정지 속도를 의미하는 기준값입니다.
SPEED_STOP = 0x21        # 33, ASCII '!'

# UART 상태 프레임에서 Auto 모드를 의미하는 값입니다.
MODE_AUTO = 0x41         # 65, ASCII 'A'

# UART 상태 프레임에서 Manual 모드를 의미하는 값입니다.
MODE_MANUAL = 0x4D       # 77, ASCII 'M'


# WheelNavCommandNode는 Nav2의 cmd_vel을 UART용 좌우 휠 명령으로 변환합니다.
class WheelNavCommandNode(Node):

    # 생성자에서 파라미터, Publisher, Subscriber, Timer, 내부 상태를 초기화합니다.
    def __init__(self) -> None:

        # ROS2 노드 이름을 기존 코드와 동일한 'wheel_cmd'로 설정합니다.
        super().__init__('wheel_cmd')

        # 좌우 휠 중심 사이의 거리를 m 단위 파라미터로 선언합니다.
        self.declare_parameter('wheel_separation', 0.54)

        # 원본 코드의 휠 속도 변환 계수 3.6을 파라미터로 선언합니다.
        # 원본 주석은 1/radius라고 되어 있으나 실제 단위와 의미는 모터 사양 확인이 필요합니다.
        self.declare_parameter('wheel_speed_coefficient', 3.6)

        # UART로 전달할 브레이크 명령값을 파라미터로 선언합니다.
        self.declare_parameter('brake_command', 79)

        # 휠 속도 명령의 최대값을 UART 1바이트 범위인 255로 제한합니다.
        self.declare_parameter('maximum_speed_command', 255)

        # 새 목표를 받을 때 초기 제자리 회전을 사용할지 설정합니다.
        self.declare_parameter('start_turn_enabled', False)

        # 초기 제자리 회전 시 좌우 휠에 전달할 속도 명령값입니다.
        self.declare_parameter('start_turn_speed_command', 40)

        # 초기 제자리 회전을 종료할 yaw 오차 허용값입니다.
        # 원본 동작을 보존하기 위해 0.5 rad를 기본값으로 사용합니다.
        self.declare_parameter('start_turn_tolerance_rad', 0.5)

        # 새 목표 또는 장애물 해제 후 부드러운 출발 기능을 사용할지 설정합니다.
        self.declare_parameter('smooth_start_enabled', True)

        # 부드러운 출발의 최초 속도 비율을 설정합니다.
        self.declare_parameter('smooth_start_initial_factor', 0.5)

        # 최초 속도 비율에서 100%까지 증가하는 콜백 횟수를 설정합니다.
        self.declare_parameter('smooth_start_steps', 50)

        # 장애물 검출 중 현재 속도에서 줄일 비율을 설정합니다.
        self.declare_parameter('obstacle_speed_ratio', 0.9)

        # 장애물 감속 중 이 값 이하가 되면 완전 정지시킵니다.
        self.declare_parameter('obstacle_stop_threshold', 35)

        # cmd_vel이 이 시간 이상 들어오지 않으면 정지 명령을 발행합니다.
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)

        # cmd_vel watchdog 검사 주기를 설정합니다.
        self.declare_parameter('watchdog_period_sec', 0.1)

        # ROS2/Nav2에서 목표 Pose를 받을 토픽 이름을 설정합니다.
        self.declare_parameter('goal_topic', 'goal_pose')

        # wheel_separation 파라미터값을 읽어 실수형으로 저장합니다.
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value
        )

        # 원본의 속도 변환 계수를 읽어 저장합니다.
        self.wheel_speed_coefficient = float(
            self.get_parameter('wheel_speed_coefficient').value
        )

        # 브레이크 명령값을 정수형으로 읽습니다.
        self.brake_command = int(
            self.get_parameter('brake_command').value
        )

        # 최대 속도 명령값을 정수형으로 읽습니다.
        self.maximum_speed_command = max(
            SPEED_STOP,
            min(255, int(self.get_parameter('maximum_speed_command').value)),
        )

        # 초기 제자리 회전 사용 여부를 읽습니다.
        self.start_turn_enabled = bool(
            self.get_parameter('start_turn_enabled').value
        )

        # 초기 제자리 회전 속도값을 읽습니다.
        self.start_turn_speed_command = max(
            SPEED_STOP,
            min(
                self.maximum_speed_command,
                int(self.get_parameter('start_turn_speed_command').value),
            ),
        )

        # 초기 회전 종료 yaw 오차값을 읽습니다.
        self.start_turn_tolerance_rad = float(
            self.get_parameter('start_turn_tolerance_rad').value
        )

        # 부드러운 출발 사용 여부를 읽습니다.
        self.smooth_start_enabled = bool(
            self.get_parameter('smooth_start_enabled').value
        )

        # 부드러운 출발의 초기 비율을 읽습니다.
        self.smooth_start_initial_factor = float(
            self.get_parameter('smooth_start_initial_factor').value
        )

        # 부드러운 출발 단계 수를 읽습니다.
        self.smooth_start_steps = max(
            1,
            int(self.get_parameter('smooth_start_steps').value),
        )

        # 장애물 감속 비율을 읽고 0.0~1.0 범위로 제한합니다.
        self.obstacle_speed_ratio = min(
            1.0,
            max(0.0, float(self.get_parameter('obstacle_speed_ratio').value)),
        )

        # 장애물 완전 정지 기준값을 읽습니다.
        self.obstacle_stop_threshold = int(
            self.get_parameter('obstacle_stop_threshold').value
        )

        # cmd_vel timeout 값을 읽습니다.
        self.cmd_vel_timeout_sec = max(
            0.0,
            float(self.get_parameter('cmd_vel_timeout_sec').value),
        )

        # watchdog Timer 주기를 읽고 지나치게 작은 값은 제한합니다.
        self.watchdog_period_sec = max(
            0.01,
            float(self.get_parameter('watchdog_period_sec').value),
        )

        # 목표 Pose 토픽 이름을 읽습니다.
        self.goal_topic = str(
            self.get_parameter('goal_topic').value
        )

        # 최신 제어 명령만 유지하도록 depth=1인 Reliable QoS를 생성합니다.
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # UART 노드로 보낼 좌/우 휠 명령 Publisher를 생성합니다.
        self.navcmd_pub = self.create_publisher(
            Int16MultiArray,
            'wheel_cmd',
            control_qos,
        )

        # 현재 초기 회전 수행 여부를 알리는 Publisher를 생성합니다.
        self.turn_status_pub = self.create_publisher(
            Int16,
            'turn_status',
            control_qos,
        )

        # Nav2 Controller가 발행하는 선속도/각속도 명령을 구독합니다.
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_callback,
            control_qos,
        )

        # UART 노드에서 수신한 모드와 현재 휠 상태를 구독합니다.
        self.wheel_status_sub = self.create_subscription(
            Int16MultiArray,
            'wheel_status',
            self.uart_callback,
            control_qos,
        )

        # ROS2/Nav2의 목표 위치를 구독합니다.
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            control_qos,
        )

        # AMCL이 발행하는 현재 위치와 방향을 구독합니다.
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            control_qos,
        )

        # 별도 장애물 검출 노드의 상태를 구독합니다.
        self.obstacle_sub = self.create_subscription(
            Int16,
            'laser_obstacle',
            self.obstacle_callback,
            control_qos,
        )

        # cmd_vel 단절을 감시하는 주기 Timer를 생성합니다.
        self.watchdog_timer = self.create_timer(
            self.watchdog_period_sec,
            self.watchdog_callback,
        )

        # UART 상태를 받기 전에는 안전하게 Manual 상태로 간주합니다.
        self.manual_mode = True

        # 초기 장애물 상태를 미검출로 설정합니다.
        self.obstacle_detected = False

        # 초기 제자리 회전 상태를 비활성화합니다.
        self.start_turn_active = False

        # 부드러운 출발 상태를 비활성화합니다.
        self.smooth_start_active = False

        # 부드러운 출발 진행 횟수를 0으로 초기화합니다.
        self.smooth_start_step = 0

        # 목표 위치 x 좌표를 초기화합니다.
        self.goal_x = 0.0

        # 목표 위치 y 좌표를 초기화합니다.
        self.goal_y = 0.0

        # 현재 위치 x 좌표를 초기화합니다.
        self.pose_x = 0.0

        # 현재 위치 y 좌표를 초기화합니다.
        self.pose_y = 0.0

        # 현재 yaw 값을 초기화합니다.
        self.pose_yaw = 0.0

        # 현재 위치에서 목표까지의 직선거리를 초기화합니다.
        self.goal_distance = 0.0

        # 목표를 바라보는 기준 yaw를 초기화합니다.
        self.goal_yaw_reference = 0.0

        # 목표 yaw와 현재 yaw의 오차를 초기화합니다.
        self.goal_yaw_error = 0.0

        # 현재 좌측 휠 방향을 정지로 초기화합니다.
        self.current_left_direction = DIR_STOP

        # 현재 좌측 휠 속도를 정지 기준값으로 초기화합니다.
        self.current_left_speed = SPEED_STOP

        # 현재 우측 휠 방향을 정지로 초기화합니다.
        self.current_right_direction = DIR_STOP

        # 현재 우측 휠 속도를 정지 기준값으로 초기화합니다.
        self.current_right_speed = SPEED_STOP

        # 이전 좌측 방향을 저장하여 전진/후진 즉시 반전을 방지합니다.
        self.previous_left_direction = DIR_STOP

        # 이전 우측 방향을 저장하여 전진/후진 즉시 반전을 방지합니다.
        self.previous_right_direction = DIR_STOP

        # 마지막 cmd_vel 수신 시각을 ROS2 Clock 기준으로 저장합니다.
        self.last_cmd_vel_time = self.get_clock().now()

        # 같은 timeout 구간에서 정지 명령을 반복 발행하지 않도록 설정합니다.
        self.watchdog_stop_sent = True

        # 노드 설정값을 시작 로그로 출력합니다.
        self.get_logger().info(
            'wheel_cmd node started: '
            f'wheel_separation={self.wheel_separation}, '
            f'speed_coefficient={self.wheel_speed_coefficient}, '
            f'goal_topic={self.goal_topic}'
        )

    # angle을 -pi 이상 pi 이하 범위로 정규화합니다.
    @staticmethod
    def normalize_angle(angle: float) -> float:

        # atan2(sin, cos)를 사용하면 각도 차이가 항상 최단 회전 방향으로 표현됩니다.
        return math.atan2(math.sin(angle), math.cos(angle))

    # geometry_msgs Quaternion을 yaw 회전각으로 변환합니다.
    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:

        # Quaternion에서 yaw를 계산할 때 사용하는 분자를 계산합니다.
        sin_yaw = 2.0 * (w * z + x * y)

        # Quaternion에서 yaw를 계산할 때 사용하는 분모를 계산합니다.
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)

        # atan2를 사용하여 -pi~pi 범위의 yaw를 반환합니다.
        return math.atan2(sin_yaw, cos_yaw)

    # 정수값을 UART 1바이트 범위로 제한합니다.
    @staticmethod
    def clamp_byte(value: int) -> int:

        # 0보다 작으면 0, 255보다 크면 255로 제한합니다.
        return max(0, min(255, int(value)))

    # 현재 위치와 목표 위치로 거리와 목표 방향 오차를 계산합니다.
    def calculate_error(self) -> None:

        # 현재 위치에서 목표 위치까지의 x축 차이를 계산합니다.
        delta_x = self.goal_x - self.pose_x

        # 현재 위치에서 목표 위치까지의 y축 차이를 계산합니다.
        delta_y = self.goal_y - self.pose_y

        # 피타고라스 거리로 목표까지의 직선거리를 계산합니다.
        self.goal_distance = math.hypot(delta_x, delta_y)

        # atan2로 현재 위치에서 목표 위치를 바라보는 yaw를 계산합니다.
        self.goal_yaw_reference = math.atan2(delta_y, delta_x)

        # 목표 yaw에서 현재 yaw를 빼고 -pi~pi 범위로 정규화합니다.
        self.goal_yaw_error = self.normalize_angle(
            self.goal_yaw_reference - self.pose_yaw
        )

    # Nav2의 선속도/각속도를 좌우 휠 방향과 속도 명령으로 변환합니다.
    def calculate_velocity(self, msg: Twist) -> tuple[int, int, int, int]:

        # Nav2가 요청한 전진 방향 선속도를 읽습니다.
        linear_x = float(msg.linear.x)

        # Nav2가 요청한 z축 회전 각속도를 읽습니다.
        angular_z = float(msg.angular.z)

        # 차동구동 운동학으로 좌측 휠 선속도를 계산하고 기존 3.6 계수를 적용합니다.
        left_wheel_speed = (
            linear_x - angular_z * self.wheel_separation / 2.0
        ) * self.wheel_speed_coefficient

        # 차동구동 운동학으로 우측 휠 선속도를 계산하고 기존 3.6 계수를 적용합니다.
        right_wheel_speed = (
            linear_x + angular_z * self.wheel_separation / 2.0
        ) * self.wheel_speed_coefficient

        # 좌측 휠 속도의 부호를 UART 방향 명령으로 변환합니다.
        left_direction = self.direction_from_speed(left_wheel_speed)

        # 우측 휠 속도의 부호를 UART 방향 명령으로 변환합니다.
        right_direction = self.direction_from_speed(right_wheel_speed)

        # 좌측 속도의 절댓값을 기존 프로토콜의 10배 스케일과 33 오프셋으로 변환합니다.
        left_speed_command = SPEED_STOP + int(abs(left_wheel_speed) * 10.0)

        # 우측 속도의 절댓값을 기존 프로토콜의 10배 스케일과 33 오프셋으로 변환합니다.
        right_speed_command = SPEED_STOP + int(abs(right_wheel_speed) * 10.0)

        # 정지 방향이면 속도값도 정확히 33으로 설정합니다.
        if left_direction == DIR_STOP:
            left_speed_command = SPEED_STOP

        # 정지 방향이면 속도값도 정확히 33으로 설정합니다.
        if right_direction == DIR_STOP:
            right_speed_command = SPEED_STOP

        # 최대 속도 명령 범위를 초과하지 않도록 좌측 속도를 제한합니다.
        left_speed_command = min(
            self.maximum_speed_command,
            max(SPEED_STOP, left_speed_command),
        )

        # 최대 속도 명령 범위를 초과하지 않도록 우측 속도를 제한합니다.
        right_speed_command = min(
            self.maximum_speed_command,
            max(SPEED_STOP, right_speed_command),
        )

        # 좌측 방향, 좌측 속도, 우측 방향, 우측 속도를 반환합니다.
        return (
            left_direction,
            left_speed_command,
            right_direction,
            right_speed_command,
        )

    # 부호가 있는 휠 속도를 UART 방향 명령으로 변환합니다.
    @staticmethod
    def direction_from_speed(speed: float) -> int:

        # 양수는 전진 명령을 반환합니다.
        if speed > 0.0:
            return DIR_FORWARD

        # 음수는 후진 명령을 반환합니다.
        if speed < 0.0:
            return DIR_BACKWARD

        # 0은 정지 명령을 반환합니다.
        return DIR_STOP

    # 장애물 검출 중 현재 휠 상태를 기준으로 점진적으로 감속합니다.
    def obstacle_deceleration(self) -> tuple[int, int, int, int]:

        # 현재 좌측 속도에서 정지 오프셋 33을 제외한 유효 속도량을 구합니다.
        left_effective_speed = max(
            0,
            self.current_left_speed - SPEED_STOP,
        )

        # 현재 우측 속도에서 정지 오프셋 33을 제외한 유효 속도량을 구합니다.
        right_effective_speed = max(
            0,
            self.current_right_speed - SPEED_STOP,
        )

        # 좌측 유효 속도량에 장애물 감속 비율을 적용합니다.
        left_speed = SPEED_STOP + int(
            left_effective_speed * self.obstacle_speed_ratio
        )

        # 우측 유효 속도량에 장애물 감속 비율을 적용합니다.
        right_speed = SPEED_STOP + int(
            right_effective_speed * self.obstacle_speed_ratio
        )

        # 기본 방향은 UART 상태에서 수신한 현재 좌측 방향을 유지합니다.
        left_direction = self.current_left_direction

        # 기본 방향은 UART 상태에서 수신한 현재 우측 방향을 유지합니다.
        right_direction = self.current_right_direction

        # 좌측 감속값이 정지 기준 이하이면 완전 정지로 변경합니다.
        if left_speed < self.obstacle_stop_threshold:
            left_speed = SPEED_STOP
            left_direction = DIR_STOP

        # 우측 감속값이 정지 기준 이하이면 완전 정지로 변경합니다.
        if right_speed < self.obstacle_stop_threshold:
            right_speed = SPEED_STOP
            right_direction = DIR_STOP

        # 계산된 장애물 감속 명령을 반환합니다.
        return left_direction, left_speed, right_direction, right_speed

    # 새 목표를 받은 직후 목표 방향으로 제자리 회전할 명령을 계산합니다.
    def start_turn_command(self) -> tuple[int, int, int, int]:

        # 현재 위치와 목표 위치를 기준으로 yaw 오차를 갱신합니다.
        self.calculate_error()

        # yaw 오차가 허용값 이내이면 초기 회전을 종료하고 정지 명령을 반환합니다.
        if abs(self.goal_yaw_error) <= self.start_turn_tolerance_rad:
            self.start_turn_active = False
            self.get_logger().info('Initial turn completed')
            return DIR_STOP, SPEED_STOP, DIR_STOP, SPEED_STOP

        # 양의 yaw 오차는 반시계 방향 회전이므로 좌측 후진, 우측 전진을 사용합니다.
        if self.goal_yaw_error > 0.0:
            return (
                DIR_BACKWARD,
                self.start_turn_speed_command,
                DIR_FORWARD,
                self.start_turn_speed_command,
            )

        # 음의 yaw 오차는 시계 방향 회전이므로 좌측 전진, 우측 후진을 사용합니다.
        return (
            DIR_FORWARD,
            self.start_turn_speed_command,
            DIR_BACKWARD,
            self.start_turn_speed_command,
        )

    # 새 목표 또는 장애물 해제 후 휠 속도를 단계적으로 증가시킵니다.
    def apply_smooth_start(self, left_speed: int, right_speed: int) -> tuple[int, int]:

        # 부드러운 출발 기능이 비활성화되어 있으면 입력 속도를 그대로 반환합니다.
        if not self.smooth_start_enabled or not self.smooth_start_active:
            return left_speed, right_speed

        # 현재 부드러운 출발 단계 수를 1 증가시킵니다.
        self.smooth_start_step += 1

        # 전체 단계에서 현재 진행 비율을 0.0~1.0 범위로 계산합니다.
        progress = min(
            1.0,
            self.smooth_start_step / float(self.smooth_start_steps),
        )

        # 초기 비율에서 1.0까지 선형으로 증가하는 배율을 계산합니다.
        factor = self.smooth_start_initial_factor + (
            1.0 - self.smooth_start_initial_factor
        ) * progress

        # 좌측 속도에서 33 오프셋을 제외한 실제 속도량에만 배율을 적용합니다.
        left_speed = SPEED_STOP + int(
            max(0, left_speed - SPEED_STOP) * factor
        )

        # 우측 속도에서 33 오프셋을 제외한 실제 속도량에만 배율을 적용합니다.
        right_speed = SPEED_STOP + int(
            max(0, right_speed - SPEED_STOP) * factor
        )

        # 마지막 단계에 도달하면 부드러운 출발 상태를 종료합니다.
        if progress >= 1.0:
            self.smooth_start_active = False
            self.smooth_start_step = 0

        # 배율이 적용된 좌우 속도를 반환합니다.
        return left_speed, right_speed

    # 전진에서 후진 또는 후진에서 전진으로 즉시 바뀔 때 한 번 정지시킵니다.
    def apply_direction_change_guard(
        self,
        left_direction: int,
        left_speed: int,
        right_direction: int,
        right_speed: int,
    ) -> tuple[int, int, int, int]:

        # 좌측 휠이 전진/후진 사이에서 직접 반전되는지 확인합니다.
        left_reversing = (
            self.previous_left_direction in (DIR_FORWARD, DIR_BACKWARD)
            and left_direction in (DIR_FORWARD, DIR_BACKWARD)
            and self.previous_left_direction != left_direction
        )

        # 우측 휠이 전진/후진 사이에서 직접 반전되는지 확인합니다.
        right_reversing = (
            self.previous_right_direction in (DIR_FORWARD, DIR_BACKWARD)
            and right_direction in (DIR_FORWARD, DIR_BACKWARD)
            and self.previous_right_direction != right_direction
        )

        # 좌측 휠 반전이 감지되면 이번 명령에서는 정지시킵니다.
        output_left_direction = DIR_STOP if left_reversing else left_direction

        # 좌측 휠 반전이 감지되면 이번 명령의 속도를 33으로 설정합니다.
        output_left_speed = SPEED_STOP if left_reversing else left_speed

        # 우측 휠 반전이 감지되면 이번 명령에서는 정지시킵니다.
        output_right_direction = DIR_STOP if right_reversing else right_direction

        # 우측 휠 반전이 감지되면 이번 명령의 속도를 33으로 설정합니다.
        output_right_speed = SPEED_STOP if right_reversing else right_speed

        # 다음 콜백의 반전 판단을 위해 요청된 좌측 방향을 저장합니다.
        self.previous_left_direction = left_direction

        # 다음 콜백의 반전 판단을 위해 요청된 우측 방향을 저장합니다.
        self.previous_right_direction = right_direction

        # 반전 보호가 적용된 명령을 반환합니다.
        return (
            output_left_direction,
            output_left_speed,
            output_right_direction,
            output_right_speed,
        )

    # turn_status 토픽으로 초기 회전 상태를 발행합니다.
    def publish_turn_status(self, active: bool) -> None:

        # Int16 메시지를 생성합니다.
        msg = Int16()

        # 회전 중이면 1, 아니면 0을 저장합니다.
        msg.data = 1 if active else 0

        # turn_status 토픽으로 발행합니다.
        self.turn_status_pub.publish(msg)

    # 좌우 휠 명령을 Int16MultiArray로 구성하여 UART 노드에 발행합니다.
    def publish_wheel_command(
        self,
        left_direction: int,
        left_speed: int,
        right_direction: int,
        right_speed: int,
    ) -> None:

        # 모든 값을 UART 1바이트 범위로 제한합니다.
        command = [
            self.clamp_byte(left_direction),
            self.clamp_byte(left_speed),
            self.clamp_byte(right_direction),
            self.clamp_byte(right_speed),
            self.clamp_byte(self.brake_command),
        ]

        # Int16MultiArray 메시지를 생성합니다.
        msg = Int16MultiArray()

        # [좌방향, 좌속도, 우방향, 우속도, 브레이크] 순서로 저장합니다.
        msg.data = command

        # wheel_cmd 토픽으로 발행합니다.
        self.navcmd_pub.publish(msg)

    # 좌우 휠 모두에 정지 명령을 발행합니다.
    def publish_stop(self) -> None:

        # 정지 명령을 wheel_cmd 토픽으로 발행합니다.
        self.publish_wheel_command(
            DIR_STOP,
            SPEED_STOP,
            DIR_STOP,
            SPEED_STOP,
        )

        # 방향 전환 상태도 정지로 초기화합니다.
        self.previous_left_direction = DIR_STOP
        self.previous_right_direction = DIR_STOP

        # 초기 회전 상태는 0으로 발행합니다.
        self.publish_turn_status(False)

    # Nav2의 cmd_vel을 받아 실제 휠 명령을 계산하는 핵심 콜백입니다.
    def vel_callback(self, msg: Twist) -> None:

        # 마지막 cmd_vel 수신 시각을 갱신합니다.
        self.last_cmd_vel_time = self.get_clock().now()

        # watchdog에서 새 명령 구간을 감시할 수 있도록 정지 발행 상태를 해제합니다.
        self.watchdog_stop_sent = False

        # Manual 모드에서는 Navigation 명령을 UART로 보내지 않습니다.
        if self.manual_mode:
            return

        # 장애물이 검출된 상태이면 현재 UART 휠 상태를 기준으로 감속합니다.
        if self.obstacle_detected:
            left_direction, left_speed, right_direction, right_speed = (
                self.obstacle_deceleration()
            )
            self.publish_turn_status(False)

        # 초기 제자리 회전 상태이면 Nav2 cmd_vel보다 초기 회전 명령을 우선합니다.
        elif self.start_turn_active:
            left_direction, left_speed, right_direction, right_speed = (
                self.start_turn_command()
            )
            self.publish_turn_status(self.start_turn_active)

        # 장애물과 초기 회전이 없으면 일반 차동구동 속도 변환을 수행합니다.
        else:
            left_direction, left_speed, right_direction, right_speed = (
                self.calculate_velocity(msg)
            )
            self.publish_turn_status(False)

            # 일반 주행 명령에 부드러운 출발 속도 배율을 적용합니다.
            left_speed, right_speed = self.apply_smooth_start(
                left_speed,
                right_speed,
            )

        # 전진/후진 즉시 반전을 방지하는 보호 로직을 적용합니다.
        left_direction, left_speed, right_direction, right_speed = (
            self.apply_direction_change_guard(
                left_direction,
                left_speed,
                right_direction,
                right_speed,
            )
        )

        # 최종 좌우 휠 명령을 UART 노드로 발행합니다.
        self.publish_wheel_command(
            left_direction,
            left_speed,
            right_direction,
            right_speed,
        )

    # UART 노드에서 발행한 모드와 현재 휠 상태를 처리합니다.
    def uart_callback(self, msg: Int16MultiArray) -> None:

        # 원본 인덱스 0~5를 사용하려면 최소 6개의 데이터가 필요합니다.
        if len(msg.data) < 6:
            self.get_logger().warning(
                'Invalid wheel_status: '
                f'length={len(msg.data)}, data={list(msg.data)}'
            )
            return

        # UART 프레임의 두 번째 바이트에서 Auto/Manual 모드를 읽습니다.
        mode = int(msg.data[1])

        # Auto 모드 65가 아니면 Manual 모드로 판단합니다.
        new_manual_mode = mode != MODE_AUTO

        # 모드가 변경된 경우 상태를 로그로 출력합니다.
        if new_manual_mode != self.manual_mode:
            mode_name = 'Manual' if new_manual_mode else 'Auto'
            self.get_logger().info(f'Wheel mode changed: {mode_name} ({mode})')

        # 현재 모드를 저장합니다.
        self.manual_mode = new_manual_mode

        # UART 상태 프레임에서 현재 좌측 방향을 읽습니다.
        self.current_left_direction = int(msg.data[2])

        # UART 상태 프레임에서 현재 좌측 속도를 읽습니다.
        self.current_left_speed = int(msg.data[3])

        # UART 상태 프레임에서 현재 우측 방향을 읽습니다.
        self.current_right_direction = int(msg.data[4])

        # UART 상태 프레임에서 현재 우측 속도를 읽습니다.
        self.current_right_speed = int(msg.data[5])

        # Manual 모드로 변경되면 ROS2 주행 상태를 안전하게 초기화합니다.
        if self.manual_mode:
            self.watchdog_stop_sent = True
            self.publish_turn_status(False)

    # 목표 Pose를 받으면 목표 좌표와 초기 회전/부드러운 출발 상태를 설정합니다.
    def goal_callback(self, msg: PoseStamped) -> None:

        # 목표 Pose의 x 좌표를 저장합니다.
        self.goal_x = float(msg.pose.position.x)

        # 목표 Pose의 y 좌표를 저장합니다.
        self.goal_y = float(msg.pose.position.y)

        # 현재 위치와 목표 위치로 초기 거리와 방향 오차를 계산합니다.
        self.calculate_error()

        # 설정이 활성화된 경우에만 초기 제자리 회전을 시작합니다.
        self.start_turn_active = self.start_turn_enabled

        # 부드러운 출발을 새로 시작합니다.
        self.smooth_start_active = self.smooth_start_enabled

        # 부드러운 출발 진행 횟수를 초기화합니다.
        self.smooth_start_step = 0

        # 새 목표 정보를 로그로 출력합니다.
        self.get_logger().info(
            'Goal received: '
            f'x={self.goal_x:.3f}, y={self.goal_y:.3f}, '
            f'distance={self.goal_distance:.3f}, '
            f'yaw_error={self.goal_yaw_error:.3f}'
        )

    # AMCL Pose에서 현재 위치와 quaternion 방향을 읽습니다.
    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:

        # 현재 위치 x 좌표를 저장합니다.
        self.pose_x = float(msg.pose.pose.position.x)

        # 현재 위치 y 좌표를 저장합니다.
        self.pose_y = float(msg.pose.pose.position.y)

        # orientation 메시지를 지역 변수로 가져옵니다.
        orientation = msg.pose.pose.orientation

        # Quaternion을 yaw로 변환하여 저장합니다.
        self.pose_yaw = self.quaternion_to_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

    # 장애물 검출 상태를 처리하고 검출 순간 즉시 첫 감속 명령을 발행합니다.
    def obstacle_callback(self, msg: Int16) -> None:

        # 0이 아닌 값은 장애물 검출 상태로 판단합니다.
        new_obstacle_detected = int(msg.data) != 0

        # 장애물 상태가 변경된 경우에만 로그를 출력합니다.
        if new_obstacle_detected != self.obstacle_detected:
            state = 'detected' if new_obstacle_detected else 'cleared'
            self.get_logger().warning(f'Obstacle {state}')

        # 새로운 장애물 상태를 저장합니다.
        self.obstacle_detected = new_obstacle_detected

        # 장애물이 검출되면 Auto 모드에서 즉시 첫 감속 명령을 발행합니다.
        if self.obstacle_detected and not self.manual_mode:
            left_direction, left_speed, right_direction, right_speed = (
                self.obstacle_deceleration()
            )
            self.publish_wheel_command(
                left_direction,
                left_speed,
                right_direction,
                right_speed,
            )
            self.publish_turn_status(False)

        # 장애물이 해제되면 다음 일반 주행을 부드럽게 시작합니다.
        elif not self.obstacle_detected:
            self.smooth_start_active = self.smooth_start_enabled
            self.smooth_start_step = 0

    # 일정 시간 cmd_vel이 들어오지 않으면 모터 정지 명령을 발행합니다.
    def watchdog_callback(self) -> None:

        # timeout 기능이 0 이하로 설정되어 있으면 watchdog을 사용하지 않습니다.
        if self.cmd_vel_timeout_sec <= 0.0:
            return

        # Manual 모드에서는 UART 노드가 wheel_cmd를 무시하므로 watchdog을 실행하지 않습니다.
        if self.manual_mode:
            return

        # 현재 ROS2 Clock 시각을 읽습니다.
        now = self.get_clock().now()

        # 마지막 cmd_vel 수신 이후 경과 시간을 초 단위로 계산합니다.
        elapsed_sec = (
            now - self.last_cmd_vel_time
        ).nanoseconds / 1_000_000_000.0

        # timeout을 초과하지 않았으면 아무 동작도 하지 않습니다.
        if elapsed_sec <= self.cmd_vel_timeout_sec:
            return

        # 같은 timeout 상태에서 이미 정지 명령을 보냈으면 중복 발행하지 않습니다.
        if self.watchdog_stop_sent:
            return

        # cmd_vel 단절 시 좌우 휠 정지 명령을 발행합니다.
        self.publish_stop()

        # 이번 timeout 구간에서 정지 명령을 보냈음을 기록합니다.
        self.watchdog_stop_sent = True

        # timeout 발생 사실을 경고 로그로 출력합니다.
        self.get_logger().warning(
            f'cmd_vel timeout: {elapsed_sec:.3f} s; stop command published'
        )


# main 함수는 ROS2 초기화, 노드 실행, 종료 처리를 담당합니다.
def main(args=None) -> None:

    # ROS2 Python 통신 계층을 초기화합니다.
    rclpy.init(args=args)

    # 생성자 예외 시에도 종료 처리가 가능하도록 노드 변수를 먼저 초기화합니다.
    node = None

    # WheelNavCommandNode 객체를 생성합니다.
    node = WheelNavCommandNode()

    # Ctrl+C 종료를 처리할 수 있도록 try 블록에서 spin을 실행합니다.
    try:

        # ROS2 Executor가 Subscriber와 Timer 콜백을 계속 처리하도록 합니다.
        rclpy.spin(node)

    # 사용자가 Ctrl+C를 입력하면 정상 종료 절차로 이동합니다.
    except KeyboardInterrupt:
        pass

    # 정상 종료와 예외 종료 모두에서 정리 작업을 수행합니다.
    finally:

        # 노드 생성이 완료되었고 ROS2 context가 유효하면 정지 명령을 한 번 시도합니다.
        if node is not None and rclpy.ok():
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.05)

        # 노드가 생성된 경우 Publisher, Subscriber, Timer 자원을 해제합니다.
        if node is not None:
            node.destroy_node()

        # ROS2 통신 계층을 종료합니다.
        rclpy.shutdown()


# 파일을 직접 실행했을 때 main 함수를 호출합니다.
if __name__ == '__main__':
    main()
