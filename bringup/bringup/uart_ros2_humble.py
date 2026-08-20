#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# threading은 UART 수신 루프를 ROS2 콜백 실행과 분리하기 위해 사용합니다.
import threading

# serial은 pyserial 패키지로 실제 UART 장치와 통신할 때 사용합니다.
import serial

# rclpy는 ROS2 Python 클라이언트 라이브러리입니다.
import rclpy

# Node는 ROS2 노드 클래스를 만들기 위한 기본 클래스입니다.
from rclpy.node import Node

# QoSProfile은 ROS2 토픽의 큐 깊이와 신뢰성 정책을 설정할 때 사용합니다.
from rclpy.qos import QoSProfile

# 기존 코드와 동일하게 휠 명령/상태는 Int16MultiArray, 모드 명령은 Int16을 사용합니다.
from std_msgs.msg import Int16, Int16MultiArray


# UARTCommunicationNode는 ROS2 노드이며 UART 송수신을 담당합니다.
class UARTCommunicationNode(Node):

    # 생성자는 ROS2 노드, 파라미터, UART, Publisher, Subscriber를 초기화합니다.
    def __init__(self) -> None:

        # ROS2 노드 이름을 'uart'로 설정합니다.
        super().__init__('uart')

        # UART 장치 경로를 ROS2 파라미터로 선언합니다.
        self.declare_parameter('port', '/dev/ttyUSB0')

        # UART 통신 속도를 ROS2 파라미터로 선언합니다.
        self.declare_parameter('baudrate', 115200)

        # UART 읽기 timeout을 초 단위 ROS2 파라미터로 선언합니다.
        self.declare_parameter('read_timeout_sec', 0.02)

        # UART 쓰기 timeout을 초 단위 ROS2 파라미터로 선언합니다.
        self.declare_parameter('write_timeout_sec', 0.1)

        # 최대 RX 프레임 길이를 선언하여 비정상 데이터 누적을 방지합니다.
        self.declare_parameter('max_frame_length', 64)

        # 선언한 UART 장치 경로 파라미터를 읽습니다.
        self.port = str(self.get_parameter('port').value)

        # 선언한 baudrate 파라미터를 읽습니다.
        self.baudrate = int(self.get_parameter('baudrate').value)

        # 선언한 UART 읽기 timeout 파라미터를 읽습니다.
        self.read_timeout_sec = float(
            self.get_parameter('read_timeout_sec').value
        )

        # 선언한 UART 쓰기 timeout 파라미터를 읽습니다.
        self.write_timeout_sec = float(
            self.get_parameter('write_timeout_sec').value
        )

        # 선언한 최대 프레임 길이 파라미터를 읽습니다.
        self.max_frame_length = int(
            self.get_parameter('max_frame_length').value
        )

        # depth=1인 QoS를 생성하여 최신 제어 명령 위주로 처리합니다.
        control_qos = QoSProfile(depth=1)

        # wheel_status 토픽 Publisher를 생성합니다.
        self.uart_pub = self.create_publisher(
            Int16MultiArray,
            'wheel_status',
            control_qos,
        )

        # wheel_cmd 토픽 Subscriber를 생성합니다.
        self.cmd_sub = self.create_subscription(
            Int16MultiArray,
            'wheel_cmd',
            self.cmd_callback,
            control_qos,
        )

        # mode_cmd 토픽 Subscriber를 생성합니다.
        self.mode_sub = self.create_subscription(
            Int16,
            'mode_cmd',
            self.mode_callback,
            control_qos,
        )

        # UART에서 수신 중인 프레임 바이트를 임시 저장합니다.
        self.wheel_data: list[int] = []

        # 정지 명령은 원본 코드와 동일하게 [좌방향, 좌속도, 우방향, 우속도, 브레이크]입니다.
        self.stop_cmd: list[int] = [83, 33, 83, 33, 79]

        # mode는 원본 코드에 초기값이 없었으므로 안전하게 수동 모드 77('M')로 초기화합니다.
        self.mode: int = 77

        # 노드 종료 시 UART 수신 스레드를 멈추기 위한 플래그입니다.
        self.running = True

        # 여러 ROS2 콜백이 동시에 UART TX를 수행하지 못하도록 Lock을 생성합니다.
        self.tx_lock = threading.Lock()

        # 지정된 포트와 115200 8N1 조건으로 UART 장치를 엽니다.
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.read_timeout_sec,
            write_timeout=self.write_timeout_sec,
        )

        # UART 수신 루프를 실행할 daemon Thread를 생성합니다.
        self.rx_thread = threading.Thread(
            target=self.rx_loop,
            name='uart_rx_thread',
            daemon=True,
        )

        # UART 수신 Thread를 시작합니다.
        self.rx_thread.start()

        # UART 노드가 정상적으로 시작되었음을 출력합니다.
        self.get_logger().info(
            f'UART opened: port={self.port}, baudrate={self.baudrate}'
        )

    # checksum은 원본 코드와 동일한 8비트 2의 보수 방식을 사용합니다.
    @staticmethod
    def checksum(data: list[int]) -> int:

        # 모든 데이터 합의 2의 보수를 계산하고 하위 8비트만 반환합니다.
        return (~sum(data) + 1) & 0xFF

    # rx_loop는 ROS2 Executor와 분리된 Thread에서 UART 데이터를 계속 읽습니다.
    def rx_loop(self) -> None:

        # ROS2가 정상이고 노드 종료 플래그가 설정되지 않은 동안 반복합니다.
        while rclpy.ok() and self.running:

            # UART 수신 중 예외가 발생해도 원인을 출력할 수 있도록 try 문을 사용합니다.
            try:

                # UART에서 최대 1바이트를 읽습니다. timeout이면 b''가 반환됩니다.
                received = self.ser.read(1)

                # timeout으로 데이터가 없으면 다음 반복으로 넘어갑니다.
                if not received:
                    continue

                # bytes 길이 1의 첫 번째 값을 0~255 정수로 변환합니다.
                received_byte = received[0]

                # 수신된 바이트를 프레임 파서로 전달합니다.
                self.process_rx_byte(received_byte)

            # UART 장치 분리, 권한 문제 등 serial 관련 오류를 처리합니다.
            except serial.SerialException as exc:

                # 오류 내용을 ROS2 로그에 출력합니다.
                self.get_logger().error(f'UART RX serial error: {exc}')

                # 심각한 UART 오류가 발생하면 수신 Thread를 종료합니다.
                self.running = False

            # 예상하지 못한 오류도 숨기지 않고 로그에 출력합니다.
            except Exception as exc:

                # 예외 타입과 내용을 ROS2 로그에 출력합니다.
                self.get_logger().error(f'UART RX unexpected error: {exc}')

                # 반복적인 오류를 방지하기 위해 수신 Thread를 종료합니다.
                self.running = False

    # process_rx_byte는 바이트 단위 입력으로 UART 프레임을 조립하고 검증합니다.
    def process_rx_byte(self, received_byte: int) -> None:

        # 새로운 프레임을 기다리는 상태에서는 Header 72('H')만 허용합니다.
        if not self.wheel_data:

            # Header가 아니면 해당 바이트를 버립니다.
            if received_byte != 72:
                return

            # Header이면 새로운 프레임의 첫 바이트로 저장합니다.
            self.wheel_data.append(received_byte)

            # 첫 Header 처리 후 아래 로직을 실행하지 않고 종료합니다.
            return

        # 이미 Header를 받은 상태이면 수신 바이트를 현재 프레임에 추가합니다.
        self.wheel_data.append(received_byte)

        # 설정한 최대 길이를 초과하면 비정상 프레임으로 판단합니다.
        if len(self.wheel_data) > self.max_frame_length:

            # 길이 초과 경고를 출력합니다.
            self.get_logger().warning(
                'UART RX frame exceeded maximum length; buffer cleared'
            )

            # 비정상 프레임 버퍼를 초기화합니다.
            self.wheel_data.clear()

            # 현재 바이트 처리를 종료합니다.
            return

        # 아직 CR/LF 종료 바이트 [13, 10]이 완성되지 않았으면 계속 수신합니다.
        if len(self.wheel_data) < 4 or self.wheel_data[-2:] != [13, 10]:
            return

        # 종료 바이트 직전 값을 수신 Checksum으로 가져옵니다.
        received_checksum = self.wheel_data[-3]

        # Header 다음부터 Checksum 직전까지를 Checksum 계산 대상 데이터로 가져옵니다.
        checksum_data = self.wheel_data[1:-3]

        # 수신 데이터로 예상 Checksum을 계산합니다.
        expected_checksum = self.checksum(checksum_data)

        # 수신 Checksum과 계산 Checksum이 다르면 프레임을 폐기합니다.
        if received_checksum != expected_checksum:

            # Checksum 불일치 내용을 경고 로그로 출력합니다.
            self.get_logger().warning(
                'UART RX checksum mismatch: '
                f'received={received_checksum}, expected={expected_checksum}, '
                f'frame={self.wheel_data}'
            )

            # 오류 프레임 버퍼를 초기화합니다.
            self.wheel_data.clear()

            # 오류 프레임이므로 publish하지 않고 종료합니다.
            return

        # 검증된 프레임 전체를 ROS2 메시지에 복사합니다.
        uart_msg = Int16MultiArray()

        # Int16MultiArray.data에는 정수 리스트를 저장합니다.
        uart_msg.data = list(self.wheel_data)

        # 검증된 UART 상태 프레임을 wheel_status 토픽으로 발행합니다.
        self.uart_pub.publish(uart_msg)

        # 프레임의 두 번째 바이트를 현재 Auto/Manual 모드로 저장합니다.
        if len(self.wheel_data) > 1:
            self.mode = self.wheel_data[1]

        # 정상 수신 프레임을 디버깅용 로그로 출력합니다.
        self.get_logger().info(f'UART RX: {self.wheel_data}')

        # 다음 프레임 수신을 위해 버퍼를 초기화합니다.
        self.wheel_data.clear()

    # tx는 모터 제어보드로 보낼 전체 UART 프레임을 생성하고 송신합니다.
    def tx(self, wheel_cmd: list[int]) -> None:

        # 모든 데이터가 UART 1바이트 범위인 0~255인지 검사합니다.
        if any(value < 0 or value > 255 for value in wheel_cmd):

            # 유효하지 않은 데이터가 있으면 송신하지 않고 오류를 출력합니다.
            self.get_logger().error(
                f'UART TX value out of byte range: {wheel_cmd}'
            )

            # 잘못된 데이터를 송신하지 않도록 함수를 종료합니다.
            return

        # Header 72, 명령 데이터, Checksum, CR, LF 순으로 전체 프레임을 만듭니다.
        cmd_data = [72] + wheel_cmd + [self.checksum(wheel_cmd), 13, 10]

        # 여러 콜백의 동시 송신으로 프레임이 섞이지 않도록 Lock을 획득합니다.
        with self.tx_lock:

            # 정수 리스트를 bytes로 변환하여 각 값을 정확히 1바이트로 송신합니다.
            self.ser.write(bytes(cmd_data))

            # OS/드라이버 송신 버퍼에 남은 데이터를 즉시 밀어냅니다.
            self.ser.flush()

        # 송신한 전체 프레임을 ROS2 로그로 출력합니다.
        self.get_logger().info(f'UART TX: {cmd_data}')

    # cmd_callback은 wheel_cmd 토픽으로부터 좌우 휠 명령을 받습니다.
    def cmd_callback(self, msg: Int16MultiArray) -> None:

        # 원본 프로토콜은 [좌방향, 좌속도, 우방향, 우속도, 브레이크] 5개를 요구합니다.
        if len(msg.data) != 5:

            # 데이터 길이가 다르면 잘못된 명령으로 판단합니다.
            self.get_logger().warning(
                f'Invalid wheel_cmd length: {len(msg.data)}, data={list(msg.data)}'
            )

            # 잘못된 메시지는 UART로 송신하지 않습니다.
            return

        # Auto 모드 65('A')일 때만 ROS2 주행 명령을 모터 제어보드로 전달합니다.
        if self.mode != 65:

            # 수동 모드에서는 Navigation 휠 명령을 무시합니다.
            self.get_logger().debug(
                f'wheel_cmd ignored because current mode is {self.mode}'
            )

            # 수동 모드이므로 함수를 종료합니다.
            return

        # ROS2 배열을 일반 Python 정수 리스트로 변환합니다.
        wheel_cmd = [int(value) for value in msg.data]

        # Mode 65를 명령 데이터 앞에 추가하여 UART로 송신합니다.
        self.tx([self.mode] + wheel_cmd)

    # mode_callback은 mode_cmd 토픽으로 Auto/Manual 모드 전환 명령을 받습니다.
    def mode_callback(self, msg: Int16) -> None:

        # ROS2 메시지 값을 Python 정수로 저장합니다.
        requested_mode = int(msg.data)

        # 현재 프로토콜에서 허용하는 모드는 Auto 65와 Manual 77입니다.
        if requested_mode not in (65, 77):

            # 지원하지 않는 모드값은 거부하고 경고를 출력합니다.
            self.get_logger().warning(
                f'Unsupported mode command: {requested_mode}'
            )

            # 잘못된 모드이므로 상태를 변경하지 않습니다.
            return

        # 요청된 모드를 현재 모드로 저장합니다.
        self.mode = requested_mode

        # Auto 모드 65('A')이면 Auto 전환 로그를 출력합니다.
        if self.mode == 65:
            self.get_logger().info('[[[ Auto Mode ]]]')

        # Manual 모드 77('M')이면 Manual 전환 로그를 출력합니다.
        else:
            self.get_logger().info('[[[ Manual Mode ]]]')

        # 모드 전환 시 원본 코드와 동일하게 먼저 정지 명령을 전송합니다.
        self.tx([self.mode] + self.stop_cmd)

    # destroy_node는 ROS2 노드 종료 시 UART와 수신 Thread를 정리합니다.
    def destroy_node(self) -> bool:

        # UART 수신 반복문이 종료되도록 플래그를 내립니다.
        self.running = False

        # UART 장치가 열려 있으면 안전하게 닫습니다.
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

        # 수신 Thread가 존재하고 현재 Thread가 아니라면 잠시 종료를 기다립니다.
        if (
            hasattr(self, 'rx_thread')
            and self.rx_thread.is_alive()
            and threading.current_thread() is not self.rx_thread
        ):
            self.rx_thread.join(timeout=1.0)

        # 상위 Node 클래스의 종료 처리를 실행합니다.
        return super().destroy_node()


# main은 ROS2 노드를 초기화하고 Executor에서 실행하는 진입점입니다.
def main(args=None) -> None:

    # ROS2 Python 통신 계층을 초기화합니다.
    rclpy.init(args=args)

    # node 변수를 먼저 None으로 선언하여 생성 실패 시에도 종료 처리가 가능하게 합니다.
    node = None

    # 노드 실행 중 예외와 종료를 안전하게 처리합니다.
    try:

        # UARTCommunicationNode 객체를 생성합니다.
        node = UARTCommunicationNode()

        # Subscription 콜백이 실행되도록 ROS2 Executor에서 노드를 계속 실행합니다.
        rclpy.spin(node)

    # Ctrl+C 입력으로 종료한 경우 별도의 오류로 처리하지 않습니다.
    except KeyboardInterrupt:
        pass

    # UART 포트 열기 실패 등 serial 관련 오류를 화면에 출력합니다.
    except serial.SerialException as exc:
        print(f'Failed to open or use UART: {exc}')

    # 그 밖의 예상하지 못한 예외도 숨기지 않고 출력합니다.
    except Exception as exc:
        print(f'UART node error: {exc}')

    # 정상 종료와 오류 종료 모두에서 자원을 정리합니다.
    finally:

        # 노드가 정상 생성된 경우 Node와 UART 자원을 정리합니다.
        if node is not None:
            node.destroy_node()

        # ROS2가 아직 실행 중이면 통신 계층을 종료합니다.
        if rclpy.ok():
            rclpy.shutdown()


# 이 파일이 직접 실행되었을 때만 main 함수를 호출합니다.
if __name__ == '__main__':
    main()
