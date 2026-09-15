# CoastalPatrol AXIS 복제판 토픽 계약

## 1. 시스템 역할

```text
Meta Display Glasses
  → 우리 WebSocket 서버
  → ROS 2 토픽
  → Jetson 및 로봇 하드웨어
```

- 글래스: 제스처, 조이스틱, PTZ, 조명 UI 제공
- 우리 서버: 웹 명령 수신 및 ROS 2 토픽 변환
- Jetson: 카메라·조명·Nav·로봇 구동부 연결
- 웨이포인트: 글래스 앱 기준 `WP1~WP4`

## 2. 로봇 수동 조종

| 방향 | 토픽 | 타입 | 의미 |
|---|---|---|---|
| 서버 → Jetson | `/server/robot_cmd` | `geometry_msgs/msg/Twist` | 글래스 조이스틱 속도 명령 |

사용 필드:

```yaml
linear:
  x: 0.0   # 전진(+), 후진(-)
angular:
  z: 0.0   # 좌회전(+), 우회전(-)
```

- 웹앱 제한: 선속도 최대 `±0.10m/s`, 각속도 최대 `±0.30rad/s`
- 명령이 0.4초간 끊기거나 웹 연결이 종료되면 `Twist 0` 발행
- 조이스틱을 중앙으로 옮기면 `Twist 0` 발행

## 3. 로봇 모드

| 방향 | 토픽 | 타입 | 값 |
|---|---|---|---|
| 서버 → Jetson | `/server/robot_mode` | `std_msgs/msg/Int16` | `0=수동`, `1=자동` |

현재는 토픽만 준비되어 있으며 자동/수동 선택 버튼은 아직 구현하지 않았다.
따라서 조이스틱 ON/OFF, 웨이포인트 선택, 웹 연결 종료 시 앱이 이 토픽을
자동으로 발행하지 않는다.

## 4. Nav 목표 도착 상태

| 방향 | 토픽 | 타입 | 의미 |
|---|---|---|---|
| 서버 → Jetson | `/robot_nav/goal` | `std_msgs/msg/Int32` | 선택한 목표 웨이포인트 번호 `1~4` |
| Jetson → 서버 | `/robot_nav/goal_success` | `std_msgs/msg/Int32` | 로봇이 현재 도달한 웨이포인트 번호 `1~4` |

### goal_success 규칙

- `/robot_nav/goal`은 앱·글래스에서 선택한 목표 WP 번호 `1~4`를 한 번 발행한다.
- 로봇은 웨이포인트에 도달할 때마다 `/robot_nav/goal_success`로 해당 번호 `1~4`를 발행한다.
- 서버는 수신 번호가 선택한 `/robot_nav/goal` 번호와 같으면 최종 도착으로 판단한다.
- 목표에 도달하기 전 번호는 통과 웨이포인트로 처리하며 앱과 글래스에서 초록색으로 누적 표시한다.

### WP2 → WP4 정방향

```text
이동 경로:                    WP2 → WP3 → WP4
/robot_nav/goal:                4
/robot_nav/goal_success:        2  →  3  →  4
```

### WP2 → WP4 역방향

```text
이동 경로:                    WP2 → WP3 → WP4
/robot_nav/goal:                4
/robot_nav/goal_success:        2  →  1  →  4

## 5. AXIS robot_vision

| 방향 | 토픽 | 타입 | 의미 |
|---|---|---|---|
| 서버 → Jetson | `/axis/axis_cmd` | `axis_camera_msgs/msg/Axis` | AXIS PTZ 명령 |
| Jetson → 서버 | `/axis/state` | `axis_camera_msgs/msg/Axis` | AXIS PTZ 현재 상태 |
| Jetson → 서버 | `/axis/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | AXIS JPEG 영상 |

PTZ에서 사용하는 주요 필드:

```yaml
pan: 0.0
tilt: 0.0
zoom: 0.0
```

- `pan`, `tilt` 명령 범위: `-100~100`
- PTZ 명령이 0.5초간 끊기면 `pan=0`, `tilt=0` 자동 발행
- 마지막 웹 연결이 종료돼도 PTZ 정지 명령 발행
- `/axis/image_raw/compressed`는 `axis_mjpeg_bridge.py`를 통해 `/stream.mjpg`로 중계

## 6. server_light

전면등과 상부 경고등은 별도 토픽을 사용하지 않고 하나의 명령 토픽으로 통합한다.

| 방향 | 토픽 | 타입 |
|---|---|---|
| 서버 → Jetson | `/server/light_cmd` | `std_msgs/msg/Int32` |

명령 값:

| 값 | 동작 |
|---:|---|
| `0` | 전조등 OFF |
| `1` | 전조등 ON |
| `2` | 상부 경고등 ON |
| `3` | 상부 경고등 OFF |

전조등과 경고등의 상태는 서로 독립적이다. 예를 들어 전조등이 켜진 상태에서
`2`를 발행하면 전조등 상태는 유지하면서 경고등만 켠다. Jetson은 마지막
명령 하나만 전체 조명 상태로 해석하지 말고 전조등과 경고등 상태를 각각
보관해야 한다.

## 7. robot_connection 서비스

`/robot/connection`은 토픽이 아니라 Jetson과 서버가 요청·응답하는 ROS 2
서비스다.

| 항목 | 값 |
|---|---|
| 서비스 이름 | `/robot/connection` |
| 방향 | Jetson ↔ Server |
| Jetson 요청 | `int 1` |
| 서버 준비 안 됨 | `int 0` 또는 무응답 |
| 서버 준비 완료 | `int 1` |

상태 흐름:

```text
1. 로봇 전원 ON
2. Jetson이 /robot/connection으로 int 1 요청
3. 서버 응답이 0 또는 timeout이면 대기
4. Jetson이 일정 주기로 int 1 재요청
5. 서버 응답이 1이면 연결 완료
6. localization과 Nav를 최초 한 번만 실행
```

Jetson의 요청값 `1`은 로봇 실행 명령이 아니라 Jetson의 생존 및 연결 요청을
의미한다. localization/Nav 실행 여부는 서버 응답값으로 결정한다.

| Jetson 요청 | 서버 응답 | 결과 |
|---:|---:|---|
| `1` | `0` | 서버 준비 안 됨, 대기 |
| `1` | 무응답 | 연결 실패/timeout, 대기 후 재시도 |
| `1` | `1` | 서버 준비 완료, localization/Nav 시작 |

안전 규칙:

- 서버 응답 `1`이 반복돼도 localization/Nav를 중복 실행하지 않는다.
- 서비스 호출에 timeout을 둔다.
- 서버 연결이 끊기면 로봇 속도를 0으로 만든다.
- 재연결 시 기존 localization/Nav 프로세스 실행 여부를 먼저 확인한다.

정확한 ROS 2 서비스 코드를 작성하려면 서비스 타입과 필드명이 추가로
필요하다. 확인 명령:

```bash
ros2 service type /robot/connection
ros2 service info /robot/connection
```

예상 개념 구조는 다음과 같지만 실제 `.srv` 정의를 우선한다.

```srv
int32 jet_state
---
int32 server_state
```

## 8. 서버 실행

```bash
source /opt/ros/humble/setup.bash
source ~/ROS_WORKSPACE/install/setup.bash
cd coastalpatrol-axis-robot-control/ros_gateway
python3 -m pip install -r requirements.txt
python3 meta_gateway.py
```

`axis_camera_msgs`가 포함된 ROS 2 워크스페이스를 먼저 빌드하고 source해야 한다.

영상 중계는 별도 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ROS_WORKSPACE/install/setup.bash
python3 axis_mjpeg_bridge.py \
  --topic /axis/image_raw/compressed \
  --host 0.0.0.0 \
  --port 8080
```

## 9. 토픽 및 서비스 확인

```bash
ros2 topic echo /server/robot_cmd
ros2 topic echo /server/robot_mode
ros2 topic echo /robot_nav/goal
ros2 topic echo /robot_nav/goal_success
ros2 topic echo /axis/axis_cmd
ros2 topic echo /axis/state
ros2 topic hz /axis/image_raw/compressed
ros2 topic echo /server/light_cmd
ros2 service type /robot/connection
ros2 service info /robot/connection
```

메시지 타입 확인:

```bash
ros2 topic list -t
ros2 topic info /robot_nav/goal_success --verbose
ros2 topic info /axis/axis_cmd --verbose
ros2 interface show axis_camera_msgs/msg/Axis
```
