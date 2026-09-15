# Meta Glasses ↔ ROS 2 게이트웨이

AXIS 통합 복제판의 웹 명령을 팀에서 합의한 ROS 2 토픽으로 변환합니다. 전체 계약은 `ROBOT_CONTROL_CONTRACT.md`를 기준으로 하며 토픽명은 `topics.json`에서 관리합니다.

## 준비된 방향

글래스 → 로봇:

- `/meta/x_vel`: `std_msgs/msg/Float32`
- `/meta/z_angle`: `std_msgs/msg/Float32`
- `/server/light_cmd`: `std_msgs/msg/Int32`
  - `0`: 전조등 OFF
  - `1`: 전조등 ON
  - `2`: 상부 경고등 ON
  - `3`: 상부 경고등 OFF
- `/meta/warning_broadcast`: `std_msgs/msg/Int32` (`0/1`)
- `/meta/waypoint_goal`: `std_msgs/msg/Int32` (`1~6`)

로봇 → 글래스:

- `/meta/connection_status`: `std_msgs/msg/Int32`
- `/meta/robot_area`: `std_msgs/msg/Int32` (`1~6`)
- `/meta/warning_broadcast_state`: `std_msgs/msg/Int32` (`0/1`)
- `/wheelchair/battery_percent`: `std_msgs/msg/Int32` (`0~100`)
- `/battery/soc`: `sensor_msgs/msg/BatteryState` (`percentage` is `0.0~1.0`)

배터리 토픽은 두 형식을 모두 구독합니다. Orin 담당자는 둘 중 현재 시스템과 맞는 하나만 발행하면 됩니다. 토픽 이름이 다르면 `topics.json`의 `battery_percent` 또는 `battery_state` 값만 변경하세요.

## 실행

ROS 2 환경을 먼저 source한 터미널에서 실행합니다.

```bash
python3 -m pip install -r requirements.txt
python3 meta_gateway.py
```

로컬 테스트 주소는 `ws://로봇PC주소:8765`입니다. 공개 HTTPS 웹앱에서 실제 글래스로 연결할 때는 이 서버 앞에 TLS 프록시를 두어 `wss://` 주소를 만들어야 합니다.

웹앱 연결 예시:

```text
https://stage-coastalpatrol-webapp.vercel.app/?ws=wss%3A%2F%2F게이트웨이주소
```

## 안전 정지

마지막 주행 명령 이후 0.4초 동안 새 명령이 없거나 마지막 글래스 연결이 끊기면 게이트웨이가 `x_vel=0`, `z_angle=0`을 발행합니다. 로봇 구동 노드에도 별도의 watchdog을 두는 것을 권장합니다.

## AXIS 카메라 → 글래스 MJPEG 시험

ROS 2 Humble에서 `/axis/image_raw/compressed`를 HTTP MJPEG로 중계합니다. JPEG
데이터를 다시 디코딩·인코딩하지 않으므로 Jetson CPU 부하와 지연을 줄입니다.

```bash
source /opt/ros/humble/setup.bash
python3 axis_mjpeg_bridge.py \
  --topic /axis/image_raw/compressed \
  --host 0.0.0.0 \
  --port 8080
```

Jetson에서 확인:

```bash
curl http://127.0.0.1:8080/health
curl -o snapshot.jpg http://127.0.0.1:8080/snapshot
```

같은 Wi-Fi의 휴대폰에서 `http://JETSON_IP:8080/stream`을 먼저 엽니다. 이후
글래스 웹앱은 다음 형식으로 실행합니다.

```text
https://WEBAPP_URL/?stream=http%3A%2F%2FJETSON_IP%3A8080%2Fstream&streamType=mjpeg
```

rosbag 시험:

```bash
ros2 bag play BAG_DIRECTORY --loop
ros2 topic hz /axis/image_raw/compressed
```

`coastal-axis-mjpeg.service`는 Jetson 부팅 자동 실행용 예시입니다. 파일 안의 사용자명과
경로를 실제 값으로 바꾼 뒤 `/etc/systemd/system/`에 설치해야 합니다.
