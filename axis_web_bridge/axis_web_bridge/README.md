# AXIS Web Bridge (ROS 2 Humble)

Meta Display 웹앱과 AXIS ROS 2 토픽 사이의 테스트/운영 중계 패키지입니다.

## 연결

| 방향 | 입력 | 출력 |
|---|---|---|
| ROS → 웹 | `/axis/image_raw/compressed` | `http://JETSON_IP:8766/stream.mjpg` |
| ROS → 웹 | `/axis/state` | `ws://JETSON_IP:8766/ws`의 `ptz_state` JSON |
| 웹 → ROS | `camera_ptz` JSON | `/axis/axis_cmd` |

rosbag과 실제 카메라 노드가 같은 토픽/타입을 사용하므로 입력 발생원만 교체하면 됩니다.

## Jetson 설치

`axis_camera_msgs`, `axis_camera_ros2`, `axis_web_bridge`를 동일한 워크스페이스의 `src`에 넣습니다.

```bash
mkdir -p ~/axis_humble_ws/src ~/axis_humble_ws/bags
cd ~/axis_humble_ws
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-aiohttp ros-humble-rosbag2-storage-mcap
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## rosbag 확인 및 재생

```bash
ros2 bag info ~/axis_humble_ws/bags/axis_relay_test2
ros2 bag play ~/axis_humble_ws/bags/axis_relay_test2 --loop
```

Jazzy에서 기록한 MCAP이 Humble에서 열리지 않으면 오류 메시지를 보존하고 `metadata.yaml`을 임의로 수정하지 마세요.

## 중계 서버 실행

새 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source ~/axis_humble_ws/install/setup.bash
ros2 launch axis_web_bridge axis_web_bridge.launch.py
```

Jetson IP 확인:

```bash
hostname -I
```

동작 확인:

```text
http://JETSON_IP:8766/health
http://JETSON_IP:8766/stream.mjpg
```

웹앱 테스트 주소:

```text
http://PC_IP:8765/?stream=http%3A%2F%2FJETSON_IP%3A8766%2Fstream.mjpg&streamType=mjpeg&ws=ws%3A%2F%2FJETSON_IP%3A8766%2Fws
```

## PTZ 메시지

웹앱 → 중계 서버:

```json
{"type":"camera_ptz","pan":5,"tilt":0,"active":true}
```

중계 서버 → 웹앱:

```json
{"type":"ptz_state","pan":12.5,"tilt":-2.0,"zoom":1000,"connected":true}
```

WebSocket이 끊기거나 0.5초 동안 새 명령이 없으면 중계 서버가 정지 명령을 발행합니다.

## 실제 카메라로 교체

rosbag을 `Ctrl+C`로 종료한 후:

```bash
ros2 launch axis_camera_ros2 axis_camera.launch.py hostname:=CAMERA_IP
```

중계 서버와 웹앱 주소는 변경할 필요가 없습니다.
