# Jetson AGX Orin BMS CAN → ROS2

## 핵심 목표

BMS의 5개 값을 각각 따로 publish하지 않고 **ROS2 토픽 하나**에 JSON 문자열로 묶어서 publish합니다.

Topic:

```text
/battery_status
```

Type:

```text
std_msgs/msg/String
```

Payload:

```json
{"mode":"Standby","fault":"Normal","soc":64,"soh":99,"remaining_time_min":0}
```

포함되는 5개 값:

1. `mode` : 동작 모드
2. `fault` : 고장 진단
3. `soc` : 배터리 잔량
4. `soh` : 배터리 수명
5. `remaining_time_min` : 잔여 시간

---

## 파일 구조

```text
battery_can_ros2/
├── README.md
├── can_setup.sh
├── battery_status_publisher.py
├── run_publisher.sh
├── start_battery_can.sh
├── verify_battery_topic.sh
├── install_autostart.sh
├── config/
│   └── battery_can.yaml
├── systemd/
│   ├── battery-can-setup.service
│   └── battery-status-publisher.service
└── tools/
    └── can_reset.sh
```

`tools/can_reset.sh`는 정상 실행에 필요한 파일이 아니라 **복구용**입니다.

---

## 1. 하드웨어

AGX Orin 40-pin:

| Pin | 역할 | Waveshare |
|---|---|---|
| 1 | 3.3 V | 3.3V |
| 30 | GND | GND |
| 29 | CAN0_DIN / RX | CAN_RX |
| 31 | CAN0_DOUT / TX | CAN_TX |

BMS:
- CAN-H → CAN-H
- CAN-L → CAN-L
- 검증된 구성에서 사용하지 않던 나머지 선은 그대로 미연결

---

## 2. CAN 설정

- interface: `can0`
- bitrate: `500000`
- pinmux:
  - `0x0c303018 = 0x0000C458`
  - `0x0c303010 = 0x0000C400`

수동 설정:

```bash
cd ~/battery_can_ros2
sudo ./can_setup.sh
```

정상 확인:

```bash
candump can0
```

---

## 3. ROS2 publisher 실행

CAN 데이터가 `candump can0`에서 보이는 것을 먼저 확인한 뒤:

```bash
cd ~/battery_can_ros2
./run_publisher.sh
```

다른 터미널:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /battery_status
```

예:

```text
data: '{"mode":"Standby","fault":"Normal","soc":64,"soh":99,"remaining_time_min":0}'
```

즉 팀에서는 `/battery_status` **하나만 subscribe**하면 됩니다.

---

## 4. CAN 설정 + publisher 한 번에 실행

```bash
cd ~/battery_can_ros2
./start_battery_can.sh
```

이 스크립트가:

1. `sudo ./can_setup.sh`
2. `./run_publisher.sh`

순서로 실행합니다.

---

## 5. Topic 검증

publisher가 실행 중일 때:

```bash
cd ~/battery_can_ros2
./verify_battery_topic.sh
```

---

## 6. BMS frame mapping

현재 기준:

```text
CAN ID = 0x100
DLC = 8

Byte 0   -> SOC
Byte 1   -> SOH
Byte 2   -> Mode
Byte 3   -> Fault
Byte 4-5 -> Remaining Time
Byte 6   -> Firmware Version (현재 ROS2 topic에는 미포함)
Byte 7   -> Alive Count      (현재 ROS2 topic에는 미포함)
```

Mode:

```text
0x00 = Standby
0x01 = Charging
0x02 = Discharging
```

Fault:

```text
0x00 = Normal
기타 = Fault(0xXX)
```

`remaining_time_min`의 Byte 4/5 endian은
`config/battery_can.yaml`의 `remaining_time_endian`으로 변경할 수 있습니다.

---

## 7. reset shell은 왜 tools/에 있나?

`can_reset.sh`는 정상 동작에 필수 아님.

사용할 때:
- 이전 테스트 때문에 CAN controller가 ERROR-PASSIVE/BUS-OFF로 꼬였을 때
- `can_setup.sh`만으로 정상 상태로 돌아오지 않을 때

평상시에는 사용하지 않습니다.

특히 다른 팀원이 Jetson native CAN을 사용 중이면 `mttcan` unload 때문에 영향을 줄 수 있으므로 실행하지 않습니다.

---

## 8. 부팅 자동실행

**수동 실행과 ROS2 topic이 모두 검증된 후에만**:

```bash
cd ~/battery_can_ros2
sudo ./install_autostart.sh
```

서비스:

```text
battery-can-setup.service
battery-status-publisher.service
```

상태 확인:

```bash
systemctl status battery-can-setup.service --no-pager
systemctl status battery-status-publisher.service --no-pager
```

---

## 최종 데이터 흐름

```text
48V BMS
  ↓  CAN-H / CAN-L
Waveshare CAN Transceiver
  ↓
Jetson AGX Orin CAN0
  ↓
battery_status_publisher.py
  ↓
ROS2 /battery_status
  ↓
{
  mode,
  fault,
  soc,
  soh,
  remaining_time_min
}
  ↓
팀 통합 / Glass
```
