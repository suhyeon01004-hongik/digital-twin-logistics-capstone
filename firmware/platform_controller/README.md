# Platform Controller Firmware

Arduino Mega 2560에서 플랫폼의 리프트, 푸셔, yaw 회전판, 층별 배리어와
하차 플레이트를 제어하는 통합 펌웨어입니다. ROS 2의
`platform_loading_control`과 9600 baud Serial로 통신합니다.

## 핀맵

| 기능 | Arduino Mega 핀 |
| --- | --- |
| Lift STEP / DIR | D3 / D4 |
| Pusher STEP / DIR | D5 / D6 |
| Unload plate servo | D8 |
| Barrier servo F1 / F2 / F3 | D9 / D11 / D12 |
| Yaw servo | D10 |

서보와 모터 드라이버는 별도 전원을 사용하고 Arduino와 GND를 공통으로
연결합니다. 현재 리미트 스위치나 절대 위치 센서는 연결되어 있지 않습니다.

현재 펌웨어에는 물리 E-stop 입력과 공통 motion abort 명령이 없으며, 푸셔 pulse
출력 중에는 command parser가 대기하는 blocking 구조입니다. 따라서 ROS의
`stop/cancel`은 이미 시작된 푸셔 동작을 즉시 멈추지 못합니다. 실기에서는 모터
전원을 직접 차단하는 하드웨어 E-stop을 사용하고, 무인 운전 전에는 푸셔를
non-blocking 상태머신으로 바꾸고 abort 경로를 추가해야 합니다.

## 운용 명령

```text
S <angle>          yaw servo absolute angle
T <angle|UP|DOWN>  unload plate
B <floor> <angle|UP|DOWN>
Z <mm>             lift relative jog
Z0                 accept current lift offset as zero
PM <mm>            pusher absolute software position
PR <mm>            pusher relative movement
H                   accept current pusher position as zero
?                   print help
```

`Y`, `R`, `P`, `L` 명령은 초기 단독 시험과 수동 조작을 위해 유지합니다.
통합 운용에서는 위의 운용 명령만 사용합니다.

## 안전 및 보정

- `AUTO_HOME_PUSHER_ON_START`는 물리 홈 센서가 없으므로 `false`를 유지합니다.
- `H`와 `Z0`는 축을 움직여 홈을 찾는 명령이 아니라 현재 위치를 0으로
  받아들이는 명령입니다.
- `L U`와 `L D`는 소프트웨어상 1~3층 범위를 벗어나면 거부합니다. 리미트
  스위치가 없으므로 이 검사는 실제 기구 원점을 보장하지 않습니다.
- 첫 실기 시험은 모터 전원을 분리한 통신 확인 후, 저속·짧은 거리 jog,
  방향, 이동량, 층 위치, 연속 시퀀스 순으로 진행합니다.
- TB6600의 물리 DIP 설정과 코드의 보정 펄스 값은 서로 다를 수 있으므로
  `PULSES_PER_REV` 변경 전 실측 이동량을 기록합니다.

컴파일에는 Arduino 기본 `Servo` 라이브러리만 필요합니다.
