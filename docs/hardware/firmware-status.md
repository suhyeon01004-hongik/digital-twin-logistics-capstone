# 펌웨어 상태

| 경로 | 상태 | 역할 |
| --- | --- | --- |
| `firmware/refuge_belt_controller` | 운용 정본 / 실기 재검증 필요 | B1~B4, 엔코더 PI, ToF, JSON 텔레메트리 |
| `firmware/platform_controller` | 운용 정본 / 실기 재검증 필요 | 리프트, 푸셔, yaw, 배리어, 하차 플레이트 |
| `firmware/legacy/refuge_planner_on_mcu` | legacy / 미검증 | MCU 내부 DB와 순환 계획 실험 |
| `firmware/tests/*` | bench test | 축별 배선·거리 검증 |

## 플랫폼 핀맵

| 기능 | 핀 |
| --- | --- |
| Lift STEP / DIR | D3 / D4 |
| Pusher STEP / DIR | D5 / D6 |
| Unload plate servo | D8 |
| Barrier servos F1/F2/F3 | D9 / D11 / D12 |
| Yaw servo | D10 |

실제 하드웨어에는 원점·리미트 센서가 없으므로 부팅 후 물리 위치와 소프트웨어
좌표가 일치하는지 확인해야 합니다. 자동 홈 기능은 검증 전까지 활성화하지 않습니다.
플랫폼 펌웨어에는 공통 abort와 E-stop 입력이 없고 푸셔가 blocking 방식이므로
하드웨어 전원 차단 E-stop과 firmware 상태머신 개선이 필요합니다.
층별 벨트 펌웨어는 fault와 E-stop을 latch하고 명시적 `CLEAR_FAULT` 전까지
motion 명령을 거부하도록 보강했습니다.

## 컴파일 대상

- Board: Arduino Mega 2560
- Refuge dependency: `VL53L0X` library
- Platform dependency: Arduino built-in `Servo`

컴파일 성공은 전기적 안전이나 보정값 정확성을 보장하지 않습니다. 실기 검증은
비상 정지, 축별 저속 jog, 방향, 원점, 거리, 연속 동작 순으로 수행합니다.
