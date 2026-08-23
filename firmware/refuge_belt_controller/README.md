# Refuge Belt Controller Firmware

각 층의 Arduino Mega 2560에서 B1~B4 컨베이어, 엔코더 기반 RPM PI 제어,
TCA9548A를 통한 VL53L0X ToF, 거리 이동과 JSON 텔레메트리를 담당하는 저수준
운용 펌웨어입니다. 택배 DB와 순환 계획은 MCU가 아니라 ROS 2/MATLAB이
소유합니다.

## 통신

- Baud: `115200`
- 시작 이벤트: `{"event":"ready","name":"refuge_low_level"}`
- 주요 명령: `PING`, `MOVE`, `AUXRUN`, `STOP`, `STOPB`, `ZERO`, `STATUS`,
  `SET`, `CLEAR_FAULT`
- 주요 이벤트: `move_start`, `move_done`, `fault`, `telemetry`

정확한 명령 생성은
`ros2_ws/src/refuge_circulation_control/refuge_circulation_control/supervisor.py`가
담당합니다. 수동 serial monitor와 ROS bridge를 동시에 연결하지 않습니다.

## 안전 동작

- E-stop은 D40, `INPUT_PULLUP`, LOW 활성으로 설정되어 있습니다.
- E-stop 또는 fault가 활성화되면 `RUN`, `MOVE`, `AUXRUN`을 거부합니다.
- 모든 fault 진입은 먼저 B1~B4 출력을 정지합니다.
- fault는 원인을 제거한 뒤 `CLEAR_FAULT`로만 해제합니다. E-stop이 눌린
  상태에서는 해제 명령도 거부합니다.
- encoder 변화가 없거나 최대 이동 시간을 넘기면 모든 모터를 정지하고 fault를
  latch합니다.
- 코드의 보정 테이블은 실험값이며 다른 층·벨트·장력에 그대로 적용하지 않습니다.

## 빌드 조건

- Board: Arduino Mega 2560
- Library: `VL53L0X`
- I2C multiplexer: TCA9548A

현재 환경에는 `arduino-cli`와 실제 보드가 없어 컴파일·업로드를 실행하지
못했습니다. 실기 전 핀맵, 회전 방향, encoder 부호, ToF 채널과 E-stop을 축별로
검증합니다.
