# 실장비 실행 체크리스트

이 문서는 Ubuntu 22.04/ROS 2 Humble 장비에서 처음 복원할 때의 순서입니다.
정적 검사를 통과했더라도 아래 확인 없이 전체 자동 시퀀스를 시작하지 않습니다.

## 1. 전원 인가 전

- 기구 간섭, 벨트 장력, 배선 풀림과 공통 GND 확인
- 플랫폼 리프트·푸셔의 실제 위치와 소프트웨어 0점 확인
- 각 층 Arduino의 USB serial by-id와 floor 매핑 기록
- D40 E-stop LOW 활성 및 모터 전원 차단 확인
- 플랫폼 제어기는 소프트웨어 abort가 불완전하므로 별도 하드웨어 전원 차단
  E-stop 확인
- 서보와 모터 드라이버의 별도 전원 용량 확인

## 2. 소프트웨어 검사

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source scripts/milemate_env.sh
bash scripts/validate_ubuntu.sh
```

`validate_runtime_assets.py`가 실패하면 모델 또는 Simulink 파일을 먼저 복원합니다.
MATLAB 회귀는 별도 MATLAB 세션에서 실행하고 결과를 새 날짜 파일로 보존합니다.

## 3. 통신만 확인

1. 모터·서보 전원을 끄고 Arduino USB만 연결합니다.
2. 플랫폼은 `Platform controller ready`, 층별 제어기는
   `refuge_low_level` ready 이벤트를 확인합니다.
3. 한 포트에 ROS bridge와 serial monitor를 동시에 연결하지 않습니다.
4. 카메라는 압축 영상 토픽의 발행 주기와 해상도를 확인합니다.

## 4. 축별 저속 시험

1. E-stop 동작과 fault latch/해제를 먼저 검증합니다.
2. B1~B4를 하나씩 짧은 거리로 움직여 방향과 encoder 부호를 확인합니다.
3. ToF 물체/빈 공간 값과 multiplexer 채널 매핑을 확인합니다.
4. 플랫폼 yaw, 배리어, 하차 플레이트를 작은 범위에서 확인합니다.
5. 리프트·푸셔는 물리 위치를 맞춘 뒤 짧은 jog와 소프트웨어 한계를 확인합니다.

## 5. 단계별 통합

1. 모델 없이 `start_perception:=false`, `dry_run:=true`로 launch 구성 확인
2. 카메라+인식만 실행해 OBB, 크기, yaw와 QR 결과 확인
3. 플랫폼 단독 정렬·승강·푸셔 시퀀스 확인
4. 층별 belt MOVE와 telemetry 확인
5. MATLAB 후보 동작 검증과 실제 명령 결과 비교
6. 빈 시스템 → 단일 택배 → 복수 택배 → 목표 하차 순서로 확대

각 단계에서 명령, 센서값, 실제 이동량, 오차와 수정한 파라미터를 기록합니다.

## 6. 네트워크

웹 UI는 기본 `0.0.0.0:5000`에 열리고 인증이 없습니다. 실험용 폐쇄망 또는
localhost에서만 사용하며 인터넷에 port forwarding 하지 않습니다.
