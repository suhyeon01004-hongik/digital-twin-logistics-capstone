# 시스템 아키텍처

## 제어 계층

1. **Main PC**: 택배 인식, DB, 목표 선택, 순환 계획, 안전 검증, 웹 UI와 MATLAB 디지털 트윈을 실행합니다.
2. **Raspberry Pi**: 각 층 Arduino와 연결된 USB Serial을 ROS 2 토픽으로 변환합니다.
3. **층별 Arduino Mega**: B1~B4 모터, 엔코더, ToF를 제어하고 이동 명령을 실행합니다.
4. **플랫폼 Arduino**: 승강축, 메인 푸셔, yaw 서보, 층별 배리어, 하차 플레이트를 제어합니다.

## 핵심 데이터 흐름

```text
Camera image
  -> OBB/QR perception
  -> loading plan request
  -> platform alignment and transfer
  -> floor circulation DB update
  -> MATLAB candidate move validation
  -> encoder-based MOVE
  -> ToF post validation
  -> DB correction and next plan
```

## 소유권 경계

- Arduino는 저수준 이동과 센서 텔레메트리만 담당합니다.
- 택배 DB, 순환·피신 판단, 목표 하차 계획은 ROS 2/MATLAB이 담당합니다.
- 플랫폼과 층별 컨베이어의 Serial 포트는 각각 하나의 ROS 노드만 소유해야 합니다.
