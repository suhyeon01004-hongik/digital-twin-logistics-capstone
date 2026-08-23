# Platform Loading Control

카메라 인식 결과를 회전판, 리프트, 푸셔, 층별 배리어와 하차 플레이트 동작으로
연결하는 ROS 2 패키지입니다. 순환 벨트의 계획과 디지털 트윈 명령은
`refuge_circulation_control`이 담당합니다.

## 노드

| 실행 파일 | 역할 |
| --- | --- |
| `parcel_perception` | 압축 영상에서 OBB·크기·yaw·QR 추정 |
| `platform_load_manager` | 적재·하차 상태머신과 플랫폼 Serial 단독 소유 |
| `debug_image_viewer` | 디버그 압축 영상 표시 |

추론 옵션은 원본 환경에서 사용한 Ultralytics `8.4.67` API에 맞췄으며 CUDA
모델에서는 `half=True`, CPU에서는 FP32를 사용합니다. 순수 기하 계산과 시리얼
읽기 스레드 종료는 `tests/`에서 ROS 없이 검사합니다.

## 처리 흐름

1. `/hik_camera/rgb/compressed`에서 OBB와 선택적 QR을 읽습니다.
2. 택배 크기와 정렬 yaw를 계산하고 회전판을 보정합니다.
3. 소프트웨어 좌표를 기준으로 목표 층 높이로 이동합니다.
4. 푸셔를 B4 접점까지 이동하고 `load_start`를 순환 제어에 전달합니다.
5. 하차 시 목표 택배를 플랫폼으로 받은 뒤 하차 플레이트를 구동합니다.

## 필요한 자산

기본 모델 경로는 저장소 루트의
`artifacts/models/box_obb_s_512/best.pt`입니다. 파일이 없으면 perception 노드는
시작하지 않습니다. `MILEMATE_MODEL_PATH`로 다른 경로를 지정할 수 있습니다.

```bash
python3 scripts/validate_runtime_assets.py
```

데이터셋은 추론에 필요하지 않습니다.

## 실행

실장비 통합 예시입니다.

```bash
ros2 launch platform_loading_control platform_loading.launch.py \
  start_camera:=true start_perception:=true \
  platform_port:=/dev/serial/by-id/YOUR_PLATFORM_ARDUINO \
  target_floor:=1
```

모델, 카메라와 Arduino 없이 제어 launch만 확인할 때는 perception도 꺼야 합니다.

```bash
ros2 launch platform_loading_control platform_loading.launch.py \
  dry_run:=true start_camera:=false start_perception:=false show_debug_view:=false
```

## 주요 launch 인자

| 인자 | 기본값 | 설명 |
| --- | ---: | --- |
| `start_camera` | `false` | Hikrobot camera launch 포함 |
| `start_perception` | `true` | 모델 기반 인식 노드 시작 |
| `show_debug_view` | `true` | 인식과 함께 디버그 창 시작 |
| `dry_run` | `false` | 플랫폼 Serial 쓰기 대신 로그만 출력 |
| `platform_port` | `auto` | 플랫폼 Arduino 포트 |
| `target_floor` | `1` | 기본 적재 층 |
| `floor1_z_mm` | `-10.0` | 1층 소프트웨어 기준 위치 |
| `floor2_z_mm` | `265.0` | 2층 소프트웨어 기준 위치 |
| `floor3_z_mm` | `515.0` | 3층 소프트웨어 기준 위치 |
| `yaw_deadband_deg` | `7.0` | 회전 정렬 허용 오차 |

높이와 푸셔 위치는 절대 센서가 아닌 소프트웨어 좌표입니다. 전원 인가 후 물리
위치와 좌표를 맞추지 않은 상태에서 절대 이동 명령을 보내면 안 됩니다.

`stop/cancel`은 상태머신의 후속 단계를 중단하지만 현재 플랫폼 펌웨어에서 이미
시작한 blocking 푸셔 pulse 출력을 즉시 중단하지는 못합니다. 소프트웨어 stop을
E-stop으로 간주하지 않습니다.

## Serial 규약

통합 펌웨어는 `firmware/platform_controller`에 있습니다. 포트가 끊기면 reader
스레드는 종료되고 다음 명령 시 한 번 재연결합니다. 명령·응답 문자열 호환성은
`scripts/validate_protocols.py`에서 확인합니다.
