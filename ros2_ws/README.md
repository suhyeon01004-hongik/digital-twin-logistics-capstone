# ROS 2 워크스페이스

기준 환경은 Ubuntu 22.04, ROS 2 Humble, Python 3.10입니다.

| 패키지 | 역할 | 운용 상태 |
| --- | --- | --- |
| `refuge_circulation_control` | 층별 supervisor, Pi Serial bridge, 웹 UI, MATLAB 연동 | 핵심 상위 제어 |
| `platform_loading_control` | OBB/QR 인식과 플랫폼 적재·하차 상태머신 | 핵심 플랫폼 제어 |
| `hik_camera` | Hikrobot MVS 이미지 publisher·recorder | MVS SDK 필요 |
| `qr_scanner` | 독립 QR 생성·스캔 도구 | 선택 유틸리티 |

`qr_scanner`의 기본 입력 `/box_detector/roi_image`는 현재 핵심 launch에서
발행하지 않습니다. 통합 경로에서는 `platform_loading_control`이 QR을 직접
디코딩하며, 독립 scanner를 사용할 때만 `image_topic`을 실제 영상 토픽으로
지정합니다.

## 빌드

저장소 루트에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install
source ros2_ws/install/setup.bash
source scripts/milemate_env.sh
```

`build`, `install`, `log`는 생성물이며 Git에 포함하지 않습니다. 모델, MATLAB,
데이터와 MVS SDK 경로는 `scripts/milemate_env.sh`의 환경변수로 설정합니다.

## 토픽·Serial 소유권

- 플랫폼 Arduino 포트는 `platform_load_manager` 하나만 엽니다.
- 층별 Arduino 포트는 해당 층의 `arduino_bridge` 하나만 엽니다.
- 같은 포트에 수동 serial monitor나 구형 노드를 동시에 연결하지 않습니다.
- 다층 실행에서는 floor prefix가 붙은 토픽을 사용하고 공용 웹 UI가 상태를
  집계합니다.

실행 전 체크리스트는 `docs/operations/runtime-checklist.md`에 있습니다.
