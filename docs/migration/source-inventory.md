# 원본 통합 기록

리팩터링은 원본을 수정하지 않고 다음 세 폴더를 읽어 새 저장소로 통합했습니다.

| 원본 | 채택한 자산 | 제외·대체한 자산 |
| --- | --- | --- |
| `main_ws_최종/main_ws` | 최신 순환 ROS 2 패키지, 저수준 벨트 펌웨어, MATLAB 디지털 트윈, QR 도구 | `.venv-yolo`, `build/install/log`, 압축 백업, 날짜별 코드 복사본 |
| `Loading_바탕화면/Loading_바탕화면` | 플랫폼 Arduino, OBB 모델, 최신 데이터 변환·학습 도구 | 구형 `platform_control` ROS 패키지, 중복 데이터 업로드 청크, 학습 run |
| `hik_camera_바탕화면/hik_camera_바탕화면` | Python Hikrobot MVS ROS 2 드라이버 | Foxy 기준 C++ `hk_camera`를 기본 빌드에서 제외 |

## 기준본 결정

- `firmware/refuge_belt_controller`: 2026-06-29 저수준 제어기를 실제 운용 기준으로 채택했습니다.
- `firmware/legacy/refuge_planner_on_mcu`: `refuge_modular_v2` 계열은 고수준 계획까지 MCU에 포함한 과거 실험으로 분류했습니다.
- `firmware/platform_controller`: 별도 `Loading` 폴더에 있던 통합 플랫폼 제어기를 복구했습니다.
- `ros2_ws/src/platform_loading_control`: 플랫폼 Serial 포트의 유일한 소유자입니다. 이전 yaw 전용 노드는 함께 실행하지 않습니다.
- `ros2_ws/src/hik_camera`: `/opt/MVS` Python binding을 사용하는 Humble 대상 기본 카메라 패키지입니다.

## 의도적으로 가져오지 않은 항목

- 10.5GB Python 가상환경
- 3.6GB `box_perception.zip`과 2.1GB `dataset.zip`
- `build/`, `install/`, `log/`, `runs/`, `training_runs/`
- 동일 MATLAB 파일의 `before_*`, `wip_*` 복사본
- Roboflow 업로드용 데이터 청크
- 라이선스가 불명확하고 Foxy 기준인 C++ `hk_camera` 포크
