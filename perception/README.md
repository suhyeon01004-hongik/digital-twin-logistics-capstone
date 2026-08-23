# 인식·학습 도구

실시간 정본 노드는
`ros2_ws/src/platform_loading_control/platform_loading_control/parcel_perception_node.py`
입니다. OBB 모델로 택배의 회전 사각형, 크기와 yaw를 추정하고 QR을 선택적으로
읽습니다.

`perception/scripts`에는 다음 도구만 보존했습니다.

- 카메라 이미지 캡처
- COCO segmentation → YOLO segmentation 변환
- YOLO segmentation → OBB 라벨 변환
- split 생성과 데이터 설정 파일 생성
- 자동 라벨 후보 생성
- OBB 학습과 런타임 진단

경로는 코드에 사용자 절대경로를 넣지 않고 아래 환경변수로 설정합니다.

| 환경변수 | 기본값 |
| --- | --- |
| `MILEMATE_DATA_ROOT` | `data/local` |
| `MILEMATE_MODEL_ROOT` | `artifacts/models` |
| `MILEMATE_RUNS_ROOT` | `artifacts/runs` |
| `MILEMATE_SEG_MODEL_PATH` | 스크립트별 입력 모델 |

원본 환경에서 확인된 주요 버전은 Python 3.10.12, Ultralytics 8.4.67,
NumPy 1.26.4, OpenCV 4.11.0.86, pyserial 3.5입니다. 공통 Python 의존성은
루트 `requirements.txt`에 기록했습니다. Torch/CUDA/TensorRT는 GPU와 드라이버
조합에 맞춰 별도로 설치합니다.

런타임 모델 정보와 체크섬은 `models/box_obb_s_512/model-card.md`, 데이터 공개
정책은 `data/README.md`를 참고합니다. 모델을 다시 평가하기 전에는 기존 성능을
현재 재현 결과로 표현하지 않습니다.

플랫폼 Serial 포트는 `platform_load_manager`만 소유합니다. 과거 yaw 전용
serial controller와 중복 launcher는 통합본에서 제외했습니다.
