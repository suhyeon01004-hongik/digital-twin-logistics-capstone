# Box OBB S 512

플랫폼 카메라에서 택배의 회전 사각형, 크기와 yaw를 추정하는 현재 통합 후보
모델입니다.

| 항목 | 값 |
| --- | --- |
| Task | Oriented bounding box detection |
| Base | `yolo11s-obb.pt` |
| Input size | 512 |
| Runtime file | `artifacts/models/box_obb_s_512/best.pt` |
| File size | 19,825,404 bytes |
| SHA-256 | `8f3009348fdf0b9d87e563daee3551fb286f33a6223bd6e6f3d2b2767d951ea2` |
| Source snapshot | Loading folder, 2026-06-26 |
| Verified runtime API | Ultralytics 8.4.67 / NumPy 1.26.4 |

학습 인자는 `training_args.yaml`에 보존했습니다. 위 런타임 모델은 재현 가능한
기본 추론을 위해 Git에 함께 커밋합니다. 다른 가중치와 학습 산출물은 포함하지
않습니다.

현재 모델은 파일 무결성과 ROS 경로 연결만 확인된 상태입니다. 공개 전에는
평가 데이터의 중복 제거, mAP/각도 오차/실시간 지연 재측정, 학습 데이터
라이선스와 개인정보 검토가 필요합니다.

추론에는 이 가중치가 필요하지만 학습 이미지·라벨은 필요하지 않습니다. 새로
clone한 환경에서는 추가 복사 없이 `python3 scripts/validate_runtime_assets.py`를
실행해 포함된 모델의 무결성을 확인합니다.
