# Dataset inventory

2026-08-24에 세 원본 폴더를 읽기 전용으로 조사한 결과입니다. 데이터는 새
저장소로 복제하지 않았으며 아래 경로는 마이그레이션 근거를 위한 원본 위치
표현입니다.

| ID | 원본 | 규모 | 권장 단계 | 비고 |
| --- | --- | ---: | --- | --- |
| `box-seg-rf-v3` | main workspace의 Roboflow YOLO segmentation | 18,517 images | 검토 후 processed | split 및 중복 검사 필요 |
| `box-coco-source` | main workspace의 COCO dataset | 약 16,308 images | raw/interim | v3 파생 관계 확인 필요 |
| `field-01` | main workspace의 현장 캡처 | 888 images | raw | QR·주소·얼굴 검수 필요 |
| `box-seg-rf-v4` | Loading 폴더의 segmentation | 3,535 images | processed 후보 | 원본 라이선스 메타데이터 유지 |
| `box-obb-v4` | Loading 폴더의 OBB 변환본 | 3,535 images | processed derivative | polygon에서 OBB로 변환 |
| `obb-upload-chunks` | Loading 폴더의 업로드 청크 | 3,535 images | export only | OBB v4와 중복이므로 정본 제외 |
| `dataset-archive` | `dataset.zip` | 약 2.1 GB | cold archive | 압축 해제본과 중복 여부 확인 후 체크섬 보존 |

## 정리 순서

1. 원본은 `data/local/raw`에 읽기 전용으로 배치합니다.
2. 이미지 해시로 데이터셋 간 중복과 train/val/test 누수를 검사합니다.
3. 개인정보와 팀 외부 저작물의 공개 가능 여부를 검수합니다.
4. 변환 스크립트와 seed를 기록해 `data/local/processed/<version>`을 만듭니다.
5. 최종 이미지 수, 클래스, split, 출처, 라이선스와 디렉터리 체크섬을 이
   매니페스트에 추가합니다.

현재 표의 수치는 파일 조사 기준이며 학습 정본을 선언한 값은 아닙니다.
