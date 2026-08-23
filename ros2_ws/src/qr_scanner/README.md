# QR Scanner Utilities

QR 생성기와 ROI 기반 독립 scanner를 제공하는 선택 패키지입니다. 핵심 플랫폼
통합 경로에서는 `parcel_perception`이 OBB 후보 내부의 QR을 직접 읽으므로 이
노드를 함께 실행할 필요가 없습니다.

독립 scanner의 과거 기본 입력은 `/box_detector/roi_image`이며 현재 통합
launch에는 이 토픽 publisher가 없습니다. 사용할 때 실제 영상 토픽으로
재지정합니다.

```bash
ros2 run qr_scanner qr_scanner_node --ros-args \
  -p image_topic:=/your/roi/image_topic
```

`pyzbar`는 OS의 `libzbar0`가 필요합니다. 생성 QR과 스캔 로그에는 배송 정보나
개인정보를 넣지 말고, 출력 경로는 `MILEMATE_QR_OUTPUT_DIR`로 관리합니다.
