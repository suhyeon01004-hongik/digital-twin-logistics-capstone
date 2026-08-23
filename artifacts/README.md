# 로컬 실행 산출물

이 디렉터리의 바이너리와 실행 결과는 Git에서 제외합니다. 현재 OBB 추론 모델의
기본 위치는 다음과 같습니다.

```text
artifacts/models/box_obb_s_512/best.pt
```

다른 위치를 사용할 때는 환경변수를 지정합니다.

```bash
export MILEMATE_MODEL_PATH=/absolute/path/to/best.pt
python3 scripts/validate_runtime_assets.py
```

ONNX·TensorRT 엔진과 학습 run은 GPU, CUDA, TensorRT 및 라이브러리 버전에
영향을 받으므로 소스와 함께 커밋하지 않습니다. 배포가 필요하면 Release 또는
별도 모델 저장소에 체크섬과 실행 환경을 함께 기록합니다.
