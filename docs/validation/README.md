# 검증 범위

## 로컬 자동 검사

| 명령 | 확인 범위 |
| --- | --- |
| `python3 -m unittest discover -s tests -v` | 인식 기하 계산, 설치 트리 경로 탐색, 시리얼 reader 종료 |
| `python3 scripts/validate_workspace.py` | Python 구문, ROS XML, 경로 이식성, 생성물·대용량 파일 정책 |
| `python3 scripts/validate_protocols.py` | ROS↔Arduino 명령·응답·baud와 펌웨어 안전 guard |
| `python3 scripts/validate_matlab_static.py` | `.slx` ZIP/XML 무결성, Model UUID, MATLAB 파일과 과거 결과 일관성 |
| `python3 scripts/validate_runtime_assets.py` | OBB 모델 크기·SHA-256과 Simulink 파일 존재 |
| `python3 scripts/validate_evidence.py` | 취업용 증빙 manifest SHA-256, CSV 행 수, 절대경로 누출 |

`validate_workspace.py`는 Git에서 제외한 모델이 없어도 경고만 냅니다.
`validate_runtime_assets.py`는 실제 실행 준비 검사이므로 모델이 없거나 체크섬이
다르면 실패합니다.

## Ubuntu 통합 검사

`bash scripts/validate_ubuntu.sh`는 위 검사에 `rosdep install`과 ROS 2 Humble
`colcon --symlink-install` 빌드를 추가합니다. 네트워크와 ROS 2 설치가 필요합니다.

## MATLAB 실행 회귀

정적 검사와 MATLAB 실행 회귀는 다른 판정입니다. MATLAB/Simulink가 준비된
환경에서 먼저 quick, 그다음 standard를 실행합니다.

```bash
python3 scripts/run_matlab_validation.py --profile quick --suite all
python3 scripts/run_matlab_validation.py --profile standard --suite all
```

`manual`은 적재·순환·목표 하차 상태머신을, `e2e`는 측정값 기반 적재부터 F1 카메라
정렬과 대기구역 전달까지 확인합니다. 둘은 독립 MATLAB 프로세스로 실행되며 모든
case의 `success=1`, 충돌/회전 0, 프로세스 종료 코드 0을 통과 기준으로 삼습니다.
실패 시 생성된 CSV·MAT·summary·progress와 `matlab-*.log`를 함께 보존합니다.

## 별도 실행이 필요한 검사

- Arduino Mega 실제 컴파일·업로드
- Hikrobot MVS SDK import와 카메라 프레임 수신
- MATLAB/Simulink 회귀 suite의 수치 결과
- 모터 방향, encoder 부호, ToF 채널·보정값과 E-stop
- 플랫폼 물리 원점, 층 높이, 푸셔 거리와 기구 간섭
- 상차→순환→목표 하차 전체 E2E

정적 검사 통과를 실장비 동작 보증으로 표현하지 않습니다. 실기 순서는
`docs/operations/runtime-checklist.md`를 따릅니다.
