# MATLAB/Simulink 디지털 트윈

`digital_twin/`에는 현재 통합 후보 소스와 비교용 기준 모델만 남기고 날짜별
`before_*`, `wip_*` 복사본은 원본 보존 폴더에 유지했습니다.

| 파일 | 용도 | 현재 검증 상태 |
| --- | --- | --- |
| `milemate_digital_twin.slx` | 통합 모델 후보 | ZIP/XML 무결성만 확인 |
| `baseline_auto_parcel_loading_process.slx` | 변경 전 비교 기준 | ZIP/XML 무결성만 확인 |
| `run_parcel_final_e2e_suite.m` | 전체 회귀 진입점 | 현재 환경에서 미실행 |
| `run_parcel_manual_regression_suite.m` | 수동 알고리즘 회귀 | 현재 환경에서 미실행 |

두 `.slx`는 같은 Model UUID를 가진 동일 계보의 파생본입니다. 동시에 열어
수정하지 말고, 회귀 기준 모델은 읽기 전용으로 취급합니다.

과거 `matlab/tests/baseline/parcel_regression_summary_20260624_200629.txt`에서는
54개(적재율 80.7%) 적재가 통과한 뒤 첫 목표 P55가 36,000 step(설정상 360초)
동안 하차 완료되지 않고 `CIRCULATION WAIT P55`에 머물렀습니다. 충돌·회전 위험은
검출되지 않았지만 대기구역 도달, 피신, 재삽입도 발생하지 않았습니다. 첫 실패에서
suite가 중단되어 나머지 quick 시나리오는 실행되지 않았습니다.

이 기록은 현재 선택한 `parcel_manual_core_step.m`보다 이전 실행 결과입니다.
현재 코드의 실패를 뜻하지도, 현재 통과 기준을 뜻하지도 않으며 재현 대상으로만
사용합니다.

## 실행 전 준비

```bash
source scripts/milemate_env.sh
python3 scripts/validate_matlab_static.py
```

MATLAB 설치 후에는 저장소 루트에서 다음 순서로 실행합니다.

```bash
python3 scripts/run_matlab_validation.py --profile quick --suite all
python3 scripts/run_matlab_validation.py --profile standard --suite all
```

수동 회귀와 최종 E2E는 별도 MATLAB 프로세스로 실행되므로 한 suite의 실패가 다른
suite 실행을 막지 않습니다. 결과는 기본적으로 `matlab/tests/runs/<timestamp>/`에
저장되며 Git에는 포함하지 않습니다.

MATLAB에서 다음처럼 디렉터리를 연 뒤 패널을 시작합니다.

```matlab
cd('/absolute/path/to/milemate-digital-twin-logistics/matlab/digital_twin')
open_refuge_digital_twin_panel
```

저장소 외 위치를 사용하면 ROS 실행 전에 `MILEMATE_TWIN_DIR`을 지정합니다.
실행 검증에는 MATLAB/Simulink 라이선스와 ROS 연동 환경이 필요합니다. 정적
무결성 검사는 MATLAB 함수의 수치 결과나 Simulink 시뮬레이션 성공을 보장하지
않습니다.
