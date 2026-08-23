# 리팩터링 현황과 남은 작업

## 이번 단계에서 완료

- 세 원본 폴더에서 ROS 2, MATLAB, perception과 firmware 정본 통합
- build/install/log, 가상환경, 압축 백업, 데이터 복제본과 날짜별 소스 제외
- 플랫폼 Serial transport 분리와 연결 종료·재연결 수명주기 수정
- 모델·MATLAB runtime path의 source/install 공용 탐색 처리
- Ultralytics 추론 인자 수정과 원본 런타임 버전 고정
- 인식 기하 함수 분리 및 순수 단위 테스트 추가
- launch parameter type 고정과 perception 없는 dry-run 추가
- 벨트 E-stop/fault latch, 리프트 층 범위 안전 guard 추가
- 모델·Simulink 자산 검사, MATLAB 정적 검사, 프로토콜 검사 확장
- README, 데이터·모델 정책, 실장비 체크리스트 정비

## 실장비 회귀 전에 하지 않은 대규모 분리

`digital_twin_compare.py`, `supervisor.py`, `platform_load_manager.py`는 각각 큰
상태머신과 하드웨어 보정값을 포함합니다. MATLAB/ROS 기준 실행 결과가 없는
상태에서 파일 구조만 크게 바꾸면 동작 차이를 추적하기 어렵기 때문에 이번에는
확인 가능한 결함만 작게 수정했습니다.

## 다음 작업 순서

1. 플랫폼 푸셔를 non-blocking 상태머신으로 바꾸고 공통 abort/E-stop 입력을
   추가해 ROS stop이 실제 motion을 중단하도록 합니다.
2. Ubuntu 22.04에서 `validate_ubuntu.sh`와 Arduino Mega compile을 통과시킵니다.
3. 과거 MATLAB baseline 실패 1건을 재현하고 원인을 기록한 뒤 새로운 기준
   결과를 고정합니다.
4. 축별 bench test로 핀, 방향, encoder/ToF, E-stop과 보정값을 확정합니다.
5. 적재·하차 상태 전이를 ROS I/O에서 분리하고 시나리오 단위 테스트를 늘립니다.
6. `digital_twin_compare.py`를 상태 저장소, 계획기, MATLAB adapter, ROS node로
   작은 단위씩 분리합니다.
7. Arduino 명령을 공용 schema로 정의해 문자열 signature 검사를 구조 검사로
   대체합니다.
8. 데이터 해시 기반 중복·split leakage 검사와 OBB mAP·각도·latency 평가를
   추가합니다.

각 구조 변경은 고정된 회귀 결과와 비교해 한 단계씩 적용합니다.
