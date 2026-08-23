# Hikrobot Camera

Hikrobot MVS SDK의 Python 바인딩을 사용해 ROS 2 압축/비압축 영상을 발행하는
패키지입니다. 통합 경로는 기본적으로 `/hik_camera/rgb/compressed`를 사용합니다.

Ubuntu 22.04에 MVS SDK를 설치한 후 Python 바인딩 디렉터리가 기본 위치와
다르면 다음 환경변수를 지정합니다.

```bash
export MILEMATE_MVS_IMPORT=/absolute/path/to/MvImport
ros2 launch hik_camera hik_camera.launch.py
```

MVS SDK와 해당 라이브러리는 저장소에 포함하지 않습니다. 장치가 없는 환경의
정적 빌드는 가능하지만 노드 실행과 프레임 수신 검증에는 SDK와 카메라가 모두
필요합니다.

SDK binding은 wildcard import를 요구하는 vendor API여서 해당 파일에 한정해
Ruff `F403/F405`를 제외했습니다. SDK 버전과 재배포 조건을 확인하기 전에는
vendor binding을 공개 배포물에 포함하지 않습니다.
