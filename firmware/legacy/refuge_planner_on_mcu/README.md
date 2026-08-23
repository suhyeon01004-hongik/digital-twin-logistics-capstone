# Legacy: Refuge Planner on MCU

초기 단계에서 택배 DB와 순환 계획까지 Arduino 내부에서 수행하려던 실험
버전입니다. 현재 정본 구조는 ROS 2/MATLAB이 상위 계획을 맡고
`firmware/refuge_belt_controller`가 저수준 벨트 제어만 수행합니다.

이 폴더는 설계 이력 확인용이며 현재 ROS 2 프로토콜과의 호환성, 컴파일,
실기 동작을 보장하지 않습니다. 운용 펌웨어로 업로드하지 마세요.
