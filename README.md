Mobile Manipulator Scanning System
이 프로젝트는 AprilTag를 기반으로 한 모바일 로봇의 자율 주행과 MoveIt을 이용한 기계팔의 정밀 스캔 동작을 결합한 통합 제어 시스템입니다.

🏗 시스템 아키텍처
📦 모듈별 상세 설명
1. TaskManager (task_manager.py)
로봇이 수행할 스캔 작업의 순서와 대상 태그(WORK Tag)를 관리하는 논리 계층입니다.

주요 기능:

스캔 구역별(Zone B, C, D, E) 태그 리스트 생성 및 순서 정의.

특정 태그 ID가 스캔 대상인지 판별하는 유효성 검사.

핵심 함수:

_task_zone_b_c() / _task_zone_d_e(): 구역별 WORK 태그 시퀀스를 생성합니다.

외부 호출 API:

get_task(task_name): 작업 이름(예: "TASK1")에 해당하는 태그 리스트를 반환합니다.

list_tasks(): 정의된 모든 작업 목록을 반환합니다.

is_scan_tag(tag_id): 입력된 ID가 작업용 태그(100~147)인지 확인합니다.

2. MobileRobotController (mobile_robot_controller.py)
AprilTag를 활용한 토폴로지 맵 기반의 주행 및 시각적 정렬을 담당합니다.

주요 기능:

NavigationGraph를 이용한 태그 간 최단 경로 검색.

AprilTag 에지(Edge)를 활용한 정밀 방향 정렬(Visual Servoing).

주행 중 일시정지(Pause) 및 재개(Resume) 제어.

핵심 함수:

align_to_tag(tag_id): 카메라 데이터를 분석하여 태그와 로봇을 수평으로 정렬합니다.

move_to_visible_tag(target_tag, backward): 대상 태그가 보일 때까지 시각 피드백 주행을 수행합니다.

외부 호출 API:

move_to_tag(target_tag): 경로 계획부터 도달까지의 전 과정을 실행합니다.

set_pause(bool): 주행 루프를 즉시 멈추거나 다시 시작합니다.

publish_pose_for_arm(tag_id): 기계팔 동작에 필요한 현재 좌표 정보를 /robot_pose 토픽으로 발행합니다.

3. ArmController (arm_controller.py)
로봇의 현재 위치를 기반으로 기계팔의 스캔 궤적을 계산하고 실행합니다.

주요 기능:

세계 좌표계 데이터를 기계팔 기저 좌표계로 변환하는 Homogeneous Transform 수행.

CSV 파일(grid_path.csv)로부터 포인트별 스캔 목표 및 속도 데이터 로드.

작업 완료 후 반드시 Home Pose로 복귀하는 안전 메커니즘.

핵심 함수:

process_transforms(goals, msg): 로봇의 현재 x, y, theta를 기반으로 좌표 변환을 수행합니다.

execute_goals(): MoveIt을 호출하여 계산된 목표 지점들을 순차적으로 방문합니다.

외부 호출 API:

move_to_home(): 기계팔을 미리 정의된 안전 위치로 이동시킵니다.

is_busy(): 현재 기계팔이 동작 중인지 상태를 확인합니다.

Topic Subscription: /robot_pose 수신 시 자동으로 execute_scan이 시작됩니다.

4. TaskExecutor (task_executor.py)
시스템 전체의 상태 머신(FSM) 역할을 하며, 모바일 플랫폼과 기계팔 사이의 동기화를 제어합니다.

주요 기능:

IDLE → MOVING → SCANNING → SCAN_DONE으로 이어지는 상태 관리.

사용자 명령에 따른 작업 중단(STOP), 건너뛰기(SKIP), 작업 전환 처리.

핵심 함수:

_run_task(scan_tags): 작업 리스트의 모든 태그에 대해 주행과 스캔을 루프로 실행합니다.

_wait_for_scan(tag_id): 기계팔이 스캔을 마칠 때까지 실행 흐름을 대기시킵니다.

외부 제어 인터페이스 (Topic: /task_command):

TASK TASK1: TASK1 시퀀스 시작

STOP: 전체 시스템 즉시 정지

PAUSE / RESUME: 일시 정지 및 재개

SKIP: 현재 태그 스캔을 건너뛰고 다음 태그로 이동

💡 권장 설치 및 실행 방법
의존성 설치: dt-apriltags, moveit_commander, scipy 등이 필요합니다.

데이터 준비: /scripts/grid_path.csv 경로에 스캔 데이터가 있어야 합니다.

실행: rosrun mold_pkg task_executor.py 명령어를 통해 메인 루프를 구동합니다.
