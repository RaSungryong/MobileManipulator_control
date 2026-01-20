# 🤖 Mobile Manipulator Scanning System 상세 명세

본 시스템은 AprilTag 기반의 자율 주행과 MoveIt!을 이용한 기계팔 제어를 통합하여 자동화된 스캔 작업을 수행합니다.

## 1. TaskManager (`task_manager.py`)

이 모듈은 로봇이 수행해야 할 스캔 작업의 논리적 순서와 대상 태그를 정의하는 역할을 합니다.

- **주요 기능 및 특징**:
    - 스캔 작업(WORK)에 사용되는 AprilTag ID 리스트만 관리합니다.
    - 이동(Move), 피벗(Pivot), 도킹(Dock) 관련 태그는 포함하지 않습니다.
    - 정의된 태그의 순서는 실제 스캔이 수행되는 순서를 의미합니다.
- **주요 기능 함수**:
    - `_task_zone_b_c()`: Zone B(태그 111→100)와 Zone C(태그 112→123)의 스캔 시퀀스를 생성합니다.
    - `_task_zone_d_e()`: Zone D(태그 135→124)와 Zone E(태그 136→147)의 스캔 시퀀스를 생성합니다.
- **외부 호출 API**:
    - `get_task(task_name)`: 작업 이름(예: "TASK1")을 입력받아 해당 스캔 태그 ID 리스트를 반환합니다.
    - `list_tasks()`: 현재 정의된 모든 작업 이름 목록을 반환합니다.
    - `is_scan_tag(tag_id)`: 입력된 ID가 유효한 작업(WORK)용 태그인지 확인합니다 (100~147 범위).

---

## 2. MobileRobotController (`mobile_robot_controller.py`)

이 모듈은 AprilTag 데이터베이스와 토폴로지 그래프를 사용하여 로봇의 주행과 정밀 정렬을 담당합니다.

- **주요 기능 및 특징**:
    - AprilTag의 세계 좌표를 관리하는 `TagDatabase`를 포함합니다.
    - 태그 간의 연결 관계를 정의한 `NavigationGraph`를 사용하여 최단 경로를 탐색합니다.
    - 카메라 피드백을 이용한 시각적 서보 제어(Visual Servoing)로 정밀 정렬을 수행합니다.
- **주요 기능 함수**:
    - `pivot_90deg(direction)`: 로봇을 제자리에서 90도 회전시킵니다.
    - `align_to_tag(tag_id)`: AprilTag의 모서리 데이터를 분석하여 로봇을 태그와 수평으로 정렬합니다.
    - `move_to_visible_tag(target_tag, backward)`: 대상 태그가 보일 때까지 시각 피드백을 통해 정밀 접근합니다.
- **외부 호출 API**:
    - `move_to_tag(target_tag)`: 경로 계획부터 주행, 정렬까지 수행하여 특정 태그 위치로 로봇을 이동시킵니다.
    - `set_pause(pause)`: 주행 프로세스를 일시 정지하거나 다시 시작합니다.
    - `publish_pose_for_arm(tag_id)`: 현재 로봇의 오도메트리 좌표와 태그 ID를 기계팔 컨트롤러로 전송합니다.

---

## 3. ArmController (`arm_controller.py`)

MoveIt! 프레임워크를 기반으로 기계팔의 스캔 동작과 좌표계 변환을 처리합니다.

- **주요 기능 및 특징**:
    - 동시에 하나의 스캔만 실행되도록 보장하는 동시성 안전(Concurrency-safe) 메커니즘을 가집니다.
    - 스캔 종료 후 또는 오류 발생 시 항상 안전한 `Home Pose`로 복귀합니다.
    - CSV 파일(`grid_path.csv`)로부터 포인트별 좌표와 속도 데이터를 로드합니다.
- **주요 기능 함수**:
    - `process_transforms(goals, msg)`: 로봇의 현재 위치를 기반으로 월드 좌표계의 포인트를 기계팔 좌표계로 변환합니다.
    - `execute_goals()`: 계산된 각 스캔 포인트에 대해 기계팔 궤적을 계획하고 실행합니다.
    - `read_and_filter_csv(target_group_id)`: 특정 태그 ID에 할당된 스캔 포인트 데이터를 CSV에서 필터링하여 가져옵니다.
- **외부 호출 API**:
    - `move_to_home()`: 기계팔을 미리 정의된 안전 위치로 강제 이동시킵니다.
    - `is_busy()`: 현재 기계팔이 동작 중인지 상태를 확인합니다.
    - **Topic Subscription**: `/robot_pose` 토픽을 구독하여 위치 정보가 수신되면 자동으로 스캔 시퀀스를 시작합니다.

---

## 4. TaskExecutor (`task_executor.py`)

시스템의 최상위 계층으로, 모바일 베이스와 기계팔 간의 동기화 및 사용자 명령 처리를 담당하는 유한 상태 머신(FSM)입니다.

- **주요 기능 및 특징**:
    - `IDLE`, `MOVING`, `ARRIVED`, `SCANNING`, `SCAN_DONE`, `ERROR` 상태를 관리합니다.
    - 주행 중 정지(STOP), 일시정지(PAUSE), 작업 전환 등의 예외 상황을 처리합니다.
- **주요 기능 함수**:
    - `run()`: 시스템의 메인 루프를 실행하며 작업 할당 여부를 체크합니다.
    - `_run_task(scan_tags)`: 할당된 태그 시퀀스를 순회하며 주행과 스캔 과정을 반복 제어합니다.
    - `_wait_for_scan(tag_id)`: 기계팔로부터 스캔 완료 신호가 올 때까지 실행을 대기시킵니다.
- **외부 호출 API (Topic: `/task_command`)**:
    - `TASK [TASK_NAME]`: 지정된 이름의 작업을 시작합니다.
    - `STOP`: 모든 동작을 중단하고 작업을 초기화합니다.
    - `PAUSE / RESUME`: 전체 시스템 동작을 일시 정지하거나 재개합니다.
    - `SKIP`: 현재 진행 중인 태그 스캔을 건너뛰고 다음 태그로 이동합니다

---
