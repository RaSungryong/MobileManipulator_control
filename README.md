# 🤖 Mobile Manipulator Autonomous Scanning System

본 프로젝트는 **AprilTag 기반 토폴로지 내비게이션**과 **MoveIt! 매니퓰레이션**을 결합하여 지도가 없는 환경에서도 자율 주행 및 정밀 스캔 작업을 수행하는 모바일 매니퓰레이터 시스템입니다.

---

## 🏗️ 시스템 아키텍처

이 시스템은 계층화된 4개의 모듈로 구성되어 있어 각 기능의 독립성과 확장성을 보장합니다.

| 레이어 | 모듈명 | 파일명 | 주요 역할 |
| :--- | :--- | :--- | :--- |
| **작업 관리** | Task Manager | `task_manager.py` | 스캔 대상 태그 및 작업 시퀀스 정의 |
| **시스템 통합** | Task Executor | `task_executor.py` | FSM 기반 주행-스캔 프로세스 동기화 및 명령 처리 |
| **주행 제어** | Mobile Controller | `mobile_robot_controller.py` | 토폴로지 맵 기반 경로 계획 및 시각적 정렬 주행 |
| **암 제어** | Arm Controller | `arm_controller.py` | 실시간 좌표 변환 기반 매니퓰레이터 궤적 실행 |



---

## 🚀 주요 모듈 상세 설명

### 1. Task Management (`task_manager.py`)
로봇이 수행해야 할 구역별 스캔 순서를 관리합니다.
* **Zone 정의**: Zone B, C, D, E 등 논리적 구역별로 WORK 태그 ID 시퀀스를 정의합니다.
* **최적화**: 주행 동선을 고려하여 태그 스캔 순서(예: 상단에서 하단으로)를 리스트 형태로 제공합니다.

### 2. Mobile Navigation (`mobile_robot_controller.py`)
AprilTag를 이정표로 사용하여 정밀한 위치 추정 및 주행을 수행합니다.
* **Topology Graph**: AprilTag 간의 연결 관계를 정의한 `NavigationGraph`를 구축하고 BFS(Breadth-First Search) 알고리즘으로 최단 경로를 탐색합니다.
* **Visual Servoing**: 
    * `align_to_tag`: 태그의 모서리 데이터를 분석하여 로봇을 정면으로 정렬합니다.
    * `move_to_visible_tag`: 카메라 피드백을 통해 목표 태그까지의 거리를 제어하며 정밀하게 접근합니다.

### 3. Manipulation Control (`arm_controller.py`)
로봇 베이스의 위치 변화를 실시간으로 보정하여 목표 지점을 정밀 타격합니다.
* **Coordinate Transformation**: 세계 좌표계의 목표 포인트를 로봇 오도메트리와 결합하여 기계팔 좌표계로 변환합니다.
  $$T = T_{mb} \times T_{ba}$$
  *(여기서 $T_{mb}$는 베이스 좌표계 변환, $T_{ba}$는 베이스-암 사이의 변환입니다.)*
* **Concurrency Safety**: `busy` 플래그를 통해 기계팔이 동작 중일 때 새로운 명령이 중첩되지 않도록 보호합니다.

### 4. Task Execution (`task_executor.py`)
시스템의 전체 상태 변화를 관리하는 중앙 제어 장치입니다.
* **State Machine**: `IDLE` ➔ `MOVING` ➔ `ARRIVED` ➔ `SCANNING` ➔ `SCAN_DONE` 순으로 상태를 전이하며 전체 루프를 제어합니다.
* **Command Dispatcher**: `/task_command` 토픽을 통해 외부 사용자의 명령을 해석하고 즉각적인 조치(멈춤, 일시정지, 건너뛰기 등)를 취합니다.

---

## 🕹️ 제어 인터페이스 (API)

### 사용자 명령 (Topic: `/task_command`)
| 명령 | 설명 |
| :--- | :--- |
| `TASK TASK1` | Zone B, C 구역 스캔 작업 시작 |
| `TASK TASK2` | Zone D, E 구역 스캔 작업 시작 |
| `STOP` | 현재 수행 중인 모든 작업을 즉각 중단 |
| `PAUSE / RESUME` | 주행 및 스캔 프로세스 일시 정지 및 재개 |
| `SKIP` | 현재 태그를 건너뛰고 다음 목표로 이동 |

### 데이터 흐름도


---

## 🛠️ 설치 및 실행 방법

### 1. 의존성 설치
```bash
pip install dt-apriltags scipy numpy
sudo apt-get install ros-<distro>-moveit-commander
