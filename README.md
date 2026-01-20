# 🤖 Mobile Manipulator Autonomous Scanning System

이 프로젝트는 **AprilTag** 기반의 토폴로지 네비게이션과 **MoveIt**을 이용한 매니퓰레이터 제어를 결합한 통합 자동화 시스템입니다. 로봇은 환경 내의 태그를 이정표로 삼아 자율 주행하며, 각 작업 포인트에서 정밀한 스캔 알고리즘을 수행합니다.



---

## 🏗️ 시스템 아키텍처

본 시스템은 높은 응집도와 낮은 결합도를 위해 4개의 핵심 모듈로 설계되었습니다:

| 모듈명 | 역할 | 주요 기술 |
| :--- | :--- | :--- |
| **Task Manager** | 작업 시퀀스 정의 | 태그 데이터베이스 관리 |
| **Mobile Controller** | 주행 및 정밀 정렬 | BFS 경로 탐색, 시각적 서보 제어 |
| **Arm Controller** | 매니퓰레이터 제어 | MoveIt!, 좌표 변환 알고리즘 |
| **Task Executor** | 시스템 통합 관리 | 유한 상태 머신 (FSM) |

---

## 📦 모듈별 상세 설명

### 1. Task Manager (`task_manager.py`)
로봇이 수행해야 할 "할 일 목록"을 정의합니다.
* **작업 정의**: 구역별(Zone B~E) WORK 태그의 스캔 순서를 리스트 형태로 생성합니다.
* **API**: 
    * `get_task(task_name)`: 특정 작업의 태그 시퀀스를 반환합니다.
    * `is_scan_tag(tag_id)`: 특정 태그가 스캔 작업용인지 검증합니다.

### 2. Mobile Robot Controller (`mobile_robot_controller.py`)
AprilTag를 활용하여 지도 없이 주행하고 정밀하게 위치를 잡습니다.
* **토폴로지 내비게이션**: `NavigationGraph`와 BFS 알고리즘을 통해 태그 간 최단 경로를 탐색합니다.
* **시각적 정렬**: 태그의 모서리 데이터를 기반으로 로봇을 정면으로 정렬하는 `align_to_tag` 기능을 수행합니다.
* **API**: 
    * `move_to_tag(target_tag)`: 목표 지점까지의 주행 프로세스를 실행합니다.

### 3. Arm Controller (`arm_controller.py`)
로봇의 현재 위치를 실시간으로 반영하여 스캔 동작을 수행합니다.
* **좌표 변환**: 세계 좌표계의 포인트를 기계팔 기저 좌표계로 변환하기 위해 다음과 같은 행렬 연산을 수행합니다.
  $$T = T_{mb} \times T_{ba}$$
* **안전 메커니즘**: 모든 작업 완료 또는 오류 발생 시 반드시 `move_to_home`으로 복귀합니다.
* **API**: 
    * `is_busy()`: 현재 매니퓰레이터의 동작 상태를 반환합니다.

### 4. Task Executor (`task_executor.py`)
전체 시스템의 흐름을 제어하는 "브레인"입니다.
* **상태 머신**: `IDLE`, `MOVING`, `SCANNING` 등의 상태를 관리하며 동기화를 제어합니다.



---

## 🚀 사용 방법

### 1. 시스템 실행
메인 실행 노드를 구동합니다.
```bash
rosrun your_package_name task_executor.py
