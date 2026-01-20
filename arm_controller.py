#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Bool
from robot_msgs.msg import Pose2DWithFlag
from geometry_msgs.msg import PoseStamped

import moveit_commander


GROUP_NAME_ARM = 'arm'
REFERENCE_FRAME = 'world'


class ArmController:
    """
    안정화된 기계팔 컨트롤러 (Concurrency-safe)

    핵심 보장:
    - 동시에 하나의 scan만 실행
    - busy 상태에서 들어오는 모든 scan 요청 무시
    - MoveIt goal 중첩 방지
    - scan 종료 후 반드시 Home Pose 복귀
    """

    # ---------------------------------------------------------
    # 초기화
    # ---------------------------------------------------------
    def __init__(self, csv_path):
        self.csv_path = csv_path

        # ---------- 상태 ----------
        self.busy = False
        self.current_msg = None
        self.goals_queue = []

        # ---------- ROS / MoveIt ----------
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.core.is_initialized():
            rospy.init_node('arm_controller', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        self.end_effector_link = self.arm.get_end_effector_link()

        self.arm.set_goal_joint_tolerance(0.01)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(5.0)

        # ---------- Home Pose 정의 ----------
        self.home_joint_positions = [
            -1.5708, -1.5708, 1.5708,
            -1.5708, -1.5708, 0
        ]

        # ---------- 초기 Home 이동 ----------
        rospy.loginfo("[Arm] 초기 Home Pose로 이동")
        self.move_to_home()

        # ---------- ROS ----------
        self.pose_sub = rospy.Subscriber('/robot_pose', Pose2DWithFlag, self.pose_cb)
        self.done_pub = rospy.Publisher('/scan_finished', Bool, queue_size=1)

        rospy.loginfo("[ArmController] 준비 완료")

    # ---------------------------------------------------------
    # robot_pose 콜백 (HARD GATE)
    # ---------------------------------------------------------
    def pose_cb(self, msg):
        if not msg.flag:
            return

        if self.busy:
            rospy.logwarn("[Arm] Busy 상태 - 새로운 scan 요청 무시")
            return

        rospy.loginfo(f"[Arm] 로봇 위치 수신 | tag {msg.id}")
        self.execute_scan(msg)

    # ---------------------------------------------------------
    # 스캔 실행 진입점
    # ---------------------------------------------------------
    def execute_scan(self, msg: Pose2DWithFlag):

        if self.busy:
            rospy.logwarn("[Arm] execute_scan 호출됐으나 이미 busy")
            return

        self.busy = True
        self.current_msg = msg

        try:
            raw_goals = self.read_and_filter_csv(msg.id)
            self.goals_queue = self.process_transforms(raw_goals, msg)

            if not self.goals_queue:
                rospy.logwarn(f"[Arm] tag {msg.id} 에 대한 스캔 포인트 없음")
                self.publish_done()
                return

            self.execute_goals()

        except Exception as e:
            rospy.logerr(f"[Arm] Scan 실행 중 예외 발생: {e}")

        finally:
            # ⭐ 어떤 경우에도 Home 복귀 + 상태 정리
            rospy.loginfo("[Arm] Scan 종료 → Home Pose 복귀")
            self.move_to_home()

            self.goals_queue = []
            self.busy = False

    # ---------------------------------------------------------
    # 스캔 포인트 실행
    # ---------------------------------------------------------
    def execute_goals(self):
        rospy.loginfo(f"[Arm] 총 {len(self.goals_queue)} 개 스캔 포인트 실행")

        for i, (pos, quat, speed) in enumerate(self.goals_queue):

            target = PoseStamped()
            target.header.frame_id = REFERENCE_FRAME
            target.header.stamp = rospy.Time.now()

            target.pose.position.x = pos[0]
            target.pose.position.y = pos[1]
            target.pose.position.z = pos[2]

            target.pose.orientation.x = quat[0]
            target.pose.orientation.y = quat[1]
            target.pose.orientation.z = quat[2]
            target.pose.orientation.w = quat[3]

            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(target, self.end_effector_link)

            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()

            if success:
                rospy.loginfo(f"[Arm] 포인트 {i+1} 도달 성공")
                rospy.sleep(0.5)
            else:
                rospy.logerr(f"[Arm] 포인트 {i+1} 실패 → 스캔 중단")
                break

        self.publish_done()

    # ---------------------------------------------------------
    # Home Pose 이동 (⭐ 신규 핵심 기능)
    # ---------------------------------------------------------
    def move_to_home(self):
        """
        기계팔을 안전한 Home Pose로 이동
        - 언제 호출해도 안전
        - busy 여부와 무관 (내부에서 stop 처리)
        """
        try:
            self.arm.stop()
            self.arm.clear_pose_targets()
            self.arm.set_joint_value_target(self.home_joint_positions)
            self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            rospy.loginfo("[Arm] Home Pose 도착")
        except Exception as e:
            rospy.logerr(f"[Arm] Home Pose 이동 실패: {e}")

    # ---------------------------------------------------------
    # CSV 로드
    # ---------------------------------------------------------
    def read_and_filter_csv(self, target_group_id):
        data = []
        try:
            with open(self.csv_path, 'r', encoding='utf-8-sig') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if int(row['group_id']) == target_group_id:
                        data.append({
                            'point_id': int(row['point_id']),
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'z': float(row['z']),
                            'rx': float(row['rx']),
                            'ry': float(row['ry']),
                            'rz': float(row['rz']),
                            'speed': float(row.get('speed', 80))
                        })
            data.sort(key=lambda x: x['point_id'])
        except Exception as e:
            rospy.logerr(f"[Arm] CSV 읽기 오류: {e}")
        return data

    # ---------------------------------------------------------
    # 좌표 변환
    # ---------------------------------------------------------
    def create_homogeneous_transform(self, Rm, t):
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

    def process_transforms(self, goals, msg):
        transformed = []

        base_x = msg.x
        base_y = msg.y
        base_z = -0.835

        for g in goals:
            dx = g['x'] - base_x
            dy = g['y'] - base_y
            dz = g['z'] - base_z

            dx, dy, dz = dx, -dy, -dz

            R_base = R.from_euler('y', 180, degrees=True)
            R_yaw = R.from_euler('z', msg.theta, degrees=True)
            R_mb = (R_base * R_yaw).as_matrix()

            T_mb = self.create_homogeneous_transform(R_mb, [base_x, base_y, 0.18])
            T_ba = self.create_homogeneous_transform(
                R.from_euler('z', 180, degrees=True).as_matrix(),
                [0, 0, -1.02]
            )

            T = T_mb @ T_ba
            R_ab = R.from_matrix(T[:3, :3])

            r_tag = R.from_euler('xyz', [g['rx'], g['ry'], g['rz']], degrees=True)
            r_goal = R_ab.inv() * r_tag

            transformed.append(
                (np.array([dx, dy, dz]), r_goal.as_quat(), g['speed'])
            )

        return transformed

    # ---------------------------------------------------------
    # 스캔 완료 신호
    # ---------------------------------------------------------
    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        rospy.loginfo("[Arm] scan_finished 발행")

    # ---------------------------------------------------------
    # 상태 조회
    # ---------------------------------------------------------
    def is_busy(self):
        return self.busy
