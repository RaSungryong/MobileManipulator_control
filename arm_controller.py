#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
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
    ArmController
    =============
    - Subscribe /robot_pose (for pose scan only)
    - Execute scan_points from TaskExecutor
    - Support:
        - pose control
        - joint control
    """

    def __init__(self):
        self.busy = False
        self.current_pose_msg = None
        self.goals_queue = []

        # ---------- MoveIt ----------
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.core.is_initialized():
            rospy.init_node('arm_controller', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        self.ee_link = self.arm.get_end_effector_link()

        self.arm.set_goal_joint_tolerance(0.01)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(5.0)

        # ---------- Home ----------
        self.home_joint_positions = [
            -1.5708, -1.5708, 1.5708,
            -1.5708, -1.5708, 0.0
        ]

        rospy.loginfo("[Arm] Move to home")
        self.move_to_home()

        # ---------- ROS ----------
        rospy.Subscriber('/robot_pose', Pose2DWithFlag, self.pose_cb, queue_size=1)
        self.done_pub = rospy.Publisher('/scan_finished', Bool, queue_size=1)

        rospy.loginfo("[ArmController] Ready")

    # --------------------------------------------------
    # robot_pose cache (pose scan only)
    # --------------------------------------------------
    def pose_cb(self, msg):
        self.current_pose_msg = msg

    # --------------------------------------------------
    # MAIN ENTRY
    # --------------------------------------------------
    def execute_scan_points(self, scan_points):

        if self.busy:
            rospy.logwarn("[Arm] Busy, ignore scan request")
            return

        if not scan_points:
            rospy.logwarn("[Arm] Empty scan_points")
            self.publish_done()
            return

        # pose scan requires robot_pose
        if any(p["mode"] == "pose" for p in scan_points):
            if self.current_pose_msg is None:
                rospy.logerr("[Arm] pose scan requires /robot_pose")
                return

        self.busy = True

        try:
            self.goals_queue = self._build_goals(scan_points)

            if not self.goals_queue:
                rospy.logwarn("[Arm] No valid goals")
                self.publish_done()
                return

            self._execute_goals()

        except Exception as e:
            rospy.logerr(f"[Arm] Scan exception: {e}")

        finally:
            rospy.loginfo("[Arm] Scan finished → home")
            self.move_to_home()
            self.goals_queue = []
            self.busy = False

    # --------------------------------------------------
    # BUILD GOALS
    # --------------------------------------------------
    def _build_goals(self, scan_points):
        goals = []

        pose_points = []
        for p in scan_points:
            if p["mode"] == "pose":
                pose_points.append(p)
            elif p["mode"] == "joint":
                goals.append(("joint", p["joints"], p.get("speed", 80)))

        # process pose points
        if pose_points:
            raw = []
            for i, p in enumerate(pose_points):
                raw.append({
                    "point_id": i,
                    "x": p["x"],
                    "y": p["y"],
                    "z": p["z"],
                    "rx": p["rx"],
                    "ry": p["ry"],
                    "rz": p["rz"],
                    "speed": p.get("speed", 80)
                })

            transformed = self.process_transforms(raw, self.current_pose_msg)
            for (pos, quat, speed) in transformed:
                goals.append(("pose", pos, quat, speed))

        return goals

    # --------------------------------------------------
    # EXECUTE
    # --------------------------------------------------
    def _execute_goals(self):
        rospy.loginfo(f"[Arm] Execute {len(self.goals_queue)} goals")

        for i, goal in enumerate(self.goals_queue):

            if goal[0] == "pose":
                _, pos, quat, speed = goal
                self._execute_pose_goal(pos, quat, speed)

            elif goal[0] == "joint":
                _, joints, speed = goal
                self._execute_joint_goal(joints, speed)

            rospy.sleep(0.3)

        self.publish_done()

    # --------------------------------------------------
    # POSE GOAL
    # --------------------------------------------------
    def _execute_pose_goal(self, pos, quat, speed):
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

        self.arm.set_max_velocity_scaling_factor(speed / 100.0)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target, self.ee_link)

        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    # --------------------------------------------------
    # JOINT GOAL
    # --------------------------------------------------
    def _execute_joint_goal(self, joints, speed):
        if len(joints) != 6:
            rospy.logerr("[Arm] joint goal must have 6 values")
            return

        self.arm.set_max_velocity_scaling_factor(speed / 100.0)
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(joints)

        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    # --------------------------------------------------
    # HOME
    # --------------------------------------------------
    def move_to_home(self):
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.arm.set_joint_value_target(self.home_joint_positions)
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    # --------------------------------------------------
    # TRANSFORM (unchanged)
    # --------------------------------------------------
    def create_homogeneous_transform(self, Rm, t):
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

    def process_transforms(self, goals, msg):
        transformed = []

        base_x, base_y = msg.x, msg.y
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

    # --------------------------------------------------
    # DONE
    # --------------------------------------------------
    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        rospy.loginfo("[Arm] scan_finished published")

    def is_busy(self):
        return self.busy


if __name__ == '__main__':
    controller = ArmController()
    rospy.spin()
