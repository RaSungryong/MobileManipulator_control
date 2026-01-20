#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool, String
from enum import Enum, auto
from typing import List, Optional

from mobile_robot_controller import MobileRobotController, TagDatabase, NavigationGraph
from arm_controller import ArmController
from task_manager import TaskManager


# ============================================================
# STATE
# ============================================================
class MobileManipulatorState(Enum):
    IDLE = auto()
    MOVING = auto()
    ARRIVED = auto()
    SCANNING = auto()
    SCAN_DONE = auto()
    ERROR = auto()


# ============================================================
# TASK EXECUTOR (FINAL – STABLE)
# ============================================================
class MobileManipulatorTaskExecutor:

    # --------------------------------------------------
    # INIT
    # --------------------------------------------------
    def __init__(self):

        rospy.init_node("mobile_manipulator_system")

        # ---------- Core ----------
        self.tag_db = TagDatabase()
        self.nav_graph = NavigationGraph(self.tag_db)

        self.mobile = MobileRobotController(self.tag_db, self.nav_graph)
        self.arm = ArmController(
            csv_path="/home/lcl/mold_ws/src/mold_pkg/scripts/grid_path.csv"
        )
        self.task_manager = TaskManager()

        # ---------- State ----------
        self.state = MobileManipulatorState.IDLE
        self.scan_done = False

        # ---------- Task control ----------
        self._current_task: Optional[List[int]] = None
        self._next_task: Optional[List[int]] = None

        self._stop_requested = False
        self._pause_requested = False
        self._skip_requested = False
        self._task_switch_requested = False

        # ---------- ROS ----------
        rospy.Subscriber('/scan_finished', Bool, self._scan_done_cb, queue_size=1)
        rospy.Subscriber('/task_command', String, self._command_cb, queue_size=10)

        rospy.loginfo("[Executor] System ready (IDLE)")

    # --------------------------------------------------
    # COMMAND CALLBACK
    # --------------------------------------------------
    def _command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        rospy.logwarn(f"[TASK COMMAND] {cmd}")

        if cmd == "STOP":
            self._stop_requested = True
            self._pause_requested = False
            self.mobile.set_pause(False)

        elif cmd == "PAUSE":
            self._pause_requested = True
            self.mobile.set_pause(True)

        elif cmd == "RESUME":
            self._pause_requested = False
            self.mobile.set_pause(False)

        elif cmd == "SKIP":
            self._skip_requested = True

        elif cmd.startswith("TASK"):
            parts = cmd.split()
            if len(parts) == 2:
                task = self.task_manager.get_task(parts[1])
                if task:
                    self.request_new_task(task)
                else:
                    rospy.logerr(f"[TASK] Unknown task {parts[1]}")

        elif cmd == "STATE":
            rospy.loginfo(f"[STATE] {self.state.name}")

    # --------------------------------------------------
    # TASK SWITCH
    # --------------------------------------------------
    def request_new_task(self, task: List[int]):
        rospy.logwarn("[TASK] New task requested")

        if self._current_task is None:
            self._current_task = task
        else:
            self._next_task = task
            self._task_switch_requested = True
            self._stop_requested = True

    # --------------------------------------------------
    # SCAN FINISHED
    # --------------------------------------------------
    def _scan_done_cb(self, msg: Bool):
        if msg.data:
            self.scan_done = True

    # --------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------
    def run(self):
        rate = rospy.Rate(5)
        rospy.loginfo("[Executor] Main loop started")

        while not rospy.is_shutdown():

            if self._current_task is None:
                rate.sleep()
                continue

            self._run_task(self._current_task)

            if self._task_switch_requested:
                self._task_switch_requested = False
                self._current_task = self._next_task
                self._next_task = None
            else:
                self._current_task = None

            rate.sleep()

    # --------------------------------------------------
    # TASK EXECUTION
    # --------------------------------------------------
    def _run_task(self, scan_tags: List[int]):

        self._reset_flags()
        rospy.loginfo(f"[TASK] Start task: {scan_tags}")

        for tag_id in scan_tags:

            # ---------- STOP ----------
            if self._stop_requested:
                rospy.logwarn("[TASK] Task stopped")
                break

            # ---------- PAUSE (BLOCKING) ----------
            while self._pause_requested and not rospy.is_shutdown():
                rospy.loginfo_throttle(1.0, "[TASK] Paused")
                rospy.sleep(0.1)

            # ---------- SKIP ----------
            if self._skip_requested:
                rospy.logwarn(f"[TASK] Skip tag {tag_id}")
                self._skip_requested = False
                continue

            # ---------- MOVE ----------
            self.state = MobileManipulatorState.MOVING
            if not self.mobile.move_to_tag(tag_id):
                self.state = MobileManipulatorState.ERROR
                continue

            self.state = MobileManipulatorState.ARRIVED

            # ---------- SCAN ----------
            self.state = MobileManipulatorState.SCANNING
            self.mobile.publish_pose_for_arm(tag_id)

            if not self._wait_for_scan(tag_id):
                self.state = MobileManipulatorState.ERROR
                continue

            self.state = MobileManipulatorState.SCAN_DONE

        self.state = MobileManipulatorState.IDLE
        rospy.loginfo("[TASK] Task finished")

    # --------------------------------------------------
    # UTILS
    # --------------------------------------------------
    def _wait_for_scan(self, tag_id, timeout=None) -> bool:
        self.scan_done = False
        start = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.scan_done:
                return True

            if self._stop_requested:
                return False

            if timeout and (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logerr(f"[TASK] Scan timeout at tag {tag_id}")
                return False

            rate.sleep()

        return False

    def _reset_flags(self):
        self._stop_requested = False
        self._pause_requested = False
        self._skip_requested = False


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    executor = MobileManipulatorTaskExecutor()
    executor.run()
