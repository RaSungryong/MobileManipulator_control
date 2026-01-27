#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
from std_msgs.msg import Bool, String
from enum import Enum, auto
from typing import List, Optional

from map_manager import MapManager
from task_manager import TaskManager          # ⭐ 新增
from robot_controller import RobotController
from arm_controller import ArmController
import utils


# ============================================================
# STATE ENUM
# ============================================================
class MobileManipulatorState(Enum):
    IDLE = auto()
    MOVING = auto()
    ARRIVED = auto()
    SCANNING = auto()
    SCAN_DONE = auto()
    ERROR = auto()


# ============================================================
# PATHS
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)

CONFIG_PATH = os.path.join(PKG_DIR, 'config', 'robot.yaml')
MAP_PATH = os.path.join(PKG_DIR, 'config', 'map.yaml')
TASK_DIR = os.path.join(PKG_DIR, 'task', 'csv')   # 多 CSV 目录
print(TASK_DIR)

# ============================================================
# TASK EXECUTOR
# ============================================================
class MobileManipulatorTaskExecutor:

    # --------------------------------------------------
    # INIT
    # --------------------------------------------------
    def __init__(self):

        rospy.init_node("mobile_manipulator_system")

        # ---------- Config ----------
        robot_config = utils.load_config(CONFIG_PATH)

        # ---------- Managers ----------
        self.map_mgr = MapManager(MAP_PATH)
        self.task_mgr = TaskManager(TASK_DIR)        # 新增

        # ---------- Controllers ----------
        self.mobile = RobotController(
            config=robot_config,
            map_manager=self.map_mgr
        )

        self.arm = ArmController()                    # 不再传 csv_path

        # ---------- State ----------
        self.state = MobileManipulatorState.IDLE
        self.scan_done = False

        # ---------- Task ----------
        self._current_task_name: Optional[str] = None
        self._current_task: Optional[List[dict]] = None
        self._stop_requested = False
        self._pause_requested = False

        # ---------- ROS ----------
        rospy.Subscriber(
            "/scan_finished",
            Bool,
            self._scan_done_cb,
            queue_size=1
        )
        rospy.Subscriber(
            "/task_command",
            String,
            self._command_cb,
            queue_size=10
        )

        rospy.loginfo("[Executor] System ready (IDLE)")


    # --------------------------------------------------
    # COMMAND CALLBACK
    # --------------------------------------------------
    def _command_cb(self, msg: String):
        cmd = msg.data.strip()
        rospy.logwarn(f"[TASK COMMAND] {cmd}")

        if cmd.upper() == "STOP":
            self._stop_requested = True
            self._pause_requested = False

        elif cmd.upper() == "PAUSE":
            self._pause_requested = True

        elif cmd.upper() == "RESUME":
            self._pause_requested = False

        elif cmd.upper().startswith("TASK"):
            # e.g. "TASK scan_task_A"
            parts = cmd.split()
            if len(parts) != 2:
                rospy.logerr("[TASK] Usage: TASK <task_name>")
                return

            task_name = parts[1]

            task = self.task_mgr.get_task(task_name)
            print(task)
            if not task:
                rospy.logerr(f"[TASK] Unknown task '{task_name}'")
                return

            self._current_task_name = task_name
            self._current_task = task

            rospy.loginfo(
                f"[TASK] Loaded task '{task_name}' "
                f"(steps={len(task)})"
            )

        elif cmd.upper() == "STATE":
            rospy.loginfo(f"[STATE] {self.state.name}")


    # --------------------------------------------------
    # SCAN FINISHED CALLBACK
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

            self._run_task(
                self._current_task_name,
                self._current_task
            )

            self._current_task_name = None
            self._current_task = None
            rate.sleep()


    # --------------------------------------------------
    # TASK EXECUTION
    # --------------------------------------------------
    def _run_task(self, task_name: str, task_items: List[dict]):
        print(task_items)

        self._reset_flags()
        rospy.loginfo(f"[TASK] Start task '{task_name}'")

        for item in task_items:

            if self._stop_requested:
                rospy.logwarn("[TASK] Task stopped")
                break

            while self._pause_requested and not rospy.is_shutdown():
                rospy.loginfo_throttle(1.0, "[TASK] Paused")
                rospy.sleep(0.1)

            tag_id = item["tag"]
            do_scan = item.get("scan", False)

            # ---------- MOVE ----------
            self.state = MobileManipulatorState.MOVING
            rospy.loginfo(f"[TASK] Moving to tag {tag_id}")

            ok = self.mobile.move_to_tag(tag_id)
            if not ok:
                rospy.logerr(f"[TASK] Failed to move to tag {tag_id}")
                self.state = MobileManipulatorState.ERROR
                return

            self.state = MobileManipulatorState.ARRIVED
            rospy.loginfo(f"[TASK] Arrived at tag {tag_id}")
            rospy.logwarn(f"[DEBUG] item={item}, do_scan={do_scan}")

            # ---------- SCAN ----------
            if do_scan:
                self.state = MobileManipulatorState.SCANNING
                rospy.loginfo(f"[TASK] Start scan at tag {tag_id}")

                scan_points = self.task_mgr.get_scan_points(
                    task_name,
                    tag_id
                )

                if not scan_points:
                    rospy.logwarn(
                        f"[TASK] No scan points for tag {tag_id}"
                    )
                    continue

                self.scan_done = False
                self.arm.execute_scan_points(scan_points)

                # ---- wait for scan_finished ----
                while not self.scan_done and not rospy.is_shutdown():
                    rospy.sleep(0.1)

                self.state = MobileManipulatorState.SCAN_DONE
                rospy.loginfo(f"[TASK] Scan finished at tag {tag_id}")

        self.state = MobileManipulatorState.IDLE
        rospy.loginfo("[TASK] Task finished")


    # --------------------------------------------------
    # UTILS
    # --------------------------------------------------
    def _reset_flags(self):
        self._stop_requested = False
        self._pause_requested = False
        self.scan_done = False


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    executor = MobileManipulatorTaskExecutor()
    executor.run()
