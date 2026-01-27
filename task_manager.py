#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import rospy
from collections import defaultdict
from typing import Dict, List


class TaskManager:
    """
    TaskManager
    ===========
    - Load multiple CSV files
    - Each CSV == one task
    - group_id == tag_id
    - Scan task detection rule:
        1) filename starts with 'scan_'  (FORCE scan)
        2) OR CSV header contains 'mode'
    - CSV without 'mode' will default to pose scan
    """

    def __init__(self, task_dir: str):
        self.task_dir = task_dir

        # task_name -> [{tag, scan}]
        self.tasks: Dict[str, List[dict]] = {}

        # task_name -> tag_id -> [scan points]
        self.scan_points: Dict[str, Dict[int, List[dict]]] = defaultdict(dict)

        rospy.loginfo(f"[TaskManager] Load tasks from {task_dir}")
        self._load_all_tasks()

    # --------------------------------------------------
    # LOAD ALL CSV TASKS
    # --------------------------------------------------
    def _load_all_tasks(self):
        if not os.path.isdir(self.task_dir):
            rospy.logerr(f"[TaskManager] Task dir not found: {self.task_dir}")
            return

        for fname in sorted(os.listdir(self.task_dir)):
            if not fname.endswith(".csv"):
                continue

            task_name = os.path.splitext(fname)[0]
            csv_path = os.path.join(self.task_dir, fname)

            rows = self._read_csv(csv_path)
            if not rows:
                rospy.logwarn(f"[TaskManager] Empty CSV: {csv_path}")
                continue

            # --------------------------------------------------
            # ⭐ SCAN TASK DETECTION RULE
            # --------------------------------------------------
            is_scan_task = (
                task_name.startswith("scan_")   # filename rule
                or "mode" in rows[0]             # header rule
            )

            rospy.loginfo(
                f"[TaskManager] Loading '{task_name}': "
                f"scan_task={is_scan_task}, headers={list(rows[0].keys())}"
            )

            if is_scan_task:
                self._build_scan_task(task_name, rows)
            else:
                self._build_move_only_task(task_name, rows)

            rospy.loginfo(
                f"[TaskManager] Task '{task_name}' loaded "
                f"(steps={len(self.tasks.get(task_name, []))})"
            )

    def _read_csv(self, csv_path: str) -> List[dict]:
        rows = []
        try:
            with open(csv_path, newline='', encoding="utf-8-sig") as f:
                reader = csv.DictReader(f)
                for r in reader:
                    rows.append(r)
        except Exception as e:
            rospy.logerr(f"[TaskManager] Failed to read {csv_path}: {e}")
        return rows

    # --------------------------------------------------
    # BUILD SCAN TASK
    # --------------------------------------------------
    def _build_scan_task(self, task_name: str, rows: List[dict]):
        task_steps = []
        scan_points_by_tag = defaultdict(list)

        # sort: order -> group_id -> point_id
        rows.sort(
            key=lambda r: (
                int(r.get("order", 0)),
                int(r.get("group_id", 0)),
                int(r.get("point_id", 0)),
            )
        )

        prev_gid = None

        for r in rows:
            gid = int(r["group_id"])   # tag_id

            # ---- task step (move + scan) ----
            if gid != prev_gid:
                task_steps.append({
                    "tag": gid,
                    "scan": True
                })
                prev_gid = gid

            # ---- scan point ----
            mode = "joint"  # ⭐ default pose
            speed = float(r.get("speed", 80))

            if mode == "pose":
                scan_points_by_tag[gid].append({
                    "mode": "pose",
                    "x": float(r["x"]),
                    "y": float(r["y"]),
                    "z": float(r["z"]),
                    "rx": float(r["rx"]),
                    "ry": float(r["ry"]),
                    "rz": float(r["rz"]),
                    "speed": speed
                })

            elif mode == "joint":
                scan_points_by_tag[gid].append({
                    "mode": "joint",
                    "joints": [
                        float(r["q1"]), float(r["q2"]), float(r["q3"]),
                        float(r["q4"]), float(r["q5"]), float(r["q6"])
                    ],
                    "speed": speed
                })
            else:
                rospy.logwarn(
                    f"[TaskManager] Unknown mode '{mode}' "
                    f"in task '{task_name}', tag {gid}"
                )

        self.tasks[task_name] = task_steps
        self.scan_points[task_name] = scan_points_by_tag

    # --------------------------------------------------
    # BUILD MOVE-ONLY TASK
    # --------------------------------------------------
    def _build_move_only_task(self, task_name: str, rows: List[dict]):
        task_steps = []

        for r in rows:
            gid = int(r["group_id"])
            task_steps.append({
                "tag": gid,
                "scan": False
            })

        self.tasks[task_name] = task_steps
        self.scan_points[task_name] = {}

    # --------------------------------------------------
    # PUBLIC API
    # --------------------------------------------------
    def get_task(self, task_name: str) -> List[dict]:
        return self.tasks.get(task_name, [])

    def get_scan_points(self, task_name: str, tag_id: int) -> List[dict]:
        return self.scan_points.get(task_name, {}).get(tag_id, [])

    def get_all_task_names(self) -> List[str]:
        return list(self.tasks.keys())
