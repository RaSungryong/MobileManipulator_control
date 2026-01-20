#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List, Dict


class TaskManager:
    """
    Final TaskManager (single responsibility, stable design)

    Design rules:
    - Defines WHAT to visit (tag) and HOW to use it (scan or move-only)
    - Does NOT include Dock / Pivot / Move tags
    - Does NOT expand navigation paths
    - Execution order is explicitly defined
    """

    # --------------------------------------------------
    # INIT
    # --------------------------------------------------
    def __init__(self):
        """
        Initialize all predefined tasks.
        """
        self._tasks: Dict[str, List[Dict]] = {
            "TASK1": self._task_zone_b_c(),
            "TASK2": self._task_zone_d_e(),
            "TASK_SINGLE": self._task_single_example(),
        }

    # --------------------------------------------------
    # Public API
    # --------------------------------------------------
    def get_task(self, task_name: str) -> List[Dict]:
        """
        Return task execution list.

        Each item format:
            {
                "tag": int,
                "scan": bool
            }

        Args:
            task_name (str): Task identifier

        Returns:
            List[Dict]: Ordered task items
        """
        task_name = task_name.upper()
        return self._tasks.get(task_name, [])

    def list_tasks(self) -> List[str]:
        """List all available task names"""
        return list(self._tasks.keys())

    # --------------------------------------------------
    # Task Definitions
    # --------------------------------------------------
    def _task_zone_b_c(self) -> List[Dict]:
        """
        TASK1:
        - Zone B: scan all
        - Zone C: scan all
        """
        zone_b = [{"tag": t, "scan": True} for t in range(111, 99, -1)]
        zone_c = [{"tag": t, "scan": True} for t in range(112, 124)]
        return zone_b + zone_c

    def _task_zone_d_e(self) -> List[Dict]:
        """
        TASK2:
        - Zone D: scan all
        - Zone E: scan all
        """
        zone_d = [{"tag": t, "scan": True} for t in range(135, 123, -1)]
        zone_e = [{"tag": t, "scan": True} for t in range(136, 148)]
        return zone_d + zone_e

    def _task_single_example(self) -> List[Dict]:
        """
        Example:
        - Move to a WORK tag but do NOT scan
        - Then scan another WORK tag
        """
        return [
            {"tag": 104, "scan": False},  # move-only
            {"tag": 105, "scan": True},   # move + scan
        ]
