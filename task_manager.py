#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List, Dict


class TaskManager:
    """
    Final TaskManager (single responsibility, stable design)

    Design rules:
    - Only defines WORK (scan) tags
    - Does NOT include Dock / Pivot / Move tags
    - Does NOT expand navigation paths
    - Order of tags == scan execution order
    """

    # --------------------------------------------------
    # INIT
    # --------------------------------------------------
    def __init__(self):
        """
        Initialize all predefined tasks.

        Task naming convention:
        - TASK1, TASK2, ...
        """
        self._tasks: Dict[str, List[int]] = {
            "TASK1": self._task_zone_b_c(),
            "TASK2": self._task_zone_d_e(),
        }

    # --------------------------------------------------
    # Public API
    # --------------------------------------------------
    def get_task(self, task_name: str) -> List[int]:
        """
        Return the scan tag list for a given task name.

        Args:
            task_name (str): Task identifier, e.g. "TASK1", "TASK2"

        Returns:
            List[int]: Ordered list of WORK tag IDs to be scanned.
                       Returns empty list if task does not exist.
        """
        task_name = task_name.upper()
        return self._tasks.get(task_name, [])

    def list_tasks(self) -> List[str]:
        """
        List all available task names.

        Returns:
            List[str]: Task name list
        """
        return list(self._tasks.keys())

    def is_scan_tag(self, tag_id: int) -> bool:
        """
        Check whether a tag ID is a WORK (scan) tag.

        Args:
            tag_id (int): AprilTag ID

        Returns:
            bool: True if tag is a scan target, False otherwise
        """
        return (
            100 <= tag_id <= 123 or
            124 <= tag_id <= 147
        )

    # --------------------------------------------------
    # Task Definitions
    # --------------------------------------------------
    def _task_zone_b_c(self) -> List[int]:
        """
        TASK1 definition:
        - Zone B: tags 111 → 100 (top to bottom)
        - Zone C: tags 112 → 123 (top to bottom)

        Returns:
            List[int]: Ordered scan sequence
        """
        zone_b = list(range(111, 99, -1))   # 111 ~ 100
        zone_c = list(range(112, 124))      # 112 ~ 123
        return zone_b + zone_c

    def _task_zone_d_e(self) -> List[int]:
        """
        TASK2 definition:
        - Zone D: tags 135 → 124 (top to bottom)
        - Zone E: tags 136 → 147 (top to bottom)

        Returns:
            List[int]: Ordered scan sequence
        """
        zone_d = list(range(135, 123, -1))  # 135 ~ 124
        zone_e = list(range(136, 148))      # 136 ~ 147
        return zone_d + zone_e
