#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from collections import deque
import cv2
import numpy as np
import tf.transformations as tft

from enum import Enum, auto
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from robot_msgs.msg import Pose2DWithFlag

try:
    from dt_apriltags import Detector
except ImportError:
    import subprocess
    subprocess.check_call(['pip', 'install', 'dt-apriltags'])
    from dt_apriltags import Detector


REFERENCE_FRAME = 'world'

# ============================================================
# TAG DATABASE
# ============================================================
class TagType(Enum):
    """Semantic type of AprilTags"""
    DOCK = auto()
    PIVOT = auto()
    WORK = auto()
    MOVE = auto()


class TagDatabase:
    """
    Centralized database for all AprilTag metadata.

    Each tag contains:
    - World position (x, y)
    - Semantic type (DOCK / PIVOT / WORK / MOVE)
    - Logical zone (A, B, C, ...)
    """

    def __init__(self):
        self.tags = {}
        self._build_database()

    def _build_database(self):
        """Hard-coded tag layout definition"""

        # Dock tag
        self.tags[508] = {'x': -0.3975, 'y': 5.35, 'type': TagType.DOCK, 'zone': 'DOCK'}

        # Pivot tags
        self.tags[500] = {'x': 0.0525, 'y': 5.35, 'type': TagType.PIVOT, 'zone': 'A'}
        self.tags[501] = {'x': -0.3975, 'y': 5.80, 'type': TagType.PIVOT, 'zone': 'B'}
        self.tags[502] = {'x': 3.2525, 'y': 5.35, 'type': TagType.PIVOT, 'zone': 'A'}
        self.tags[503] = {'x': 2.8025, 'y': 4.90, 'type': TagType.PIVOT, 'zone': 'C'}
        self.tags[504] = {'x': 3.9225, 'y': 5.35, 'type': TagType.PIVOT, 'zone': 'A'}
        self.tags[505] = {'x': 3.4725, 'y': 5.80, 'type': TagType.PIVOT, 'zone': 'D'}
        self.tags[506] = {'x': 7.1225, 'y': 5.35, 'type': TagType.PIVOT, 'zone': 'A'}
        self.tags[507] = {'x': 6.6725, 'y': 4.90, 'type': TagType.PIVOT, 'zone': 'E'}

        # Zone A corridor (left side)
        move_a_left = [(400, 0.4525), (401, 0.8525), (402, 1.2525),
                       (403, 1.6525), (404, 2.0525),
                       (405, 2.4525), (406, 2.8525)]
        for tag_id, x in move_a_left:
            self.tags[tag_id] = {'x': x, 'y': 5.35, 'type': TagType.MOVE, 'zone': 'A'}

        # Zone A corridor (right side)
        move_a_right = [(408, 4.3225), (409, 4.7225), (410, 5.1225),
                        (411, 5.5225), (412, 5.9225),
                        (413, 6.3225), (414, 6.7225)]
        for tag_id, x in move_a_right:
            self.tags[tag_id] = {'x': x, 'y': 5.35, 'type': TagType.MOVE, 'zone': 'A'}

        # Transition tags between zones
        self.tags[407] = {'x': 2.8025, 'y': 4.395, 'type': TagType.MOVE, 'zone': 'C'}
        self.tags[415] = {'x': 6.6725, 'y': 4.395, 'type': TagType.MOVE, 'zone': 'E'}

        # Zone B work tags (vertical corridor)
        y = 4.85
        for tag_id in range(111, 99, -1):
            self.tags[tag_id] = {'x': -0.3975, 'y': y, 'type': TagType.WORK, 'zone': 'B'}
            y -= 0.4

        # Zone C work tags
        y = 3.995
        for tag_id in range(112, 124):
            self.tags[tag_id] = {'x': 2.8025, 'y': y, 'type': TagType.WORK, 'zone': 'C'}
            y -= 0.4

        # Zone D work tags
        y = 4.85
        for tag_id in range(135, 123, -1):
            self.tags[tag_id] = {'x': 3.4725, 'y': y, 'type': TagType.WORK, 'zone': 'D'}
            y -= 0.4

        # Zone E work tags
        y = 3.995
        for tag_id in range(136, 148):
            self.tags[tag_id] = {'x': 6.6725, 'y': y, 'type': TagType.WORK, 'zone': 'E'}
            y -= 0.4

    def get(self, tag_id):
        """Return tag metadata"""
        return self.tags.get(tag_id)

    def exists(self, tag_id):
        """Check if tag exists"""
        return tag_id in self.tags


# ============================================================
# NAVIGATION GRAPH
# ============================================================
class NavigationGraph:
    """
    Topological navigation graph.

    Nodes   : AprilTags
    Edges   : movement primitives (forward / backward / pivot)
    """

    def __init__(self, tag_db):
        self.tag_db = tag_db
        self.edges = {}
        self._build_graph()

    def _add_edge(self, from_tag, to_tag, direction, action_type='move'):
        """Add directed edge"""
        if from_tag not in self.edges:
            self.edges[from_tag] = []
        self.edges[from_tag].append({
            'to': to_tag,
            'direction': direction,
            'type': action_type
        })

    def _build_graph(self):
        """Hard-coded navigation topology"""

        # Zone A main corridor
        zone_a = [508, 500, 400, 401, 402, 403, 404, 405, 406, 502,
                  504, 408, 409, 410, 411, 412, 413, 414, 506]
        for i in range(len(zone_a) - 1):
            self._add_edge(zone_a[i], zone_a[i + 1], 'forward')
            self._add_edge(zone_a[i + 1], zone_a[i], 'backward')

        # Zone B
        zone_b = [501, 111, 110, 109, 108, 107, 106, 105,
                  104, 103, 102, 101, 100]
        for i in range(len(zone_b) - 1):
            self._add_edge(zone_b[i], zone_b[i + 1], 'backward')
            self._add_edge(zone_b[i + 1], zone_b[i], 'forward')

        self._add_edge(500, 501, 'turn_ccw_90', 'pivot')
        self._add_edge(501, 500, 'turn_cw_90', 'pivot')

        # Zone C
        zone_c = [503, 407, 112, 113, 114, 115, 116,
                  117, 118, 119, 120, 121, 122, 123]
        for i in range(len(zone_c) - 1):
            self._add_edge(zone_c[i], zone_c[i + 1], 'forward')
            self._add_edge(zone_c[i + 1], zone_c[i], 'backward')

        self._add_edge(502, 503, 'turn_cw_90', 'pivot')
        self._add_edge(503, 502, 'turn_ccw_90', 'pivot')

        # Zone D
        zone_d = [505, 135, 134, 133, 132, 131, 130,
                  129, 128, 127, 126, 125, 124]
        for i in range(len(zone_d) - 1):
            self._add_edge(zone_d[i], zone_d[i + 1], 'backward')
            self._add_edge(zone_d[i + 1], zone_d[i], 'forward')

        self._add_edge(504, 505, 'turn_ccw_90', 'pivot')
        self._add_edge(505, 504, 'turn_cw_90', 'pivot')

        # Zone E
        zone_e = [507, 415, 136, 137, 138, 139, 140,
                  141, 142, 143, 144, 145, 146, 147]
        for i in range(len(zone_e) - 1):
            self._add_edge(zone_e[i], zone_e[i + 1], 'forward')
            self._add_edge(zone_e[i + 1], zone_e[i], 'backward')

        self._add_edge(506, 507, 'turn_cw_90', 'pivot')
        self._add_edge(507, 506, 'turn_ccw_90', 'pivot')

    def find_path(self, start, goal):
        """Breadth-first search path planning"""
        if start == goal:
            return [start]
        if start not in self.edges:
            return None

        visited = {start}
        queue = deque([(start, [start])])

        while queue:
            current, path = queue.popleft()
            for edge in self.edges.get(current, []):
                nxt = edge['to']
                if nxt not in visited:
                    if nxt == goal:
                        return path + [nxt]
                    visited.add(nxt)
                    queue.append((nxt, path + [nxt]))
        return None

    def get_edge(self, from_tag, to_tag):
        """Return edge metadata"""
        for edge in self.edges.get(from_tag, []):
            if edge['to'] == to_tag:
                return edge
        return None


# ============================================================
# MOBILE ROBOT CONTROLLER
# ============================================================
class MobileRobotController:
    """
    Vision-guided mobile base controller.

    Features:
    - AprilTag-based navigation
    - Topological graph planning
    - Safe pause / resume handling
    - Arm synchronization support
    """

    def __init__(self, tag_db, nav_graph):
        rospy.loginfo("[MobileRobotController] Initializing...")

        self.tag_db = tag_db
        self.nav_graph = nav_graph

        # Pause flag controlled by TaskExecutor
        self._paused = False

        # AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ROS interfaces
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('/robot_pose', Pose2DWithFlag, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rgb', Image, self.image_cb)
        self.camera_info_sub = rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        # Robot state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.theta = 0.0

        self.detected_tags = {}
        self.camera_ready = False
        self.camera_params = None

        self.current_tag = 508  # Start at dock

        # Motion parameters
        self.linear_speed = 0.30
        self.align_angle_threshold = 0.5
        self.center_y_tolerance = 50

        rospy.loginfo("[MobileRobotController] Ready")

    # ============================================================
    # PAUSE INTERFACE
    # ============================================================
    def set_pause(self, pause: bool):
        """Pause or resume robot motion"""
        self._paused = pause
        if pause:
            self.stop()
            rospy.logwarn("[Mobile] Paused")
        else:
            rospy.logwarn("[Mobile] Resumed")

    def _pause_gate(self):
        """Global pause gate for all motion loops"""
        rate = rospy.Rate(20)
        while self._paused and not rospy.is_shutdown():
            self.stop()
            rate.sleep()

    # ============================================================
    # ROS CALLBACKS
    # ============================================================
    def camera_info_cb(self, msg):
        if not self.camera_ready:
            K = msg.K
            self.camera_params = [K[0], K[4], K[2], K[5]]
            self.camera_ready = True
            rospy.loginfo("[Camera] CameraInfo received")

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def image_cb(self, msg):
        if not self.camera_ready:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=0.06
        )

        self.detected_tags.clear()
        for det in detections:
            if det.pose_t is None:
                continue
            pose_t = det.pose_t.flatten()
            self.detected_tags[det.tag_id] = {
                'x': pose_t[0],
                'center_y': det.center[1],
                'corners': det.corners
            }

    # ============================================================
    # MOTION PRIMITIVES
    # ============================================================
    def send_vel(self, linear, angular):
        self._pause_gate()
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def pivot_90deg(self, direction):
        """Rotate robot by 90 degrees"""
        angle = math.pi / 2
        start = self.theta
        target = start - angle if direction == 'cw' else start + angle

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self._pause_gate()
            err = target - self.theta
            if abs(err) < math.radians(1.0):
                break
            self.send_vel(0.0, 0.25 if err > 0 else -0.25)
            rate.sleep()

        self.stop()

    def align_to_tag(self, tag_id):
        """Align robot orientation using AprilTag edges"""
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._pause_gate()
            if tag_id not in self.detected_tags:
                rate.sleep()
                continue

            corners = self.detected_tags[tag_id]['corners']
            tl, tr = corners[0], corners[1]
            dx, dy = tr[0] - tl[0], tr[1] - tl[1]
            angle = math.degrees(math.atan2(dy, dx))
            angle = angle - 180 if angle > 90 else angle + 180 if angle < -90 else angle

            if abs(angle) < self.align_angle_threshold:
                break

            w = max(min(-math.radians(angle) * 0.8, 0.2), -0.2)
            self.send_vel(0.0, w)
            rate.sleep()

        self.stop()

    def move_to_visible_tag(self, target_tag, backward=False):
        """Move towards a visible tag using visual servoing"""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self._pause_gate()

            if target_tag not in self.detected_tags:
                self.send_vel(-0.2 if backward else 0.2, 0.0)
                rate.sleep()
                continue

            tag = self.detected_tags[target_tag]
            if abs(tag['center_y'] - 360) < self.center_y_tolerance:
                break

            angular = max(min(-tag['x'] * 1.5, 0.3), -0.3)
            linear = -self.linear_speed if backward else self.linear_speed
            self.send_vel(linear, angular)
            rate.sleep()

        self.stop()

    # ============================================================
    # HIGH-LEVEL API
    # ============================================================
    def move_to_tag(self, target_tag):
        """Navigate from current tag to target tag"""
        path = self.nav_graph.find_path(self.current_tag, target_tag)
        if not path:
            rospy.logerr("[Mobile] Path not found")
            return False

        for i in range(len(path) - 1):
            self._pause_gate()
            cur, nxt = path[i], path[i + 1]
            edge = self.nav_graph.get_edge(cur, nxt)

            if edge['type'] == 'pivot':
                self.pivot_90deg('ccw' if 'ccw' in edge['direction'] else 'cw')
            else:
                self.align_to_tag(cur)
                self.move_to_visible_tag(nxt, 'backward' in edge['direction'])

            self.current_tag = nxt

        rospy.loginfo(f"[Mobile] Arrived at tag {target_tag}")
        return True

    # ============================================================
    # ARM INTERFACE
    # ============================================================
    def publish_pose_for_arm(self, tag_id):
        """Send robot base pose to ArmController"""
        msg = Pose2DWithFlag()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = REFERENCE_FRAME
        msg.x = self.odom_x
        msg.y = self.odom_y
        msg.theta = math.degrees(self.theta)
        msg.flag = True
        msg.id = tag_id

        self.pose_pub.publish(msg)
