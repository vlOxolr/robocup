#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Guest Perception Node for TIAGo

Responsibilities:
- Person detection (YOLOv8)
- Wave gesture detection (YOLOv8 Pose)
- Guest counting and confirming
- Publish semantic perception results

NO speech, NO state machine, NO task logic.

Author: Your Name
"""

import time
import threading
from collections import deque

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from ultralytics import YOLO


class GuestPerceptionNode:
    def __init__(self):
        rospy.init_node("guest_perception_node", anonymous=False)

        # ======================================================
        # Parameters
        # ======================================================
        self.image_topic = rospy.get_param("~image_topic", "/xtion/rgb/image_raw")

        self.det_model_path = rospy.get_param("~det_model", "yolov8n.pt")
        self.pose_model_path = rospy.get_param("~pose_model", "yolov8n-pose.pt")
        self.device = rospy.get_param("~device", "cpu")  # 0=GPU, "cpu"=CPU

        self.conf_det = float(rospy.get_param("~conf_det", 0.35))
        self.iou_det = float(rospy.get_param("~iou_det", 0.5))
        self.conf_pose = float(rospy.get_param("~conf_pose", 0.45))
        self.iou_pose = float(rospy.get_param("~iou_pose", 0.5))

        self.process_hz = float(rospy.get_param("~process_hz", 8.0))

        # Session logic
        self.empty_reset_sec = float(rospy.get_param("~empty_reset_sec", 2.0))
        self.session_timeout_sec = float(rospy.get_param("~session_timeout_sec", 30.0))

        # Wave detection tuning
        self.wave_time_sec = float(rospy.get_param("~wave_time_sec", 1.0))
        self.wave_min_swing_px = float(rospy.get_param("~wave_min_swing_px", 65.0))
        self.wave_min_dir_changes = int(rospy.get_param("~wave_min_dir_changes", 2))
        self.wave_min_raised_frames = int(rospy.get_param("~wave_min_raised_frames", 3))
        self.wave_raise_margin_ratio = float(rospy.get_param("~wave_raise_margin_ratio", 0.12))

        # Debug visualization
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))

        # ======================================================
        # ROS publishers / subscribers
        # ======================================================
        self.bridge = CvBridge()

        self.visible_pub = rospy.Publisher("/guest_visible_count", Int32, queue_size=10)
        self.confirmed_pub = rospy.Publisher("/guest_confirmed_count", Int32, queue_size=10)
        self.event_pub = rospy.Publisher("/guest_event", String, queue_size=10)

        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher(
                "/guest_perception/debug_image", Image, queue_size=1
            )

        rospy.Subscriber(self.image_topic, Image, self._cb_image, queue_size=1, buff_size=2**24)

        # ======================================================
        # Models
        # ======================================================
        self.det_model = YOLO(self.det_model_path)
        self.pose_model = YOLO(self.pose_model_path)

        # ======================================================
        # Internal state
        # ======================================================
        self.state = "WAIT_WAVE"
        self.confirmed_count = 0
        self.confirm_time = 0.0

        self.last_process_time = 0.0
        self.last_nonzero_time = time.time()

        self._wrist_hist = deque(maxlen=60)
        self._raised_hist = deque(maxlen=20)

        rospy.loginfo("GuestPerceptionNode started.")
        rospy.loginfo("Subscribing image topic: %s", self.image_topic)

    # ======================================================
    # Utility
    # ======================================================
    def _throttle(self):
        if self.process_hz <= 0:
            return False
        now = time.time()
        if (now - self.last_process_time) < (1.0 / self.process_hz):
            return True
        self.last_process_time = now
        return False

    # ======================================================
    # Perception
    # ======================================================
    def _count_persons(self, frame):
        results = self.det_model.predict(
            source=frame,
            conf=self.conf_det,
            iou=self.iou_det,
            classes=[0],
            device=self.device,
            verbose=False
        )
        r0 = results[0]
        count = len(r0.boxes) if r0.boxes is not None else 0
        return count, r0

    def _wave_detected(self, frame):
        results = self.pose_model.predict(
            source=frame,
            conf=self.conf_pose,
            iou=self.iou_pose,
            classes=[0],
            device=self.device,
            verbose=False
        )
        r0 = results[0]
        kpts = getattr(r0, "keypoints", None)

        if kpts is None or kpts.xy is None or len(kpts.xy) == 0:
            self._raised_hist.append(False)
            return False, r0

        now = time.time()
        best = None

        for i in range(len(kpts.xy)):
            pts = kpts.xy[i]
            if pts is None or len(pts) < 11:
                continue

            nose = pts[0]
            lw = pts[9]
            rw = pts[10]

            margin = 20.0
            if r0.boxes is not None and len(r0.boxes) > i:
                _, y1, _, y2 = r0.boxes[i].xyxy[0].tolist()
                h = max(1.0, y2 - y1)
                margin = self.wave_raise_margin_ratio * h

            left_raised = (lw[1] + margin) < nose[1]
            right_raised = (rw[1] + margin) < nose[1]

            if left_raised or right_raised:
                cand = lw if left_raised else rw
                if best is None or cand[1] < best[1]:
                    best = (float(cand[0]), float(cand[1]))

        raised_now = best is not None
        self._raised_hist.append(raised_now)

        if sum(self._raised_hist) < self.wave_min_raised_frames:
            self._wrist_hist.clear()
            return False, r0

        wrist_x = best[0]
        self._wrist_hist.append((now, wrist_x))

        while self._wrist_hist and (now - self._wrist_hist[0][0]) > self.wave_time_sec:
            self._wrist_hist.popleft()

        if len(self._wrist_hist) < 6:
            return False, r0

        xs = [p[1] for p in self._wrist_hist]
        swing = max(xs) - min(xs)

        if swing < self.wave_min_swing_px:
            return False, r0

        dir_changes = 0
        prev_dx = 0.0
        for j in range(1, len(xs)):
            dx = xs[j] - xs[j - 1]
            if abs(dx) < 2.0:
                continue
            if prev_dx == 0.0:
                prev_dx = dx
                continue
            if dx * prev_dx < 0:
                dir_changes += 1
                prev_dx = dx

        if dir_changes >= self.wave_min_dir_changes:
            self._wrist_hist.clear()
            self._raised_hist.clear()
            return True, r0

        return False, r0

    # ======================================================
    # Callback
    # ======================================================
    def _cb_image(self, msg):
        if self._throttle():
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        now = time.time()

        visible_count, det_r0 = self._count_persons(frame)
        self.visible_pub.publish(Int32(visible_count))

        if visible_count > 0:
            self.last_nonzero_time = now

        if self.state == "WAIT_WAVE":
            wave, pose_r0 = self._wave_detected(frame) if visible_count > 0 else (False, None)

            if wave:
                self.state = "CONFIRMED"
                self.confirmed_count = visible_count
                self.confirm_time = now
                self.event_pub.publish(String(f"guest_wave_detected:{self.confirmed_count}"))
                rospy.loginfo("Wave detected, confirmed_count=%d", self.confirmed_count)

        elif self.state == "CONFIRMED":
            self.confirmed_pub.publish(Int32(self.confirmed_count))

            if (now - self.last_nonzero_time) > self.empty_reset_sec:
                self._reset("session_end")
            elif self.session_timeout_sec > 0 and (now - self.confirm_time) > self.session_timeout_sec:
                self._reset("session_timeout")

        if self.publish_debug_image and self.debug_pub is not None:
            img = det_r0.plot()
            out = self.bridge.cv2_to_imgmsg(img, "bgr8")
            out.header = msg.header
            self.debug_pub.publish(out)

    def _reset(self, reason):
        self.state = "WAIT_WAVE"
        self.confirmed_count = 0
        self.confirm_time = 0.0
        self._wrist_hist.clear()
        self._raised_hist.clear()
        self.event_pub.publish(String(reason))
        rospy.loginfo("Reset session (%s)", reason)


if __name__ == "__main__":
    GuestPerceptionNode()
    rospy.spin()
