#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import copy
import math
from collections import deque, defaultdict
from typing import Dict, List, Tuple

from visualization_msgs.msg import MarkerArray, Marker
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped


def sanitize_id(s: str) -> str:
    if s is None:
        return ""
    return s.replace(" ", "_").replace("/", "_").replace("\\", "_")


def _euclid3(a, b) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def kde_mode_discrete(points: List[Tuple[float, float, float]], bandwidth: float) -> Tuple[float, float, float]:
    """
    Discrete KDE mode estimator: evaluate kernel density at sample points and pick argmax.

    points: list of 3D points
    bandwidth: gaussian kernel bandwidth (meters)
    """
    if not points:
        return (0.0, 0.0, 0.0)
    if len(points) == 1:
        return points[0]
    h = max(float(bandwidth), 1e-6)
    inv2h2 = 1.0 / (2.0 * h * h)

    best_i = 0
    best_score = -1.0
    for i, pi in enumerate(points):
        score = 0.0
        for pj in points:
            d = _euclid3(pi, pj)
            score += math.exp(-(d * d) * inv2h2)
        if score > best_score:
            best_score = score
            best_i = i
    return points[best_i]


class SceneFreezer:
    """
    Subscribe MarkerArray (e.g., /item_markers) and build a *stable* set of collision boxes for MoveIt.

    Workflow:
      - Continuously buffer MarkerArray messages in a deque.
      - When freeze_once() is called:
          1) wait freeze_wait_sec to fill the buffer
          2) collect messages in the last freeze_wait_sec window
          3) canonicalize labels via label_aliases
          4) estimate pose/size per canonical label (KDE mode + median)
          5) add_box() into PlanningScene

    IMPORTANT:
      - This class used to be "one-shot" (frozen forever). That breaks repeated pick calls.
      - Use reset() before each new freeze_once() to re-enable buffering.
    """

    def __init__(self, marker_topic: str, freeze_wait_sec: float, planning_frame: str):
        self.marker_topic = marker_topic
        self.freeze_wait_sec = float(freeze_wait_sec)
        self.planning_frame = planning_frame

        # Canonical label -> aliases (override via ~label_aliases in launch)
        default_aliases = {
            "apple": ["apple", "sports ball", "ball"],
            "burger": ["box", "book", "laptop"],
            "cola": ["bottle", "can", "glass"],
        }
        self.label_aliases: Dict[str, List[str]] = rospy.get_param("~label_aliases", default_aliases)

        # Build reverse map: alias -> canonical
        self._alias_to_canon: Dict[str, str] = {}
        for canon, aliases in self.label_aliases.items():
            self._alias_to_canon[canon] = canon
            for a in aliases:
                self._alias_to_canon[a] = canon

        # KDE bandwidth (meters) and minimum samples in window
        self.kde_bandwidth = float(rospy.get_param("~kde_bandwidth", 0.03))
        self.min_samples = int(rospy.get_param("~min_samples", 2))

        # Internal state
        self._frozen = False
        self._lock = threading.Lock()
        self._buf = deque(maxlen=3000)  # (stamp, MarkerArray)
        self.best_estimates: Dict[str, dict] = {}
        self._last_added_ids: List[str] = []

        # Use async scene interface to avoid blocking on /get_planning_scene
        self._scene = PlanningSceneInterface(synchronous=False)
        self._sub = rospy.Subscriber(self.marker_topic, MarkerArray, self._cb, queue_size=10)

    def canonicalize(self, ns: str) -> str:
        if ns is None:
            return ""
        return self._alias_to_canon.get(ns, ns)

    def reset(self, clear_scene: bool = True):
        """Reset to allow another freeze_once().

        clear_scene:
            If True, remove previously added collision boxes from world.
        """
        with self._lock:
            self._buf.clear()
        self.best_estimates = {}
        self._frozen = False

        if clear_scene and self._last_added_ids:
            for obj_id in list(self._last_added_ids):
                try:
                    self._scene.remove_world_object(obj_id)
                except Exception:
                    pass
            self._last_added_ids = []

    def _cb(self, msg: MarkerArray):
        # Keep buffering only when not frozen
        if self._frozen:
            return
        with self._lock:
            self._buf.append((rospy.Time.now(), msg))

    def _collect_window_msgs(self) -> List[MarkerArray]:
        now = rospy.Time.now()
        t_min = now - rospy.Duration.from_sec(self.freeze_wait_sec)
        out = []
        with self._lock:
            for (t, m) in list(self._buf):
                if t >= t_min:
                    out.append(m)
        return out

    @staticmethod
    def _robust_median(vals: List[float]) -> float:
        if not vals:
            return 0.0
        s = sorted(vals)
        n = len(s)
        if n % 2 == 1:
            return float(s[n // 2])
        return float(0.5 * (s[n // 2 - 1] + s[n // 2]))

    def freeze_once(self) -> bool:
        """Freeze a window and add estimated boxes to the planning scene."""
        rospy.loginfo("Freeze requested. Collecting observations for %.2fs ...", self.freeze_wait_sec)

        # Fill the buffer for at least freeze_wait_sec
        end_t = rospy.Time.now() + rospy.Duration.from_sec(self.freeze_wait_sec)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < end_t:
            rate.sleep()

        window_msgs = self._collect_window_msgs()
        if not window_msgs:
            rospy.logwarn("No MarkerArray buffered from %s. Freeze aborted.", self.marker_topic)
            return False

        dets = defaultdict(list)
        for ma in window_msgs:
            for m in ma.markers:
                if m.type != Marker.CUBE:
                    continue
                canon = self.canonicalize(m.ns)
                dets[canon].append({
                    "pos": (float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z)),
                    "size": (float(m.scale.x), float(m.scale.y), float(m.scale.z)),
                    "pose": copy.deepcopy(m.pose),
                })

        if not dets:
            rospy.logwarn("Freeze window had no CUBE markers.")
            return False

        estimates = {}
        for canon, items in dets.items():
            if len(items) < self.min_samples:
                continue

            positions = [it["pos"] for it in items]
            mode_pos = kde_mode_discrete(positions, self.kde_bandwidth)

            sx = self._robust_median([it["size"][0] for it in items])
            sy = self._robust_median([it["size"][1] for it in items])
            sz = self._robust_median([it["size"][2] for it in items])

            # Use orientation from the sample closest to the mode position
            best_pose = None
            best_dist = 1e9
            for it in items:
                d = _euclid3(it["pos"], mode_pos)
                if d < best_dist:
                    best_dist = d
                    best_pose = it["pose"]

            best_pose.position.x = mode_pos[0]
            best_pose.position.y = mode_pos[1]
            best_pose.position.z = mode_pos[2]

            obj_id = f"{sanitize_id(canon)}"
            estimates[canon] = {
                "count": len(items),
                "pose": best_pose,
                "size": (sx, sy, sz),
                "obj_id": obj_id,
            }

        if not estimates:
            rospy.logwarn("No canonical label has enough samples (min_samples=%d).", self.min_samples)
            return False

        # Mark frozen so the buffer stops changing during scene update
        self._frozen = True
        self.best_estimates = estimates

        # Add boxes (remove existing id first to avoid stale objects)
        added = 0
        added_ids = []
        for canon, est in estimates.items():
            pose_st = PoseStamped()
            pose_st.header.stamp = rospy.Time.now()
            pose_st.header.frame_id = self.planning_frame
            pose_st.pose = est["pose"]

            obj_id = est["obj_id"]
            size = est["size"]
            try:
                self._scene.remove_world_object(obj_id)
                self._scene.add_box(obj_id, pose_st, size=size)
                added += 1
                added_ids.append(obj_id)
            except Exception as e:
                rospy.logwarn("Failed to add box '%s' (canon=%s): %s", obj_id, canon, str(e))

        self._last_added_ids = added_ids

        rospy.loginfo("Frozen planning scene: added %d canonical collision boxes.", added)
        for canon, est in sorted(estimates.items(), key=lambda kv: kv[1]["count"], reverse=True):
            p = est["pose"].position
            rospy.loginfo("  canon=%s count=%d pos=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f)",
                          canon, est["count"], p.x, p.y, p.z, est["size"][0], est["size"][1], est["size"][2])

        return added > 0

    def is_frozen(self) -> bool:
        return self._frozen
