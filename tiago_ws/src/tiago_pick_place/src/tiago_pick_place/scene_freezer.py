#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive


def sanitize_id(s: str) -> str:
    if s is None:
        return ""
    return s.replace(" ", "_").replace("/", "_").replace("\\", "_")


def marker_to_collision_object(marker: Marker, override_frame: str):
    """Convert Marker(CUBE) into moveit_msgs/CollisionObject(BOX)."""
    if marker.type != Marker.CUBE:
        return None

    co = CollisionObject()
    co.header.stamp = rospy.Time.now()
    co.header.frame_id = override_frame if override_frame else marker.header.frame_id
    co.id = (sanitize_id(marker.ns) + "_" + str(marker.id)) if marker.ns else str(marker.id)

    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [
        max(float(marker.scale.x), 1e-6),
        max(float(marker.scale.y), 1e-6),
        max(float(marker.scale.z), 1e-6),
    ]

    co.primitives.append(prim)
    co.primitive_poses.append(marker.pose)
    co.operation = CollisionObject.ADD
    return co


class SceneFreezer:
    """
    Cache latest MarkerArray until freeze_once() is called.
    freeze_once(): wait freeze_wait_sec, publish PlanningScene diff to /planning_scene once,
    then stop accepting updates.
    """
    def __init__(self, marker_topic: str, freeze_wait_sec: float, planning_frame: str):
        self.marker_topic = marker_topic
        self.freeze_wait_sec = float(freeze_wait_sec)
        self.planning_frame = planning_frame

        self._latest = None
        self._frozen = False

        self._sub = rospy.Subscriber(self.marker_topic, MarkerArray, self._cb, queue_size=1)
        # latch=True so late subscribers (RViz etc.) can still see last diff
        self._pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=10, latch=True)

    def _cb(self, msg: MarkerArray):
        if self._frozen:
            return
        self._latest = msg

    def freeze_once(self) -> bool:
        rospy.loginfo("Freeze requested. Waiting %.2fs to stabilize markers...", self.freeze_wait_sec)

        end_t = rospy.Time.now() + rospy.Duration.from_sec(self.freeze_wait_sec)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < end_t:
            rate.sleep()

        if self._latest is None:
            rospy.logwarn("No MarkerArray received from %s. Freeze aborted.", self.marker_topic)
            return False

        self._frozen = True

        scene = PlanningScene()
        scene.is_diff = True

        added = 0
        for m in self._latest.markers:
            co = marker_to_collision_object(m, self.planning_frame)
            if co is None:
                continue
            scene.world.collision_objects.append(co)
            added += 1

        if added == 0:
            rospy.logwarn("Freeze done but no CUBE markers converted.")
            return False

        self._pub.publish(scene)
        rospy.loginfo("Published PlanningScene diff: %d collision objects to /planning_scene", added)
        return True

    def get_latest(self):
        return self._latest

    def is_frozen(self) -> bool:
        return self._frozen
