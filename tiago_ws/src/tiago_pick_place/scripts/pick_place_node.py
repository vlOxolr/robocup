#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
import threading
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize

from tiago_pick_place.scene_freezer import SceneFreezer


class TiagoPickPlace:
    def __init__(self):
        roscpp_initialize([])

        # Params
        self.marker_topic = rospy.get_param("~marker_topic", "/item_markers")
        self.freeze_wait_sec = float(rospy.get_param("~freeze_wait_sec", 3.0))
        self.planning_frame = rospy.get_param("~planning_frame", "base_link")
        self.target_ns = rospy.get_param("~target_ns", "apple")  # canonical label or alias

        self.arm_group_name = rospy.get_param("~arm_group", "arm_torso")
        self.gripper_group_name = rospy.get_param("~gripper_group", "pal-gripper")
        self.ee_link = rospy.get_param("~ee_link", "")

        self.pregrasp_dx = float(rospy.get_param("~pregrasp_dx", 0.0))
        self.pregrasp_dy = float(rospy.get_param("~pregrasp_dy", 0.0))
        self.pregrasp_dz = float(rospy.get_param("~pregrasp_dz", 0.15))
        self.approach_dist = float(rospy.get_param("~approach_dist", 0.10))
        self.retreat_dist = float(rospy.get_param("~retreat_dist", 0.10))

        self.gripper_open = float(rospy.get_param("~gripper_open", 0.04))
        self.gripper_closed = float(rospy.get_param("~gripper_closed", 0.00))

        # MoveIt (async scene to avoid blocking on /get_planning_scene)
        self.scene = PlanningSceneInterface(synchronous=False)
        self.arm = MoveGroupCommander(self.arm_group_name)
        self.gripper = MoveGroupCommander(self.gripper_group_name)

        if self.ee_link:
            self.arm.set_end_effector_link(self.ee_link)

        self.arm.set_planning_time(5.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.allow_replanning(True)

        # Freezer (label_aliases + KDE mode)
        self.freezer = SceneFreezer(
            marker_topic=self.marker_topic,
            freeze_wait_sec=self.freeze_wait_sec,
            planning_frame=self.planning_frame
        )

        # Persisted state for cleanup across calls
        self._last_attached_obj_id = ""
        self._srv_lock = threading.Lock()

        # Service
        self.srv = rospy.Service("~trigger_pick", Trigger, self.handle_trigger_pick)

        rospy.loginfo("tiago_pick_place ready.")
        rospy.loginfo(" - marker_topic: %s", self.marker_topic)
        rospy.loginfo(" - planning_frame: %s", self.planning_frame)
        rospy.loginfo(" - target_ns (requested): %s", self.target_ns)
        rospy.loginfo(" - arm_group: %s", self.arm_group_name)
        rospy.loginfo(" - gripper_group: %s", self.gripper_group_name)
        rospy.loginfo(" - ee_link: %s", self.arm.get_end_effector_link())

    # -------- Gripper --------
    def _set_gripper(self, value: float) -> bool:
        try:
            joints = self.gripper.get_active_joints()
            if not joints:
                rospy.logwarn("Gripper group has no active joints.")
                return False
            target = {j: value for j in joints}
            self.gripper.set_joint_value_target(target)
            ok = self.gripper.go(wait=True)
            self.gripper.stop()
            return bool(ok)
        except Exception as e:
            rospy.logwarn("Gripper command failed: %s", str(e))
            return False

    def _open_gripper(self) -> bool:
        return self._set_gripper(self.gripper_open)

    def _close_gripper(self) -> bool:
        return self._set_gripper(self.gripper_closed)

    # -------- Motion helpers --------
    def _compute_pregrasp_pose(self, obj_pose: PoseStamped) -> PoseStamped:
        pre = copy.deepcopy(obj_pose)
        pre.header.frame_id = self.planning_frame
        pre.header.stamp = rospy.Time.now()
        pre.pose.position.x += self.pregrasp_dx
        pre.pose.position.y += self.pregrasp_dy
        pre.pose.position.z += self.pregrasp_dz
        return pre

    def _cartesian_shift_world_z(self, dz: float) -> bool:
        """
        Cartesian shift along WORLD Z by dz (meters) from current EE pose.

        MoveIt Python bindings have different signatures across versions.
        We'll try common variants until one works:
        A) (waypoints, eef_step, jump_threshold, avoid_collisions)
        B) (waypoints, eef_step, jump_threshold, avoid_collisions, path_constraints)
        C) (waypoints, eef_step, jump_threshold, path_constraints)  # 4th is constraints
        D) (waypoints, eef_step, jump_threshold)  # very old
        """
        try:
            cur = self.arm.get_current_pose().pose
            wps = [copy.deepcopy(cur)]
            nxt = copy.deepcopy(cur)
            nxt.position.z += dz
            wps.append(nxt)

            eef_step = float(rospy.get_param("~cart_eef_step", 0.01))
            jump_threshold = float(rospy.get_param("~cart_jump_threshold", 0.0))
            avoid_collisions = bool(rospy.get_param("~cart_avoid_collisions", True))

            # Try variants
            variants = []

            # Variant A: classic
            variants.append(("A", (wps, eef_step, jump_threshold, avoid_collisions)))

            # Variant B: classic + constraints(None)
            variants.append(("B", (wps, eef_step, jump_threshold, avoid_collisions, None)))

            # Variant C: 4th is constraints, so pass None
            variants.append(("C", (wps, eef_step, jump_threshold, None)))

            # Variant D: old
            variants.append(("D", (wps, eef_step, jump_threshold)))

            last_err = None
            for tag, args in variants:
                try:
                    traj, fraction = self.arm.compute_cartesian_path(*args)
                    rospy.loginfo("compute_cartesian_path variant %s succeeded. fraction=%.2f", tag, fraction)
                    if fraction < 0.95:
                        rospy.logwarn("Cartesian path fraction too low: %.2f", fraction)
                        return False
                    ok = self.arm.execute(traj, wait=True)
                    self.arm.stop()
                    return bool(ok)
                except Exception as e:
                    last_err = e
                    # only warn at debug level to reduce spam
                    rospy.logdebug("Variant %s failed: %s", tag, str(e))
                    continue

            rospy.logwarn("Cartesian shift failed: %s", str(last_err))
            return False

        except Exception as e:
            rospy.logwarn("Cartesian shift failed: %s", str(e))
            return False


    # -------- Cleanup --------
    def _cleanup_previous(self):
        """Cleanup attached/world objects from previous runs to avoid accumulating stale state."""
        ee_link = self.arm.get_end_effector_link()

        # Remove any attached object on the end-effector
        if ee_link:
            try:
                self.scene.remove_attached_object(ee_link)
            except Exception:
                pass

        # Remove the previously attached object from world as well (safe even if not present)
        if self._last_attached_obj_id:
            try:
                self.scene.remove_world_object(self._last_attached_obj_id)
            except Exception:
                pass
            self._last_attached_obj_id = ""

    # -------- Target selection --------
    def _select_target_from_freeze(self):
        canon_target = self.freezer.canonicalize(self.target_ns)

        if not self.freezer.best_estimates:
            return (False, None, None)

        if canon_target not in self.freezer.best_estimates:
            best = sorted(self.freezer.best_estimates.items(), key=lambda kv: kv[1]["count"], reverse=True)
            hint = ", ".join([f"{k}(count={v['count']})" for k, v in best[:5]])
            rospy.logwarn("Target canon='%s' not found. Candidates: %s", canon_target, hint)
            return (False, None, None)

        est = self.freezer.best_estimates[canon_target]
        obj_id = est["obj_id"]

        obj_pose = PoseStamped()
        obj_pose.header.frame_id = self.planning_frame
        obj_pose.header.stamp = rospy.Time.now()
        obj_pose.pose = est["pose"]

        rospy.loginfo("Selected target canon='%s' obj_id='%s' count=%d",
                      canon_target, obj_id, est["count"])
        return (True, obj_id, obj_pose)

    # -------- Main service --------
    def handle_trigger_pick(self, _req):
        # Avoid concurrent calls (Trigger service can be spammed from clients)
        with self._srv_lock:
            # Refresh runtime params (your test script sets /tiago_pick_place/target_ns each call)
            self.target_ns = rospy.get_param("~target_ns", self.target_ns)

            # Reset freezer to make it reusable across repeated calls
            self.freezer.reset(clear_scene=True)

            # Cleanup previous attached/world objects
            self._cleanup_previous()

            # Open gripper (do not hard-fail if it returns False; keep going)
            self._open_gripper()

            # Freeze and build stable collision boxes
            if not self.freezer.freeze_once():
                return TriggerResponse(False, "Freeze failed: no stable canonical collision boxes built.")

            ok_sel, obj_id, obj_pose = self._select_target_from_freeze()
            if not ok_sel:
                return TriggerResponse(False, f"Target '{self.target_ns}' not found in freeze window (after alias mapping).")

            # Plan to pregrasp pose
            pre = self._compute_pregrasp_pose(obj_pose)
            self.arm.set_pose_target(pre)
            ok_pre = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            if not ok_pre:
                return TriggerResponse(False, "Failed to reach pregrasp pose.")

            # Approach
            if not self._cartesian_shift_world_z(-abs(self.approach_dist)):
                return TriggerResponse(False, "Approach failed (cartesian world Z).")

            # Close gripper
            if not self._close_gripper():
                return TriggerResponse(False, "Close gripper failed.")

            # Attach object to end-effector for planning
            ee_link = self.arm.get_end_effector_link()
            if not ee_link:
                return TriggerResponse(False, "End-effector link empty. Set ~ee_link correctly.")

            try:
                touch_links = self.arm.get_link_names()
                self.scene.attach_box(ee_link, obj_id, touch_links=touch_links)
                self._last_attached_obj_id = obj_id
            except Exception as e:
                return TriggerResponse(False, "Attach failed: " + str(e))

            # Retreat
            if not self._cartesian_shift_world_z(+abs(self.retreat_dist)):
                return TriggerResponse(False, "Retreat failed (cartesian world Z).")

            return TriggerResponse(True, f"Pick done. Attached object: {obj_id}")


def main():
    rospy.init_node("tiago_pick_place")
    TiagoPickPlace()
    rospy.spin()


if __name__ == "__main__":
    main()
