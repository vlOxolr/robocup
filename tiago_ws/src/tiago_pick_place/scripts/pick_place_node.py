#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
import actionlib
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped

from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_msgs.msg import MoveGroupAction, PlanningScene, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

from tiago_pick_place.scene_freezer import SceneFreezer, sanitize_id


class TiagoPickPlace:
    def __init__(self):
        roscpp_initialize([])

        # ---------------- Params ----------------
        self.marker_topic = rospy.get_param("~marker_topic", "/item_markers")
        self.freeze_wait_sec = float(rospy.get_param("~freeze_wait_sec", 2.5))
        self.planning_frame = rospy.get_param("~planning_frame", "base_link")
        self.target_ns = rospy.get_param("~target_ns", "apple")

        self.arm_group_name = rospy.get_param("~arm_group", "arm_torso")
        self.gripper_group_name = rospy.get_param("~gripper_group", "gripper")
        self.ee_link = rospy.get_param("~ee_link", "")

        # Wait time for MoveIt move_group action server
        self.move_group_wait_sec = float(rospy.get_param("~move_group_wait_sec", 30.0))

        self.pregrasp_dx = float(rospy.get_param("~pregrasp_dx", 0.0))
        self.pregrasp_dy = float(rospy.get_param("~pregrasp_dy", 0.0))
        self.pregrasp_dz = float(rospy.get_param("~pregrasp_dz", 0.15))
        self.approach_dist = float(rospy.get_param("~approach_dist", 0.10))
        self.retreat_dist = float(rospy.get_param("~retreat_dist", 0.10))

        self.gripper_open = float(rospy.get_param("~gripper_open", 0.04))
        self.gripper_closed = float(rospy.get_param("~gripper_closed", 0.00))

        # ---------------- Ensure move_group is ready ----------------
        self._wait_for_move_group("/move_group", self.move_group_wait_sec)

        # ---------------- MoveIt groups (after server is ready) ----------------
        self.arm = MoveGroupCommander(self.arm_group_name)
        self.gripper = MoveGroupCommander(self.gripper_group_name)

        if self.ee_link:
            self.arm.set_end_effector_link(self.ee_link)

        self.arm.set_planning_time(5.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.allow_replanning(True)

        # ---------------- Scene freezer & /planning_scene publisher ----------------
        self.freezer = SceneFreezer(
            marker_topic=self.marker_topic,
            freeze_wait_sec=self.freeze_wait_sec,
            planning_frame=self.planning_frame
        )
        self._scene_pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=10, latch=False)

        # ---------------- Service ----------------
        self.srv = rospy.Service("~trigger_pick", Trigger, self.handle_trigger_pick)

        rospy.loginfo("tiago_pick_place ready.")
        rospy.loginfo(" - move_group action: /move_group (wait %.1fs)", self.move_group_wait_sec)
        rospy.loginfo(" - marker_topic: %s", self.marker_topic)
        rospy.loginfo(" - planning_frame: %s", self.planning_frame)
        rospy.loginfo(" - target_ns: %s", self.target_ns)
        rospy.loginfo(" - arm_group: %s", self.arm_group_name)
        rospy.loginfo(" - gripper_group: %s", self.gripper_group_name)
        rospy.loginfo(" - ee_link: %s", self.arm.get_end_effector_link())

    def _wait_for_move_group(self, action_name: str, timeout_sec: float):
        rospy.loginfo("Waiting for MoveGroupAction server at '%s' (timeout=%.1fs)...", action_name, timeout_sec)
        client = actionlib.SimpleActionClient(action_name, MoveGroupAction)
        ok = client.wait_for_server(rospy.Duration.from_sec(timeout_sec))
        if not ok:
            # Give diagnostics
            try:
                topics = rospy.get_published_topics()
                mg_topics = [t for (t, _ty) in topics if "move_group" in t]
                rospy.logerr("MoveGroupAction server not available at '%s'.", action_name)
                rospy.logerr("Published topics containing 'move_group' (sample): %s", mg_topics[:30])
            except Exception:
                pass
            raise RuntimeError(f"MoveGroupAction server not available: {action_name}")

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
        try:
            cur = self.arm.get_current_pose().pose
            wps = [copy.deepcopy(cur)]
            nxt = copy.deepcopy(cur)
            nxt.position.z += dz
            wps.append(nxt)

            traj, fraction = self.arm.compute_cartesian_path(
                wps, eef_step=0.01, jump_threshold=0.0
            )
            if fraction < 0.95:
                rospy.logwarn("Cartesian path fraction too low: %.2f", fraction)
                return False

            self.arm.execute(traj, wait=True)
            self.arm.stop()
            return True
        except Exception as e:
            rospy.logwarn("Cartesian shift failed: %s", str(e))
            return False

    # -------- Target selection --------
    def _select_target_marker(self):
        msg = self.freezer.get_latest()
        if msg is None or len(msg.markers) == 0:
            return (False, None, None, None)

        for m in msg.markers:
            if m.type != m.CUBE:
                continue
            if m.ns != self.target_ns:
                continue

            obj_id = (sanitize_id(m.ns) + "_" + str(m.id)) if m.ns else str(m.id)

            obj_pose = PoseStamped()
            obj_pose.header.frame_id = self.planning_frame
            obj_pose.header.stamp = rospy.Time.now()
            obj_pose.pose = m.pose

            size = (float(m.scale.x), float(m.scale.y), float(m.scale.z))
            rospy.loginfo("Selected target: ns='%s' id=%d -> object_id='%s'", m.ns, m.id, obj_id)
            return (True, obj_id, obj_pose, size)

        return (False, None, None, None)

    # -------- Attach via /planning_scene diff --------
    def _publish_attach(self, ee_link: str, obj_id: str, obj_pose, size_xyz) -> bool:
        try:
            aco = AttachedCollisionObject()
            aco.link_name = ee_link

            aco.object.header.stamp = rospy.Time.now()
            aco.object.header.frame_id = self.planning_frame
            aco.object.id = obj_id
            aco.object.operation = aco.object.ADD

            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [max(size_xyz[0], 1e-6), max(size_xyz[1], 1e-6), max(size_xyz[2], 1e-6)]
            aco.object.primitives.append(prim)
            aco.object.primitive_poses.append(obj_pose)

            aco.touch_links = self.arm.get_link_names()

            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.attached_collision_objects.append(aco)
            self._scene_pub.publish(scene)
            return True
        except Exception as e:
            rospy.logwarn("Publish attach failed: %s", str(e))
            return False

    # -------- Main service --------
    def handle_trigger_pick(self, _req):
        self._open_gripper()

        if not self.freezer.freeze_once():
            return TriggerResponse(False, "Freeze failed: no boxes published (no markers?).")

        ok_sel, obj_id, obj_pose, size_xyz = self._select_target_marker()
        if not ok_sel:
            return TriggerResponse(False, f"Target ns='{self.target_ns}' not found in markers.")

        pre = self._compute_pregrasp_pose(obj_pose)
        self.arm.set_pose_target(pre)
        ok_pre = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not ok_pre:
            return TriggerResponse(False, "Failed to reach pregrasp pose.")

        if not self._cartesian_shift_world_z(-abs(self.approach_dist)):
            return TriggerResponse(False, "Approach failed (cartesian world Z).")

        if not self._close_gripper():
            return TriggerResponse(False, "Close gripper failed.")

        ee_link = self.arm.get_end_effector_link()
        if not ee_link:
            return TriggerResponse(False, "End-effector link empty. Set ~ee_link correctly.")

        if not self._publish_attach(ee_link, obj_id, obj_pose.pose, size_xyz):
            return TriggerResponse(False, "Attach publish failed.")

        if not self._cartesian_shift_world_z(+abs(self.retreat_dist)):
            return TriggerResponse(False, "Retreat failed (cartesian world Z).")

        return TriggerResponse(True, f"Pick done. Attached object: {obj_id}")


def main():
    rospy.init_node("tiago_pick_place")
    TiagoPickPlace()
    rospy.spin()


if __name__ == "__main__":
    main()
