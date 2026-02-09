#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PickupAction, PickupGoal, MoveItErrorCodes
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from geometry_msgs.msg import PoseStamped
from tiago_pick_demo.msg import (
    PickUpPoseAction,
    PickUpPoseGoal,
    PickUpPoseResult,
)
from spherical_grasps_server import SphericalGrasps
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
import math
import tf.transformations as tf_trans

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name.startswith("_"):
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

def create_pickup_goal(group="arm_torso",
                       target="part",
                       grasp_pose=PoseStamped(),
                       possible_grasps=None,
                       links_to_allow_contact=None):
    if possible_grasps is None:
        possible_grasps = []
    if links_to_allow_contact is None:
        links_to_allow_contact = []
    goal = PickupGoal()
    goal.target_name = target
    goal.group_name = group
    goal.possible_grasps.extend(possible_grasps)
    goal.allowed_planning_time = 35.0
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    goal.planning_options.plan_only = False
    goal.planning_options.replan = True
    goal.planning_options.replan_attempts = 1
    goal.allowed_touch_objects = [target, "table"]
    goal.attached_object_touch_links = []
    goal.attached_object_touch_links.extend(links_to_allow_contact)
    return goal

class BottlePickServer(object):
    def __init__(self):
        rospy.loginfo("Initializing BottlePickServer ...")
        self.sg = SphericalGrasps()
        rospy.loginfo("Connecting to /pickup (MoveIt PickupAction) ...")
        self.pickup_ac = SimpleActionClient("/pickup", PickupAction)
        if not self.pickup_ac.wait_for_server(rospy.Duration(20.0)):
            rospy.logerr("Could not connect to /pickup action server")
            raise RuntimeError("No /pickup action server")
        rospy.loginfo("Connected to /pickup.")
        self.scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service ...")
        self.scene_srv = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected to /get_planning_scene.")
        rospy.loginfo("Connecting to /clear_octomap service ...")
        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo("Connected to /clear_octomap.")
        self.object_height = rospy.get_param("~object_height", 0.22)
        self.object_width  = rospy.get_param("~object_width",  0.059)
        self.object_depth  = rospy.get_param("~object_depth",  0.059)
        self.links_to_allow_contact = rospy.get_param(
            "~links_to_allow_contact",
            ["gripper_left_finger_link", "gripper_right_finger_link"]
        )
        self.roll_min_deg  = rospy.get_param("~ee_orientation_limits/roll_min",  -100.0)
        self.roll_max_deg  = rospy.get_param("~ee_orientation_limits/roll_max",   -80.0)
        self.pitch_min_deg = rospy.get_param("~ee_orientation_limits/pitch_min",  -30.0)
        self.pitch_max_deg = rospy.get_param("~ee_orientation_limits/pitch_max",   30.0)
        self.yaw_min_deg   = rospy.get_param("~ee_orientation_limits/yaw_min",    150.0)
        self.yaw_max_deg   = rospy.get_param("~ee_orientation_limits/yaw_max",    210.0)
        self.pick_as = SimpleActionServer(
            "/pickup_pose",
            PickUpPoseAction,
            execute_cb=self.pick_cb,
            auto_start=False
        )
        self.pick_as.start()
        rospy.loginfo("BottlePickServer: /pickup_pose action server started.")
    def wait_for_planning_scene_object(self, object_name="part"):
        rospy.loginfo("Waiting for object '%s' in planning scene ...", object_name)
        req = GetPlanningSceneRequest()
        req.components.components = req.components.WORLD_OBJECT_NAMES
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            resp = self.scene_srv.call(req)
            found = False
            for co in resp.scene.world.collision_objects:
                if co.id == object_name:
                    found = True
                    break
            if found:
                rospy.loginfo("Object '%s' is now in planning scene.", object_name)
                return True
            rate.sleep()
        return False
    def add_part_and_table(self, object_pose):
        rospy.loginfo("Clearing previous 'part' and 'table' from scene ...")
        self.scene.remove_attached_object("arm_tool_link")
        self.scene.remove_world_object("part")
        self.scene.remove_world_object("table")
        rospy.loginfo("Clearing octomap ...")
        try:
            self.clear_octomap_srv.call(EmptyRequest())
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to call clear_octomap: %s", str(e))
        rospy.sleep(1.0)
        obj_pose = deepcopy(object_pose)
        obj_pose.pose.position.z += 0.01
        rospy.loginfo("Adding 'part' at pose: (%.3f, %.3f, %.3f) in frame %s",
                      obj_pose.pose.position.x,
                      obj_pose.pose.position.y,
                      obj_pose.pose.position.z,
                      obj_pose.header.frame_id)
        self.scene.add_box("part", obj_pose, (self.object_depth, self.object_width, self.object_height))
        table_pose = deepcopy(obj_pose)
        table_top_z = obj_pose.pose.position.z - self.object_height / 2.0
        table_height = max(table_top_z, 0.02)
        table_size_x = 0.8
        table_size_y = 0.8
        table_pose.pose.position.x = obj_pose.pose.position.x
        table_pose.pose.position.y = obj_pose.pose.position.y
        table_pose.pose.position.z = table_height / 2.0
        rospy.loginfo("Adding 'table' below part: size=(%.2f, %.2f, %.2f)",
                      table_size_x, table_size_y, table_height)
        self.scene.add_box("table", table_pose, (table_size_x, table_size_y, table_height))
        self.wait_for_planning_scene_object("part")
        return obj_pose
    def filter_grasps_by_orientation(self, grasps):
        filtered = []
        for g in grasps:
            q = g.grasp_pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = tf_trans.euler_from_quaternion(quat)
            roll_deg  = roll  * 180.0 / math.pi
            pitch_deg = pitch * 180.0 / math.pi
            yaw_deg   = yaw   * 180.0 / math.pi
            if (self.roll_min_deg  <= roll_deg  <= self.roll_max_deg and
                self.pitch_min_deg <= pitch_deg <= self.pitch_max_deg and
                self.yaw_min_deg   <= yaw_deg   <= self.yaw_max_deg):
                filtered.append(g)
        rospy.loginfo("Grasp orientation filter: %d -> %d grasps",
                      len(grasps), len(filtered))
        if len(filtered) == 0:
            rospy.logwarn("All grasps filtered out by EE orientation limits, "
                          "falling back to unfiltered grasps. "
                          "Consider relaxing ee_orientation_limits.")
            return grasps
        return filtered
    def plan_and_pick(self, object_pose):
        obj_pose_scene = self.add_part_and_table(object_pose)
        rospy.loginfo("Creating grasps from object pose ...")
        possible_grasps = self.sg.create_grasps_from_object_pose(obj_pose_scene)
        rospy.loginfo("Generated %d grasps (before EE orientation filter).",
                      len(possible_grasps))
        possible_grasps = self.filter_grasps_by_orientation(possible_grasps)
        possible_grasps = self.rotate_grasps_fix_finger_dir(possible_grasps, angle_deg=90.0)
        possible_grasps = self.offset_grasps_down_world_z(possible_grasps, dz=0.02)
        goal = create_pickup_goal(
            group="arm_torso",
            target="part",
            grasp_pose=obj_pose_scene,
            possible_grasps=possible_grasps,
            links_to_allow_contact=self.links_to_allow_contact
        )
        goal.support_surface_name = "table"
        goal.allow_gripper_support_collision = True
        rospy.loginfo("Sending PickupGoal to /pickup ...")
        self.pickup_ac.send_goal(goal)
        rospy.loginfo("Waiting for pickup result ...")
        self.pickup_ac.wait_for_result()
        result = self.pickup_ac.get_result()
        if result is None:
            rospy.logerr("No result from /pickup action!")
            return MoveItErrorCodes.PLANNING_FAILED
        code = result.error_code.val
        rospy.loginfo("Pick result: %s (%d)",
                    moveit_error_dict.get(code, "UNKNOWN"), code)
        return code
    def offset_grasps_down_world_z(self, grasps, dz=0.04):
        for g in grasps:
            g.grasp_pose.pose.position.z -= dz
        rospy.loginfo("Offset %d grasps down by %.3f m along world Z",
                      len(grasps), dz)
        return grasps
    def rotate_grasps_fix_finger_dir(self, grasps, angle_deg=90.0):
        angle = math.radians(angle_deg)
        q_fix = tf_trans.quaternion_from_euler(angle, 0.0, 0.0)
        for g in grasps:
            q = g.grasp_pose.pose.orientation
            q_orig = [q.x, q.y, q.z, q.w]
            q_new = tf_trans.quaternion_multiply(q_orig, q_fix)
            q.x, q.y, q.z, q.w = q_new
        rospy.loginfo("Applied fixed rotation of %.1f deg around tool x-axis to %d grasps",
                      angle_deg, len(grasps))
        return grasps
    def pick_cb(self, goal):
        rospy.loginfo("BottlePickServer: received goal with pose in frame %s",
                      goal.object_pose.header.frame_id)
        error_code = self.plan_and_pick(goal.object_pose)
        res = PickUpPoseResult()
        res.error_code = error_code
        if 0:
            rospy.logerr("BottlePickServer: pick failed (%s)",
                         moveit_error_dict.get(error_code, "UNKNOWN"))
            self.pick_as.set_aborted(res)
        else:
            rospy.loginfo("BottlePickServer: pick succeeded.")
            self.pick_as.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("bottle_pick_server")
    server = BottlePickServer()
    rospy.loginfo("BottlePickServer is ready.")
    rospy.spin()
