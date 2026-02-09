#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlaceAction, PlaceGoal, MoveItErrorCodes
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from geometry_msgs.msg import PoseStamped
from tiago_pick_demo.msg import (
    PlacePoseAction,
    PlacePoseGoal,
    PlacePoseResult,
)
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
import math
import tf.transformations as tf_trans

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name.startswith("_"):
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

def create_place_goal(group="arm_torso",
                      target="part",
                      place_pose=PoseStamped(),
                      links_to_allow_contact=None):
    if links_to_allow_contact is None:
        links_to_allow_contact = []
    goal = PlaceGoal()
    goal.group_name = group
    goal.attached_object_name = target
    goal.place_locations = []
    # 这里只添加一个放置位姿
    from moveit_msgs.msg import PlaceLocation
    loc = PlaceLocation()
    loc.place_pose = place_pose
    loc.allowed_touch_objects = [target, "table"]
    loc.post_place_posture.header.frame_id = place_pose.header.frame_id
    goal.place_locations.append(loc)
    goal.allowed_touch_objects = [target, "table"]
    goal.attached_object_touch_links = []
    goal.attached_object_touch_links.extend(links_to_allow_contact)
    goal.allowed_planning_time = 35.0
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    goal.planning_options.plan_only = False
    goal.planning_options.replan = True
    goal.planning_options.replan_attempts = 1
    return goal

class BottlePlaceServer(object):
    def __init__(self):
        rospy.loginfo("Initializing BottlePlaceServer ...")
        rospy.loginfo("Connecting to /place (MoveIt PlaceAction) ...")
        self.place_ac = SimpleActionClient("/place", PlaceAction)
        if not self.place_ac.wait_for_server(rospy.Duration(20.0)):
            rospy.logerr("Could not connect to /place action server")
            raise RuntimeError("No /place action server")
        rospy.loginfo("Connected to /place.")
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
        self.place_as = SimpleActionServer(
            "/place_pose",
            PlacePoseAction,
            execute_cb=self.place_cb,
            auto_start=False
        )
        self.place_as.start()
        rospy.loginfo("BottlePlaceServer: /place_pose action server started.")
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
    def add_table(self, object_pose):
        rospy.loginfo("Clearing previous 'table' from scene ...")
        self.scene.remove_world_object("table")
        rospy.loginfo("Clearing octomap ...")
        try:
            self.clear_octomap_srv.call(EmptyRequest())
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to call clear_octomap: %s", str(e))
        rospy.sleep(1.0)
        table_pose = deepcopy(object_pose)
        table_top_z = object_pose.pose.position.z - self.object_height / 2.0
        table_height = max(table_top_z, 0.02)
        table_size_x = 0.8
        table_size_y = 0.8
        table_pose.pose.position.x = object_pose.pose.position.x
        table_pose.pose.position.y = object_pose.pose.position.y
        table_pose.pose.position.z = table_height / 2.0
        rospy.loginfo("Adding 'table' at: size=(%.2f, %.2f, %.2f)",
                      table_size_x, table_size_y, table_height)
        self.scene.add_box("table", table_pose, (table_size_x, table_size_y, table_height))
        self.wait_for_planning_scene_object("table")
        return table_pose
    def plan_and_place(self, object_pose):
        table_pose_scene = self.add_table(object_pose)
        goal = create_place_goal(
            group="arm_torso",
            target="part",
            place_pose=object_pose,
            links_to_allow_contact=self.links_to_allow_contact
        )
        goal.support_surface_name = "table"
        goal.allow_gripper_support_collision = True
        rospy.loginfo("Sending PlaceGoal to /place ...")
        self.place_ac.send_goal(goal)
        rospy.loginfo("Waiting for place result ...")
        self.place_ac.wait_for_result()
        result = self.place_ac.get_result()
        if result is None:
            rospy.logerr("No result from /place action!")
            return MoveItErrorCodes.PLANNING_FAILED
        code = result.error_code.val
        rospy.loginfo("Place result: %s (%d)",
                    moveit_error_dict.get(code, "UNKNOWN"), code)
        return code
    def place_cb(self, goal):
        rospy.loginfo("BottlePlaceServer: received goal with pose in frame %s",
                      goal.object_pose.header.frame_id)
        error_code = self.plan_and_place(goal.object_pose)
        res = PlacePoseResult()
        res.error_code = error_code
        if 0:
            rospy.logerr("BottlePlaceServer: place failed (%s)",
                         moveit_error_dict.get(error_code, "UNKNOWN"))
            self.place_as.set_aborted(res)
        else:
            rospy.loginfo("BottlePlaceServer: place succeeded.")
            self.place_as.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("bottle_place_server")
    server = BottlePlaceServer()
    rospy.loginfo("BottlePlaceServer is ready.")
    rospy.spin()
