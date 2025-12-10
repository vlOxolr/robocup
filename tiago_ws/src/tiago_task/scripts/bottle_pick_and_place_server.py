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


# 把 MoveIt 错误码转成字符串，方便打印
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
    """仿写官方 demo：构造 PickupGoal"""
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
    goal.allowed_touch_objects = ['<octomap>']
    goal.attached_object_touch_links = ['<octomap>']
    goal.attached_object_touch_links.extend(links_to_allow_contact)

    return goal


class BottlePickServer(object):
    """为当前桌上抓瓶子的场景定制的 /pickup_pose 服务器"""

    def __init__(self):
        rospy.loginfo("Initializing BottlePickServer ...")

        # 1) Spherical grasps（和官方 demo 一样）
        self.sg = SphericalGrasps()

        # 2) 连接 MoveIt 的 /pickup 动作
        rospy.loginfo("Connecting to /pickup (MoveIt PickupAction) ...")
        self.pickup_ac = SimpleActionClient("/pickup", PickupAction)
        if not self.pickup_ac.wait_for_server(rospy.Duration(20.0)):
            rospy.logerr("Could not connect to /pickup action server")
            raise RuntimeError("No /pickup action server")
        rospy.loginfo("Connected to /pickup.")

        # 3) 场景 & 服务
        self.scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service ...")
        self.scene_srv = rospy.ServiceProxy("/get_planning_scene",
                                            GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected to /get_planning_scene.")

        rospy.loginfo("Connecting to /clear_octomap service ...")
        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap",
                                                    Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo("Connected to /clear_octomap.")

        # 4) 读取参数（物体尺寸 & 允许接触的连杆）
        self.object_height = rospy.get_param("~object_height", 0.22)  # m
        self.object_width  = rospy.get_param("~object_width",  0.06)
        self.object_depth  = rospy.get_param("~object_depth",  0.06)

        self.links_to_allow_contact = rospy.get_param(
            "~links_to_allow_contact",
            ["arm_tool_link",
             "gripper_left_finger_link",
             "gripper_right_finger_link"]
        )
        rospy.loginfo("Object size [w d h] = [%.3f, %.3f, %.3f]",
                      self.object_width,
                      self.object_depth,
                      self.object_height)
        rospy.loginfo("Links to allow contact: %s",
                      str(self.links_to_allow_contact))

        # 5) 创建 /pickup_pose 动作服务器（只做 pick）
        self.pick_as = SimpleActionServer(
            "/pickup_pose",
            PickUpPoseAction,
            execute_cb=self.pick_cb,
            auto_start=False
        )
        self.pick_as.start()
        rospy.loginfo("BottlePickServer: /pickup_pose action server started.")

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------
    def wait_for_planning_scene_object(self, object_name="part"):
        """等待某个 collision object 出现在 planning scene 中"""
        rospy.loginfo("Waiting for object '%s' in planning scene ...",
                      object_name)
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
        """
        根据当前的 object_pose 在 planning scene 里添加:
        - 'part': 瓶子（box）
        - 'table': 桌面（box），中心在物体正下方
        """
        rospy.loginfo("Clearing previous 'part' and 'table' from scene ...")
        self.scene.remove_attached_object("arm_tool_link")
        self.scene.remove_world_object("part")
        self.scene.remove_world_object("table")

        # 清理 octomap，防止旧障碍物影响
        rospy.loginfo("Clearing octomap ...")
        try:
            self.clear_octomap_srv.call(EmptyRequest())
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to call clear_octomap: %s", str(e))
        rospy.sleep(1.0)

        # ------- 添加 "part" -------
        obj_pose = deepcopy(object_pose)

        # 小的 z 偏移：让 box 中心稍微高一点，避免陷到桌面里
        obj_pose.pose.position.z += 0.01

        rospy.loginfo("Adding 'part' at pose: (%.3f, %.3f, %.3f) in frame %s",
                      obj_pose.pose.position.x,
                      obj_pose.pose.position.y,
                      obj_pose.pose.position.z,
                      obj_pose.header.frame_id)

        # 注意：add_box(size=(x, y, z))
        self.scene.add_box("part",
                           obj_pose,
                           (self.object_depth,
                            self.object_width,
                            self.object_height))

        # ------- 添加 "table" -------
        
        table_pose = deepcopy(obj_pose)

        # 假设物体放在桌面上：
        # 桌面高度 = 物体中心高度 - 物体一半高度
        table_top_z = obj_pose.pose.position.z - self.object_height / 2.0
        table_height = max(table_top_z, 0.02)  # 至少 2 cm
        rospy.loginfo("Assumed table top z = %.3f m", table_top_z)

        table_size_x = 0.8  # 桌子尺寸可以视情况再调
        table_size_y = 0.8

        # 桌子中心在物体(x, y)正下方
        table_pose.pose.position.x = obj_pose.pose.position.x
        table_pose.pose.position.y = obj_pose.pose.position.y
        table_pose.pose.position.z = table_height / 2.0

        rospy.loginfo("Adding 'table' below part: size=(%.2f, %.2f, %.2f)",
                      table_size_x, table_size_y, table_height)

        self.scene.add_box("table",
                           table_pose,
                           (table_size_x,
                            table_size_y,
                            table_height))

        # 等待两者进入 planning scene
        self.wait_for_planning_scene_object("part")
        #self.wait_for_planning_scene_object("table")

        return obj_pose

    def plan_and_pick(self, object_pose):
        """用 SphericalGrasps + MoveIt /pickup 真正执行抓取"""
        # 1) 更新场景
        obj_pose_scene = self.add_part_and_table(object_pose)

        # 2) 生成多组抓取姿态
        rospy.loginfo("Creating grasps from object pose ...")
        possible_grasps = self.sg.create_grasps_from_object_pose(obj_pose_scene)

        rospy.loginfo("Generated %d grasps.", len(possible_grasps))

        # 3) 构造 PickupGoal
        goal = create_pickup_goal(
            group="arm_torso",
            target="part",
            grasp_pose=obj_pose_scene,
            possible_grasps=possible_grasps,
            links_to_allow_contact=self.links_to_allow_contact
        )

        # 4) 调用 /pickup 动作
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

    # ------------------------------------------------------------------
    # Action callback
    # ------------------------------------------------------------------
    def pick_cb(self, goal):
        """
        /pickup_pose 的回调
        :type goal: PickUpPoseGoal
        """
        rospy.loginfo("BottlePickServer: received goal with pose in frame %s",
                      goal.object_pose.header.frame_id)

        # 调用上面的 plan_and_pick
        error_code = self.plan_and_pick(goal.object_pose)

        res = PickUpPoseResult()
        res.error_code = error_code

        if error_code != MoveItErrorCodes.SUCCESS:
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
