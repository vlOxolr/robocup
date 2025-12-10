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

# >>> NEW: 用来做欧拉角和 clamp
import math
import tf.transformations as tf_trans


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
    if possible_grasps is None:
        possible_grasps = []
    if links_to_allow_contact is None:
        links_to_allow_contact = []

    goal = PickupGoal()
    goal.target_name = target
    goal.group_name = group
    goal.possible_grasps.extend(possible_grasps)

    # 规划相关
    goal.allowed_planning_time = 35.0
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    goal.planning_options.plan_only = False
    goal.planning_options.replan = True
    goal.planning_options.replan_attempts = 1

    # --------- 关键：允许接触的物体 & link ----------

    # 允许在抓取过程中碰撞的 world objects（至少要包括要抓的物体）
    # 支撑桌子如果叫 "table"，也可以一并放进去
    goal.allowed_touch_objects = [target, "table"]

    # 抓住之后，这些 link 允许和附着物体发生接触
    goal.attached_object_touch_links = []
    goal.attached_object_touch_links.extend(links_to_allow_contact)

    # ------------------------------------------------

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
            ["gripper_left_finger_link",
             "gripper_right_finger_link"]
        )
        rospy.loginfo("Object size [w d h] = [%.3f, %.3f, %.3f]",
                      self.object_width,
                      self.object_depth,
                      self.object_height)
        rospy.loginfo("Links to allow contact: %s",
                      str(self.links_to_allow_contact))

        # >>> NEW: 夹爪姿态角度限制（单位：度，在 base_link 下理解）
        # 你可以在 launch/yaml 里覆盖这些参数
        self.roll_min_deg  = rospy.get_param("~ee_orientation_limits/roll_min",  -100.0)
        self.roll_max_deg  = rospy.get_param("~ee_orientation_limits/roll_max",   -80.0)
        self.pitch_min_deg = rospy.get_param("~ee_orientation_limits/pitch_min",  -30.0)
        self.pitch_max_deg = rospy.get_param("~ee_orientation_limits/pitch_max",   30.0)
        self.yaw_min_deg   = rospy.get_param("~ee_orientation_limits/yaw_min",    150.0)
        self.yaw_max_deg   = rospy.get_param("~ee_orientation_limits/yaw_max",    210.0)

        rospy.loginfo(
            "EE orientation limits (deg): "
            "roll[%.1f, %.1f], pitch[%.1f, %.1f], yaw[%.1f, %.1f]",
            self.roll_min_deg,  self.roll_max_deg,
            self.pitch_min_deg, self.pitch_max_deg,
            self.yaw_min_deg,   self.yaw_max_deg
        )

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
        # self.wait_for_planning_scene_object("table")

        return obj_pose

    # >>> NEW: 夹爪角度过滤函数
    def filter_grasps_by_orientation(self, grasps):
        """
        按 roll/pitch/yaw 限制过滤 SphericalGrasps 生成的 grasps。
        限制在度的范围内，超出范围的 grasp 直接丢弃。
        """
        filtered = []

        for g in grasps:
            q = g.grasp_pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]

            # tf 返回的是弧度
            roll, pitch, yaw = tf_trans.euler_from_quaternion(quat)

            roll_deg  = roll  * 180.0 / math.pi
            pitch_deg = pitch * 180.0 / math.pi
            yaw_deg   = yaw   * 180.0 / math.pi

            # 判断是否在可接受范围内
            if (self.roll_min_deg  <= roll_deg  <= self.roll_max_deg and
                self.pitch_min_deg <= pitch_deg <= self.pitch_max_deg and
                self.yaw_min_deg   <= yaw_deg   <= self.yaw_max_deg):
                filtered.append(g)

        rospy.loginfo("Grasp orientation filter: %d -> %d grasps",
                      len(grasps), len(filtered))

        # 如果全被过滤掉，为了不完全瘫痪，先回退到原 grasps（你也可以选择直接失败）
        if len(filtered) == 0:
            rospy.logwarn("All grasps filtered out by EE orientation limits, "
                          "falling back to unfiltered grasps. "
                          "Consider relaxing ee_orientation_limits.")
            return grasps

        return filtered

    def plan_and_pick(self, object_pose):
        # 1) 更新场景
        obj_pose_scene = self.add_part_and_table(object_pose)

        # 2) 生成多组抓取姿态
        rospy.loginfo("Creating grasps from object pose ...")
        possible_grasps = self.sg.create_grasps_from_object_pose(obj_pose_scene)
        rospy.loginfo("Generated %d grasps (before EE orientation filter).",
                      len(possible_grasps))

        # 先按 EE 姿态范围过滤
        possible_grasps = self.filter_grasps_by_orientation(possible_grasps)

        # >>> 在这里加一行：统一转 90°，把手指从“上下”转成“左右”
        possible_grasps = self.rotate_grasps_fix_finger_dir(
            possible_grasps,
            angle_deg=90.0  # 不对就改成 -90.0 或者改成绕 z 轴，下面有说明
        )

        # 3) 构造 PickupGoal
        goal = create_pickup_goal(
            group="arm_torso",
            target="part",
            grasp_pose=obj_pose_scene,
            possible_grasps=possible_grasps,
            links_to_allow_contact=self.links_to_allow_contact
        )

        goal.support_surface_name = "table"
        goal.allow_gripper_support_collision = True

        # 4) 调用 /pickup
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

    def rotate_grasps_fix_finger_dir(self, grasps, angle_deg=90.0):
        """
        把所有 grasp 的末端姿态再绕“工具坐标系的 x 轴”旋转 angle_deg（默认 90°），
        用来把手指从“上下夹”转成“左右夹”。

        如果旋转方向反了，物体前后反了，可以把 angle_deg 改成 -90 试一下。
        """
        angle = math.radians(angle_deg)
        # 绕 x 轴旋转 angle
        q_fix = tf_trans.quaternion_from_euler(angle, 0.0, 0.0)

        for g in grasps:
            q = g.grasp_pose.pose.orientation
            q_orig = [q.x, q.y, q.z, q.w]

            # 注意乘法顺序：这里假设是在当前姿态的基础上，再转一圈
            q_new = tf_trans.quaternion_multiply(q_orig, q_fix)

            q.x, q.y, q.z, q.w = q_new

        rospy.loginfo("Applied fixed rotation of %.1f deg around tool x-axis to %d grasps",
                      angle_deg, len(grasps))

        return grasps

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
