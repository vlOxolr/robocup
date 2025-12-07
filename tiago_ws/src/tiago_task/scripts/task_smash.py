#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


#from navigation_client import NavigationClient
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from actionlib import SimpleActionClient
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal


import subprocess
import time


class NavigationClient(object):
    """
    调度器级 NavigationClient
    """

    def __init__(self, pkg="tiago_move", launch_file="tiago_move_1tg.launch"):
        self.pkg = pkg
        self.launch_file = launch_file

    def goto(self, x, y, yaw):
        """
        执行：底盘导航到 (x, y, yaw)
        """
        rospy.set_param("/tiago_move_1tg/target_position", [x, y, yaw])
        rospy.loginfo("Set navigation target: [%.3f, %.3f, %.3f]" % (x, y, yaw))

        cmd = ["roslaunch", self.pkg, self.launch_file]
        p = subprocess.Popen(cmd)

        while p.poll() is None:
            time.sleep(0.5)

        if p.returncode == 0:
            rospy.loginfo("Navigation succeeded.")
            return True
        else:
            rospy.logwarn("Navigation failed.")
            return False


# -----------------------------
# State: GoTo (导航到底盘目标)
# -----------------------------
class GoTo(smach.State):
    def __init__(self, nav_client, target, label=""):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.nav = nav_client
        self.target = target      # [x, y, yaw]
        self.label = label

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State GoTo(%s), target=%s", self.label, self.target)

        ok = self.nav.goto(self.target[0], self.target[1], self.target[2])

        if ok:
            rospy.loginfo("[SMACH] GoTo(%s:[%f,%f,%f]) succeeded", self.label, self.target[0], self.target[1], self.target[2])
            return "succeeded"
        else:
            rospy.logerr("[SMACH] GoTo(%s) failed", self.label)
            return "failed"


# -----------------------------
# State: DetectBottleFromMarkers
# 从 /text_markers 找到名为 bottle 的物体位姿
# -----------------------------
class DetectBottleFromMarkers(smach.State):
    """
    通过 /text_markers 话题找到 bottle 的位姿。
    这里假定 /text_markers 类型为 visualization_msgs/MarkerArray，
    且其中某个 marker 的 text 字段为 "bottle"，pose 即物体位置。
    """

    def __init__(self,
                 topic_name="/text_markers",
                 target_label="bottle",
                 timeout=5.0):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["object_pose"],
        )
        self.topic_name = topic_name
        self.target_label = target_label
        self.timeout = timeout

        self.bottle_pub = rospy.Publisher(
            "/bottle_pos", Marker, queue_size=1, latch=True
        )

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State DetectBottleFromMarkers: waiting for %s", self.topic_name)

        try:
            markers_msg = rospy.wait_for_message(
                self.topic_name, MarkerArray, timeout=self.timeout
            )
        except rospy.ROSException:
            rospy.logerr("[SMACH] No message on %s within %.1f s",
                         self.topic_name, self.timeout)
            return "failed"

        # 在 markers 里查找 label = bottle 的那个
        bottle_marker = None
        for m in markers_msg.markers:
            # 这一行根据你实际的消息内容改：
            # 有的代码把类别写在 m.text，有的写在 m.ns 或 id，自己对照 rostopic echo 一下即可
            if getattr(m, "text", "") == self.target_label:
                bottle_marker = m
                break

        if bottle_marker is None:
            rospy.logerr("[SMACH] No marker with label '%s' found in /text_markers",
                         self.target_label)
            return "failed"

        # 把 Marker 的 pose 封装成 PoseStamped，后面抓取用
        pose_st = PoseStamped()
        pose_st.header = bottle_marker.header
        pose_st.pose = bottle_marker.pose

        userdata.object_pose = pose_st

        marker = Marker()
        marker.header = bottle_marker.header
        marker.ns = "bottle_pos"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = bottle_marker.pose

        # 大小（自己按需求调）
        marker.scale.x = 0.11
        marker.scale.y = 0.11
        marker.scale.z = 0.20

        # 颜色（红色不透明）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.bottle_pub.publish(marker)
        rospy.loginfo("[SMACH] Published bottle marker on /bottle_pos")

        rospy.loginfo("[SMACH] Found bottle at (%.3f, %.3f, %.3f) in %s",
                      pose_st.pose.position.x,
                      pose_st.pose.position.y,
                      pose_st.pose.position.z,
                      pose_st.header.frame_id)

        return "succeeded"


# -----------------------------
# State: PickObject (机械臂抓取)
# -----------------------------
class PickBottle(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["object_pose"],
            output_keys=["object_pose"],   # 现在自己写 object_pose
        )

        # Action client
        rospy.loginfo("[PickBottle] Waiting for /pickup_pose action server...")
        self.client = SimpleActionClient("/pickup_pose", PickUpPoseAction)
        self.client.wait_for_server()
        rospy.loginfo("[PickBottle] Connected to /pickup_pose.")

        # TF2: 用于把 item_markers 的位姿变到 base_footprint
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_topic = "/item_markers"
        self.target_class = "bottle"
        self.target_frame = "base_footprint"


    def _wait_for_bottle_pose(self):
        """
        循环等待 /item_markers 中出现 ns == 'bottle' 的 marker，
        找到后把 pose 变换到 base_footprint，并返回 PoseStamped。
        """
        rospy.loginfo("[PickBottle] Waiting for '%s' in %s ...",
                        self.target_class, self.marker_topic)

        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(
                    self.marker_topic, MarkerArray, timeout=2.0
                )
            except rospy.ROSException:
                # 超时就继续等
                continue

            bottle_marker = None
            for m in msg.markers:
                # C++ 里：box.ns = assigned_classes[i]
                if m.ns == self.target_class:
                    bottle_marker = m
                    break

            if bottle_marker is None:
                rospy.loginfo("[PickBottle] '%s' not found yet, keep waiting ...",
                                self.target_class)
                continue

            # 先构造原始 pose（在 item_markers 的 header.frame_id 下）
            raw_pose = PoseStamped()
            raw_pose.header = bottle_marker.header
            raw_pose.pose = bottle_marker.pose

            # 去掉可能的前导 '/'，以免 TF 查找失败
            if raw_pose.header.frame_id.startswith("/"):
                raw_pose.header.frame_id = raw_pose.header.frame_id[1:]

            rospy.loginfo(
                    "[PickBottle] Found '%s' at (%.3f, %.3f, %.3f) in %s",
                    self.target_class,
                    raw_pose.pose.position.x,
                    raw_pose.pose.position.y,
                    raw_pose.pose.position.z,
                    raw_pose.header.frame_id,
                )

            # 再用 TF 变换到 base_footprint
            try:
                # 使用最新的 TF
                self.tf_buffer.can_transform(
                    self.target_frame,
                    raw_pose.header.frame_id,
                    rospy.Time(0),
                    timeout=rospy.Duration(1.0)
                )
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    raw_pose.header.frame_id,
                    rospy.Time(0)
                )
                bottle_in_base = do_transform_pose(raw_pose, transform)
                bottle_in_base.header.frame_id = self.target_frame

                rospy.loginfo(
                    "[PickBottle] Found '%s' at (%.3f, %.3f, %.3f) in %s",
                    self.target_class,
                    bottle_in_base.pose.position.x,
                    bottle_in_base.pose.position.y,
                    bottle_in_base.pose.position.z,
                    bottle_in_base.header.frame_id,
                )
                return bottle_in_base

            except tf2_ros.TransformException as e:
                rospy.logwarn("[PickBottle] TF transform %s -> %s failed: %s",
                                raw_pose.header.frame_id, self.target_frame, str(e))
                rospy.sleep(0.1)
                continue

        return None


    def execute(self, userdata):
        # 1) 先从 /item_markers 等到 bottle 的位姿
        obj_pose = self._wait_for_bottle_pose()
        if obj_pose is None:
            rospy.logerr("[PickBottle] Failed to get bottle pose (node shutdown?)")
            return "failed"

        # 记录到 userdata，后面如果还有别的状态要用也方便
        userdata.object_pose = obj_pose

        # 2) 调用抓取 action
        rospy.loginfo("[PickBottle] Sending grasping pose to pick server...")
        goal = PickUpPoseGoal()
        goal.object_pose = obj_pose

        self.client.send_goal(goal)
        self.client.wait_for_result()

        result = self.client.get_result()
        err = result.error_code

        if err == 1:  # MoveItErrorCodes.SUCCESS
            rospy.loginfo("[PickBottle] Grasp SUCCESS")
            return "succeeded"
        else:
            rospy.logerr("[PickBottle] Grasp FAILED, error_code = %s", str(err))
            return "failed"



# -----------------------------
# 主状态机
# -----------------------------
def main():
    rospy.init_node("task_smash")

    nav = NavigationClient()

    # 桌子 A / B 的导航目标
    table_A = [0.15, -0.354, -0.524]
    table_B = [-0.821, -0.468, 2.617]

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])
    sm.userdata.object_pose = None

    with sm:
        # 1. 到桌子 A
        smach.StateMachine.add(
            "GOTO_A",
            GoTo(nav, table_A, label="table_A"),
            transitions={
                "succeeded": "PICK_BOTTLE",
                "failed": "FAILED",
            },
        )

        # 2. 调用抓取 action server
        smach.StateMachine.add(
            "PICK_BOTTLE",
            PickBottle(),
            transitions={
                "succeeded": "GOTO_B",
                "failed": "FAILED",
            },
        )

        # 3. 拿着物体走到桌子 B
        smach.StateMachine.add(
            "GOTO_B",
            GoTo(nav, table_B, label="table_B"),
            transitions={
                "succeeded": "DONE",
                "failed": "FAILED",
            },
        )


    sis = smach_ros.IntrospectionServer("task_smash_server", sm, "/TASK_SMASH")
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("[SMACH] Final outcome: %s", outcome)

    sis.stop()


if __name__ == "__main__":
    main()
