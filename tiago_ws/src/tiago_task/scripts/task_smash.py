#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

#from navigation_client import NavigationClient
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
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
            rospy.loginfo("[SMACH] GoTo(%s) succeeded", self.label)
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
        )

        # 创建 Action Client
        rospy.loginfo("[PickBottle] Waiting for /pickup_pose action server...")
        self.client = SimpleActionClient("/pickup_pose", PickUpPoseAction)
        self.client.wait_for_server()
        rospy.loginfo("[PickBottle] Connected to /pickup_pose.")

    def execute(self, userdata):
        obj_pose = userdata.object_pose
        if obj_pose is None:
            rospy.logerr("[PickBottle] No object_pose available")
            return "failed"

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
    table_A = [0.366, -0.534, -0.524]
    table_B = [-0.821, -0.468, 2.617]

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])
    sm.userdata.object_pose = None

    with sm:
        # 1. 到桌子 A
        smach.StateMachine.add(
            "GOTO_A",
            GoTo(nav, table_A, label="table_A"),
            transitions={
                "succeeded": "DETECT_BOTTLE",
                "failed": "FAILED",
            },
        )

        # 2. 找 /text_markers 中 bottle 的位姿
        smach.StateMachine.add(
            "DETECT_BOTTLE",
            DetectBottleFromMarkers(topic_name="/text_markers",
                                    target_label="bottle",
                                    timeout=5.0),
            transitions={
                "succeeded": "PICK_BOTTLE",
                "failed": "FAILED",
            },
        )

        # 3. 调用抓取 action server
        smach.StateMachine.add(
            "PICK_BOTTLE",
            PickBottle(),
            transitions={
                "succeeded": "GOTO_B",
                "failed": "FAILED",
            },
        )

        # 4. 拿着物体走到桌子 B
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
