#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

import subprocess
import time

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from rospy.exceptions import ROSException


# -----------------------------
# NavigationClient （不变）
# -----------------------------
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
            rospy.loginfo("[SMACH] GoTo(%s:[%f,%f,%f]) succeeded",
                          self.label, self.target[0], self.target[1], self.target[2])
            return "succeeded"
        else:
            rospy.logerr("[SMACH] GoTo(%s) failed", self.label)
            return "failed"


# -----------------------------
# State: WaitStable (静止等待图像稳定)
# -----------------------------
class WaitStable(smach.State):
    def __init__(self, duration=3.0, label=""):
        smach.State.__init__(self, outcomes=["done"])
        self.duration = duration
        self.label = label

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State WaitStable(%s): sleep %.1f s",
                      self.label, self.duration)
        rospy.sleep(self.duration)
        return "done"


# -----------------------------
# State: DetectBottle (从 MarkerArray 里找 ns=='bottle')
# -----------------------------
class DetectBottle(smach.State):
    """
    在当前位置尝试检测瓶子：
    - 从 /item_markers (MarkerArray) 里找 ns == 'bottle' 的 marker
    - 找到至少一个：认为找到瓶子，取第一个 marker 的位姿，返回 found
    - 超时或数组里没有 ns=='bottle'：返回 not_found
    """
    def __init__(self,
                 topic="/item_markers",
                 timeout=5.0,
                 label=""):
        smach.State.__init__(self,
                             outcomes=["found", "not_found"],
                             output_keys=["bottle_pose"])
        self.topic = topic
        self.timeout = timeout
        self.label = label

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State DetectBottle(%s): wait for MarkerArray from %s (timeout=%.1f s)",
                      self.label, self.topic, self.timeout)
        try:
            marker_array = rospy.wait_for_message(self.topic,
                                                  MarkerArray,
                                                  timeout=self.timeout)
        except ROSException:
            rospy.loginfo("[SMACH] DetectBottle(%s): no MarkerArray received (timeout).",
                          self.label)
            return "not_found"

        # 找 ns == "bottle" 的 marker
        bottle_markers = [m for m in marker_array.markers if m.ns == "bottle"]

        if not bottle_markers:
            rospy.loginfo("[SMACH] DetectBottle(%s): MarkerArray received but no ns=='bottle' marker.",
                          self.label)
            return "not_found"

        # 取第一个 bottle marker，当作瓶子位置
        bottle_marker = bottle_markers[0]
        p = bottle_marker.pose.position

        # 转成 PoseStamped 存到 userdata
        pose_stamped = PoseStamped()
        pose_stamped.header = bottle_marker.header
        pose_stamped.pose = bottle_marker.pose
        userdata.bottle_pose = pose_stamped

        rospy.loginfo("[SMACH] DetectBottle(%s): bottle marker found. Position = (%.3f, %.3f, %.3f)",
                      self.label, p.x, p.y, p.z)

        return "found"


# -----------------------------
# State: ReportFound / ReportNotFound
# -----------------------------
class ReportFound(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["done"],
                             input_keys=["bottle_pose"])

    def execute(self, userdata):
        pose = userdata.bottle_pose.pose
        p = pose.position
        rospy.loginfo("===================================")
        rospy.loginfo("  BOTTLE FOUND")
        rospy.loginfo("  Position: x=%.3f, y=%.3f, z=%.3f", p.x, p.y, p.z)
        rospy.loginfo("===================================")
        return "done"


class ReportNotFound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"])

    def execute(self, userdata):
        rospy.logwarn("===================================")
        rospy.logwarn("  BOTTLE NOT FOUND AT ANY POSITION")
        rospy.logwarn("===================================")
        return "done"


# -----------------------------
# main：四个点巡逻找瓶子
# -----------------------------
def main():
    rospy.init_node("task_smash_find_bottle")

    nav = NavigationClient()

    # ===== 四个寻找点（自己根据实际改数值）=====
    point_1 = [0.28, -0.39, -0.524]    # table_A
    point_2 = [-0.88, -0.95, 2.617]    # table_B
    point_3 = [2.19, -2.69, -0.524]     # cabinet
    point_4 = [-0.42, -0.11, 2.700]        # living room

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])
    sm.userdata.bottle_pose = PoseStamped()

    with sm:
        # ---------- 点 1 ----------
        smach.StateMachine.add(
            "GOTO_1",
            GoTo(nav, point_1, label="point_1"),
            transitions={
                "succeeded": "WAIT_1",
                "failed": "FAILED",
            },
        )

        smach.StateMachine.add(
            "WAIT_1",
            WaitStable(duration=3.0, label="point_1"),
            transitions={
                "done": "CHECK_1",
            },
        )

        smach.StateMachine.add(
            "CHECK_1",
            DetectBottle(topic="/item_markers", timeout=5.0, label="point_1"),
            transitions={
                "found": "REPORT_FOUND",
                "not_found": "GOTO_2",
            },
        )

        # ---------- 点 2 ----------
        smach.StateMachine.add(
            "GOTO_2",
            GoTo(nav, point_2, label="point_2"),
            transitions={
                "succeeded": "WAIT_2",
                "failed": "FAILED",
            },
        )

        smach.StateMachine.add(
            "WAIT_2",
            WaitStable(duration=3.0, label="point_2"),
            transitions={
                "done": "CHECK_2",
            },
        )

        smach.StateMachine.add(
            "CHECK_2",
            DetectBottle(topic="/item_markers", timeout=5.0, label="point_2"),
            transitions={
                "found": "REPORT_FOUND",
                "not_found": "GOTO_3",
            },
        )

        # ---------- 点 3 ----------
        smach.StateMachine.add(
            "GOTO_3",
            GoTo(nav, point_3, label="point_3"),
            transitions={
                "succeeded": "WAIT_3",
                "failed": "FAILED",
            },
        )

        smach.StateMachine.add(
            "WAIT_3",
            WaitStable(duration=3.0, label="point_3"),
            transitions={
                "done": "CHECK_3",
            },
        )

        smach.StateMachine.add(
            "CHECK_3",
            DetectBottle(topic="/item_markers", timeout=5.0, label="point_3"),
            transitions={
                "found": "REPORT_FOUND",
                "not_found": "GOTO_4",
            },
        )

        # ---------- 点 4 ----------
        smach.StateMachine.add(
            "GOTO_4",
            GoTo(nav, point_4, label="point_4"),
            transitions={
                "succeeded": "WAIT_4",
                "failed": "FAILED",
            },
        )

        smach.StateMachine.add(
            "WAIT_4",
            WaitStable(duration=3.0, label="point_4"),
            transitions={
                "done": "CHECK_4",
            },
        )

        smach.StateMachine.add(
            "CHECK_4",
            DetectBottle(topic="/item_markers", timeout=5.0, label="point_4"),
            transitions={
                "found": "REPORT_FOUND",
                "not_found": "REPORT_NOT_FOUND",   # 四个点都没找到
            },
        )

        # ---------- 结果 ----------
        smach.StateMachine.add(
            "REPORT_FOUND",
            ReportFound(),
            transitions={"done": "DONE"},
        )

        smach.StateMachine.add(
            "REPORT_NOT_FOUND",
            ReportNotFound(),
            transitions={"done": "DONE"},
        )

    # SMACH 可视化（可选）
    sis = smach_ros.IntrospectionServer("find_bottle_smach", sm, "/FIND_BOTTLE_SM")
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("State machine finished with outcome: %s", outcome)

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
