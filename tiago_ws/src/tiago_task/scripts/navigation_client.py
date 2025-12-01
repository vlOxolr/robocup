#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
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
