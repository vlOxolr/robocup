#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from navigation_client import NavigationClient


class TaskOrchestrator(object):
    def __init__(self):
        self.nav = NavigationClient()

    def run(self):
        rospy.loginfo("Task started.")

        table_A = [0.366, -0.534, -0.524]
        table_B = [-0.821, -0.468, 2.617]

        rospy.loginfo("Going to table A...")
        if not self.nav.goto(*table_A):
            rospy.logerr("Failed to reach table A.")
            return

        rospy.loginfo("Going to table B...")
        if not self.nav.goto(*table_B):
            rospy.logerr("Failed to reach table B.")
            return

        rospy.loginfo("Navigation task finished.")


if __name__ == "__main__":
    rospy.init_node("task_orchestrator")
    orchestrator = TaskOrchestrator()
    orchestrator.run()
