#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

from navigation_client import NavigationClient


# ---------------------------------------------------------
# State: GoTo
# ---------------------------------------------------------
class GoTo(smach.State):
    """
    SMACH 状态：导航到指定 (x, y, yaw)
    """

    def __init__(self, nav_client, target, label=""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.nav = nav_client
        self.target = target
        self.label = label

    def execute(self, userdata):
        rospy.loginfo("[SMACH] Executing GoTo(%s): %s", self.label, str(self.target))

        ok = self.nav.goto(self.target[0], self.target[1], self.target[2])

        if ok:
            rospy.loginfo("[SMACH] Navigation succeeded: %s", self.label)
            return 'succeeded'
        else:
            rospy.logerr("[SMACH] Navigation failed: %s", self.label)
            return 'failed'


# ---------------------------------------------------------
# SMACH 主状态机
# ---------------------------------------------------------
def main():
    rospy.init_node("task_smach")

    nav = NavigationClient()

    # 目标点
    table_A = [0.366, -0.534, -0.524]
    table_B = [-0.821, -0.468, 2.617]

    sm = smach.StateMachine(outcomes=['DONE', 'FAILED'])

    with sm:
        # START -> GOTO_A
        smach.StateMachine.add(
            'GOTO_A',
            GoTo(nav, table_A, label="table_A"),
            transitions={
                'succeeded': 'GOTO_B',
                'failed': 'FAILED'
            }
        )

        # GOTO_A -> GOTO_B -> DONE
        smach.StateMachine.add(
            'GOTO_B',
            GoTo(nav, table_B, label="table_B"),
            transitions={
                'succeeded': 'DONE',
                'failed': 'FAILED'
            }
        )

    # 启动 SMACH 可视化（可选）
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    result = sm.execute()

    sis.stop()
    rospy.loginfo("[SMACH] Final result: %s", result)


if __name__ == "__main__":
    main()
