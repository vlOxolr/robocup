#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
简单放置物体脚本：导航 -> 移动手臂 -> 松开夹爪 -> 收回手臂
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import subprocess

class SimplePlacer:
    def __init__(self):
        rospy.init_node('simple_placer')

        self.goal_pose = None

        # 订阅 rviz 2D Nav Goal
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # 导航客户端
        self.nav_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航...")
        self.nav_client.wait_for_server()

        # 手臂控制客户端
        self.arm_client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("等待手臂控制器...")
        self.arm_client.wait_for_server()

        # 夹爪控制客户端
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("等待夹爪控制器...")
        self.gripper_client.wait_for_server()

        rospy.loginfo("初始化完成")

    def goal_callback(self, msg):
        """保存 2D Nav Goal"""
        self.goal_pose = msg

    def navigate_to_target(self, x, y, oz=-0.736, ow=0.677):
        """导航到目标位置"""
        rospy.loginfo("导航到: [%.2f, %.2f]" % (x, y))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = oz
        goal.target_pose.pose.orientation.w = ow

        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()
        rospy.loginfo("✓ 导航完成")

    def move_arm_to_position(self):
        """移动手臂到指定位置"""
        rospy.loginfo("移动手臂...")

        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.36, -0.50, -1.27, 1.74, 0.82, 0.68, 1.76]
        point.time_from_start = rospy.Duration(3.0)

        traj.points.append(point)
        goal.trajectory = traj

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        rospy.loginfo("✓ 手臂到位")

    def open_gripper(self):
        """松开夹爪"""
        rospy.loginfo("松开夹爪...")

        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.time_from_start = rospy.Duration(2.0)

        traj.points.append(point)
        goal.trajectory = traj

        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        rospy.loginfo("✓ 夹爪已松开")

    def tuck_arm(self):
        """收回手臂"""
        rospy.loginfo("收回手臂...")
        subprocess.call(['rosrun', 'tiago_gazebo', 'tuck_arm.py'])
        rospy.loginfo("✓ 手臂已收回")

    def place(self, nav_x, nav_y):
        """完整放置流程"""
        rospy.loginfo("="*60)
        rospy.loginfo("开始放置物体")
        rospy.loginfo("="*60)

        # 1. 导航
        self.navigate_to_target(nav_x, nav_y)
        rospy.sleep(1.0)

        # 2. 移动手臂
        self.move_arm_to_position()
        rospy.sleep(1.0)

        # 3. 松开夹爪
        self.open_gripper()
        rospy.sleep(0.5)

        # 4. 收回手臂
        self.tuck_arm()

        rospy.loginfo("="*60)
        rospy.loginfo("✓ 放置完成")
        rospy.loginfo("="*60)


def main():
    placer = SimplePlacer()

    # 固定放置位置
    x = 2.10
    y = 0.15

    placer.place(x, y)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
