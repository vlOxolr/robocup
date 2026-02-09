#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    # Initialize ROS node
    rospy.init_node('tilt_head_node')

    # Create action client for Tiago head controller
    # NOTE: For Tiago, the action server name is usually:
    #       /head_controller/follow_joint_trajectory
    client = actionlib.SimpleActionClient(
        '/head_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )

    rospy.loginfo("Waiting for /head_controller/follow_joint_trajectory action server...")
    client.wait_for_server()
    rospy.loginfo("Head controller is available.")

    # Create goal
    goal = FollowJointTrajectoryGoal()
    traj = JointTrajectory()

    # Tiago head joints
    traj.joint_names = ['head_1_joint', 'head_2_joint']

    # Define a single trajectory point
    point = JointTrajectoryPoint()
    # head_1_joint: yaw 
    # head_2_joint: pitch 
    # Here we keep yaw = 0.0 and look down with pitch = -0.7 rad
    point.positions = [0.0, -0.7]

    # Optional: set velocities to zero (controller will interpolate)
    point.velocities = [0.0, 0.0]

    # Time to reach the target (2 seconds)
    point.time_from_start = rospy.Duration(2.0)

    traj.points.append(point)

    # Set header stamp slightly in the future
    traj.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

    goal.trajectory = traj

    # Send goal
    rospy.loginfo("Sending head trajectory goal...")
    client.send_goal(goal)

    # Wait for the motion to finish
    client.wait_for_result()
    rospy.loginfo("Head motion finished.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
