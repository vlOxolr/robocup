#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Navigate from current position to specified locations
Supports 4 functions:
1. Current position -> entrance
2. Current position -> table_1
3. Current position -> table_2
4. Current position -> kitchen_bar
"""

import rospy
import actionlib
import yaml
import sys
import os
import rospkg
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class SimpleNavigator:
    def __init__(self):
        # Initialize clients
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.torso_client = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        rospy.loginfo("Waiting for action servers...")
        self.nav_client.wait_for_server()
        self.head_client.wait_for_server()
        self.torso_client.wait_for_server()
        rospy.loginfo("Connected to servers")

        # Set torso lift to 0.05m at initialization
        self.set_torso_height(0.05)
        rospy.loginfo("Torso lift set to 0.05m")
        
        # Load location configuration
        config_file = self._find_config_file()
        with open(config_file, 'r') as f:
            self.locations = yaml.safe_load(f)
    
    def _find_config_file(self):
        """Find configuration file"""
        candidates = [
            rospy.get_param('~location_config', ''),
            # Fallback to config next to the package when running from source
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'restaurant_locations.yaml'),
        ]
        
        try:
            rp = rospkg.RosPack()
            pkg_path = rp.get_path('navigation')
            candidates.append(os.path.join(pkg_path, 'config', 'restaurant_locations.yaml'))
        except:
            pass
        
        for path in candidates:
            if path and os.path.exists(path):
                rospy.loginfo("Using config: %s" % path)
                return path
        
        raise IOError("Location config file not found!")

    def set_torso_height(self, height):
        """
        Set torso lift height
        height: target height in meters (0.0 - 0.35)
        """
        torso_goal = FollowJointTrajectoryGoal()
        torso_goal.trajectory.joint_names = ['torso_lift_joint']
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = rospy.Duration(2.0)
        torso_goal.trajectory.points = [point]
        self.torso_client.send_goal_and_wait(torso_goal)

    def navigate_to(self, target_name):
        """
        Navigate from current position to target
        target_name: 'entrance', 'kitchen_bar', 'table_1', 'table_2'
        """
        # Get target location
        if target_name in ['entrance', 'kitchen_bar']:
            loc = self.locations[target_name]
        elif target_name.startswith('table_'):
            loc = self.locations['tables'][target_name]
        else:
            rospy.logerr("Unknown location: %s" % target_name)
            return False
        
        rospy.loginfo("="*50)
        rospy.loginfo("Navigating to: %s" % target_name)
        
        # Create navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(**loc['position'])
        goal.target_pose.pose.orientation = Quaternion(**loc['orientation'])
        
        # Execute navigation
        self.nav_client.send_goal(goal)
        success = self.nav_client.wait_for_result(rospy.Duration(100.0))
        
        if not success:
            rospy.logwarn("Navigation timeout!")
            return False
        
        rospy.loginfo("Arrived at: %s" % target_name)
        
        # Adjust head position
        if 'head' in loc:
            rospy.sleep(0.5)
            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
            point = JointTrajectoryPoint()
            point.positions = [loc['head']['head_1'], loc['head']['head_2']]
            point.time_from_start = rospy.Duration(2.0)
            head_goal.trajectory.points = [point]
            self.head_client.send_goal_and_wait(head_goal)
            rospy.loginfo("Head adjusted")
        
        return True
    
    # ========== 4 navigation functions ==========
    
    def goto_entrance(self):
        """Function 1: Go to entrance"""
        return self.navigate_to('entrance')
    
    def goto_table1(self):
        """Function 2: Go to table 1"""
        return self.navigate_to('table_1')
    
    def goto_table2(self):
        """Function 3: Go to table 2"""
        return self.navigate_to('table_2')
    
    def goto_bar(self):
        """Function 4: Go to kitchen bar"""
        return self.navigate_to('kitchen_bar')


def main():
    rospy.init_node('simple_navigator')
    
    # Function mapping
    functions = {
        '1': ('goto_entrance', 'Go to entrance'),
        '2': ('goto_table1', 'Go to table 1'),
        '3': ('goto_table2', 'Go to table 2'),
        '4': ('goto_bar', 'Go to kitchen bar')
    }
    
    # Get command line arguments
    if len(sys.argv) < 2:
        print("\nUsage: rosrun SLAM simple_navigator.py <function_number>")
        print("\nAvailable functions:")
        for key, (func, desc) in functions.items():
            print("  %s - %s" % (key, desc))
        sys.exit(1)
    
    function_num = sys.argv[1]
    
    # Check function number
    if function_num not in functions:
        rospy.logerr("Invalid function number: %s (use 1-4)" % function_num)
        sys.exit(1)
    
    # Execute function
    func_name, desc = functions[function_num]
    rospy.loginfo("Executing function %s: %s" % (function_num, desc))
    
    nav = SimpleNavigator()
    method = getattr(nav, func_name)
    
    if method():
        rospy.loginfo("✓ Function completed successfully!")
    else:
        rospy.logerr("✗ Function failed!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass