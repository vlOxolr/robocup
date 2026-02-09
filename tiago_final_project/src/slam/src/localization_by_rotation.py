#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Improve robot localization by rotating in place
Rotates until pose estimation error (covariance) is below threshold
Based on tiago_localization C++ implementation
"""

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty


class LocalizationByRotation:
    def __init__(self):
        rospy.init_node('localization_by_rotation', anonymous=True)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Localization state
        self.covariance_sum = 0.0
        self.pose_received = False
        self.localization_complete = False

        # Parameters - read from ROS parameter server
        self.spin_speed = rospy.get_param('spin_speed', 1)  # rad/s
        self.threshold = rospy.get_param('threshold', 0.06)  # Covariance sum threshold
        self.max_rotation_time = rospy.get_param('~max_rotation_time', 100.0)  # Maximum rotation time (seconds)

        rospy.loginfo("="*60)
        rospy.loginfo("Localization by Rotation Node Started")
        rospy.loginfo("Spin speed: %.2f rad/s" % self.spin_speed)
        rospy.loginfo("Covariance threshold: %.4f" % self.threshold)
        rospy.loginfo("Max rotation time: %.1f seconds" % self.max_rotation_time)
        rospy.loginfo("="*60)

        # Call global localization service
        try:
            rospy.wait_for_service('/global_localization', timeout=5.0)
            global_loc_service = rospy.ServiceProxy('/global_localization', Empty)
            global_loc_service()
            rospy.loginfo("Global localization initialized.")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("Failed to call /global_localization service: %s" % str(e))
            rospy.logwarn("Continuing without global localization...")

    def pose_callback(self, msg):
        """
        Callback for AMCL pose updates
        Calculate covariance sum and check if below threshold
        """
        self.pose_received = True

        # Compute time difference between now and message timestamp
        time_diff = rospy.Time.now() - msg.header.stamp

        # Only process recent messages (within 0.5 seconds)
        if time_diff.to_sec() < 0.5:
            # Calculate sum of all covariance values (36 elements)
            self.covariance_sum = sum(msg.pose.covariance)

            rospy.loginfo_throttle(2.0, "Covariance sum: %.6f (threshold: %.6f)" %
                                  (self.covariance_sum, self.threshold))

            # Check if localization is good enough
            if self.covariance_sum < self.threshold:
                rospy.loginfo("\n" + "="*60)
                rospy.loginfo("✓ Localization successful!")
                rospy.loginfo("Covariance sum: %.6f (below threshold: %.6f)" %
                            (self.covariance_sum, self.threshold))
                rospy.loginfo("="*60)
                self.localization_complete = True

    def robot_spin(self):
        """
        Rotate robot in place until localization improves
        Continuously publishes velocity commands and monitors covariance
        """
        rospy.loginfo("\nWaiting for initial AMCL pose...")

        # Wait for first pose estimate
        while not self.pose_received and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("Initial pose received")
        initial_covariance = self.covariance_sum
        rospy.loginfo("Initial covariance sum: %.6f" % initial_covariance)

        rospy.loginfo("\nStarting rotation for localization improvement...")
        rospy.loginfo("Target covariance sum: < %.6f" % self.threshold)

        # Create velocity command
        twist = Twist()
        twist.angular.z = self.spin_speed

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # Publish rotation command
            self.cmd_vel_pub.publish(twist)

            # Check if localization is complete
            if self.localization_complete:
                break

            # Check timeout
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > self.max_rotation_time:
                rospy.logwarn("\n" + "="*60)
                rospy.logwarn("⚠ Max rotation time reached!")
                rospy.logwarn("Current covariance sum: %.6f (target: %.6f)" %
                            (self.covariance_sum, self.threshold))
                rospy.logwarn("="*60)
                break

            # Spin once to process callbacks
            rate.sleep()

        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Rotation stopped")

        return self.localization_complete

    def run(self):
        """Main execution"""
        try:
            rospy.loginfo("Publishing robot velocity commands...")
            success = self.robot_spin()

            if success:
                rospy.loginfo("✓ Localization completed successfully!")
            else:
                rospy.logwarn("✗ Localization did not reach target threshold")

        except rospy.ROSInterruptException:
            rospy.loginfo("Localization interrupted")
        finally:
            # Ensure robot stops
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)


def main():
    try:
        localizer = LocalizationByRotation()
        localizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Localization interrupted")
    except Exception as e:
        rospy.logerr("Error: %s" % str(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
