cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

find_package(catkin REQUIRED
  rospy
  smach
  smach_ros
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/state_machine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
