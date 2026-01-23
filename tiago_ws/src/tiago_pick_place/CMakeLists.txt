cmake_minimum_required(VERSION 3.0.2)
project(tiago_pick_place)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  moveit_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pick_place_node.py
  scripts/test_trigger_pick.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
