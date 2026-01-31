cmake_minimum_required(VERSION 3.0.2)
project(wave)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cv_bridge
  pal_interaction_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/yolo_wave_tiago2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
