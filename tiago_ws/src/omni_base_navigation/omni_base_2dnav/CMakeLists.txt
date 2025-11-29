cmake_minimum_required(VERSION 3.0.2)
project(omni_base_2dnav)

find_package(catkin REQUIRED)

catkin_package()

# install(
#     PROGRAMS
#         scripts/record_mapping.sh
#         scripts/record_move_base.sh
#         scripts/record_odom.sh
#         scripts/record_patrol.sh
#         scripts/record_rgbd.sh
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
