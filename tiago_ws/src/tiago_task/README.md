# RoboCup@Home Tutorial 5

## Member

Bangdong Zhang & Shuowen Li

## Startup

run following commands in every terminal.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## Launch rviz and Gazebo

Terminal 1:

```bash
roslaunch tiago_task tiago_nav_vision.launch map:=/tiago_ws/src/tmc_wrs_gazebo_world/maps/wrs2020 end_effector:=pal-gripper
```

## Setup head pose

look down

```bash
rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "
joint_names: ['head_1_joint', 'head_2_joint']
points:
- positions: [0.0, -0.6]
  time_from_start: {secs: 2}
" -1
```

## Enable moveit service

Terminal 2:

```bash
roslaunch tiago_task task_moveit.launch public_sim:=true pipeline:=ompl
```

## Build communication between 2 containers

replace <name_of_tiago_container> and <name_of_cuda_name> to its real name.

In tiago container:

```bash
export ROS_MASTER_URI=http://<name_of_tiago_container>:11311
export ROS_HOSTNAME=<name_of_tiago_container>
```

In cuda container:

```bash
export ROS_MASTER_URI=http://<name_of_tiago_container>:11311
export ROS_HOSTNAME=<name_of_cuda_name>
```

## Enable YOLO

In tiago container (Terminal 3):

```bash
roslaunch object_detection object_detection.launch image:="/xtion/rgb/image_raw"
```

## Enable segmentation and labeling

Terminal 4:

```bash
rosrun plane_segmentation plane_segmentation_node
```

Terminal 5:

```bash
rosrun object_labeling object_labeling_node
```

## Enable gripper planning

Terminal 6:

```bash
roslaunch tiago_task pick_and_place_server.launch
```

## Task 1: move bottle from table to another table

Terminal 7:

```bash
roslaunch tiago_task task_smash.launch 
```

## Task 2: find bottle in office

Terminal 7:

```bash
roslaunch tiago_task task_finder.launch 
```
