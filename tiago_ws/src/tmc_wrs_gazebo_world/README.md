# RoboCup@Home Tutorial 5

## Launch rviz and Gazebo

Terminal 1:

```bash
roslaunch tiago_task tiago_nav_vision.launch map:=/tiago_ws/src/tmc_wrs_gazebo_world/maps/wrs2020 end_effector:=pal-gripper

# look down
rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "
joint_names: ['head_1_joint', 'head_2_joint']
points:
- positions: [0.0, -0.6]
  time_from_start: {secs: 2}
" -1
```

Terminal 2:

```bash
roslaunch tiago_task task_moveit.launch public_sim:=true pipeline:=ompl
```

##### Reminder

(developing only)

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=wrs2020 gzpose:="-x -1.565665 -y 0.576238 -z 0.0 -Y 0.398122" map:=/tiago_ws/src/tmc_wrs_gazebo_world/maps/wrs2020

# in new terminal
roslaunch tiago_moveit_config moveit_planning_execution.launch public_sim:=true pipeline:=ompl
```

```bash
# for debuging only
roslaunch object_detection_world tiago.launch world_suffix:=wrs2020 robot_pos:="-x -1.565665 -y 0.576238 -z 0.0 -Y 0.398122"

roslaunch object_detection object_detection.launch
```

old moveit launcher

```bash
roslaunch tiago_moveit_config moveit_planning_execution.launch public_sim:=true pipeline:=ompl
```

## Build communication between 2 containers

In ros container:

```bash
export ROS_MASTER_URI=http://ros_container:11311
export ROS_HOSTNAME=ros_container
```

In cuda container:

```bash
export ROS_MASTER_URI=http://ros_container:11311
export ROS_HOSTNAME=yolo_container
```

## Enable YOLO

```bash
roslaunch object_detection object_detection.launch image:="/xtion/rgb/image_raw"
```

## Enable segmentation

```bash
rosrun plane_segmentation plane_segmentation_node
rosrun object_labeling object_labeling_node
```

## Enable gripper planning

```bash
roslaunch tiago_task pick_and_place_server.launch
```

old

```bash
roslaunch tiago_pick_demo pick_demo.launch
```

## Whole task

```bash
roslaunch tiago_task task_smash.launch 
```

### Reminder

(developing only)

- navigation

```bash
roslaunch tiago_moveit_config moveit_planning_execution.launch public_sim:=true pipeline:=ompl

roslaunch tiago_move tiago_move_1tg.launch
```

- Connect containers

```bash
docker network create ros_net
docker network connect ros_net ros_container
docker network connect ros_net yolo_container
```
