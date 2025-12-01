# RoboCup@Home Tutorial 5


## Launch rviz and Gazebo

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=wrs2020 gzpose:="-x -1.565665 -y 0.576238 -z 0.0 -Y 0.398122" map:=/tiago_ws/src/tmc_wrs_gazebo_world/maps/wrs2020

# in new terminal
roslaunch tiago_moveit_config moveit_planning_execution.launch   public_sim:=true pipeline:=ompl
```


## State Machine

```bash
roslaunch tiago_task task_orchestrator.launch 
```

### 1 target navigation

```bash
roslaunch tiago_moveit_config moveit_planning_execution.launch \
  public_sim:=true pipeline:=ompl


roslaunch tiago_move tiago_move_1tg.launch
```

## Look down

```bash
rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "
joint_names: ['head_1_joint', 'head_2_joint']
points:
- positions: [0.0, -0.6]
  time_from_start: {secs: 2}
" -1
```
