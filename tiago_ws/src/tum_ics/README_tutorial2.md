# TIAGo — RoboCup@Home — Tutorial 2
# Member
# Shuowen Li
# Bangdong Zhang

---

## Exercise 6.1 – Create a Simulation Scenario
**Robot:** TIAGo  
**Packages Used:** `ics_gazebo`

### Objective
Build and save a Gazebo world including at least the **door** and **tv** models 

### Procedure
```bash
# Enter container & set environment
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
source ~/ros/workspaces/tiago_ws/devel/setup.bash

# Clean old sessions
pkill -f gzserver; pkill -f gzclient; pkill -f roslaunch || true
rm -rf ~/.ros/log

# Enable stable software rendering (inside Docker)
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=FBO

# Open Gazebo world editor
roslaunch ics_gazebo edit_world.launch world_suffix:=empty
```



## Exercise 6.2 – Create an RViz Configuration File
**Robot:** TIAGo  
**Packages Used:** `ics_gazebo`

### Objective
Save a visualization layout that displays the robot model and sensors in RViz.

### Procedure
```bash
# Start simulation
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
source ~/ros/workspaces/tiago_ws/devel/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=FBO

roslaunch ics_gazebo tiago.launch world_suffix:=tutorial2
```

In a new terminal:
```bash
rosrun rviz rviz
```
Then save the layout as:
```
/root/ros/workspaces/tiago_ws/src/tum_ics/ics_gazebo/config/tiago.rviz
```

Verify:
```bash
ls ~/ros/workspaces/tiago_ws/src/tum_ics/ics_gazebo/config/tiago.rviz
head -n 10 ~/ros/workspaces/tiago_ws/src/tum_ics/ics_gazebo/config/tiago.rviz
```
RViz will automatically load this layout when launching `tiago.launch`.

---

## Exercise 6.3 – Create a Controller Plugin
**Robot:** TIAGo  
**Packages Used:** `controllers_tutorials`

### Objective
Create and test a custom controller plugin controlling TIAGo’s head joints.

### Procedure
```bash
# Enter container & environment
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
source ~/ros/workspaces/tiago_ws/devel/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=FBO
```


```



#### Load and test controller in simulation
Launch Gazebo + TIAGo (first terminal):
```bash
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial2
```

Control plugin (second terminal):
```bash
# Load parameters
rosparam load $(rospack find controllers_tutorials)/config/new_head_controller_tiago.yaml

# Stop default head controller
rosservice call /controller_manager/switch_controller "start_controllers: []
stop_controllers: ['head_controller']
strictness: 2"

# Load and start new controller
rosservice call /controller_manager/load_controller "name: 'new_head_controller'"
rosservice call /controller_manager/switch_controller "start_controllers: ['new_head_controller']
stop_controllers: []
strictness: 2"

# Check status
rosservice call /controller_manager/list_controllers
```
should see:
```
name: "new_head_controller"
state: "running"
type: "controllers_tutorials/NewHeadController"
```
and the robot’s head moves slightly.

#### 6 – Stop and restore default controller
```bash
rosservice call /controller_manager/switch_controller "start_controllers: []
stop_controllers: ['new_head_controller']
strictness: 2"
rosservice call /controller_manager/unload_controller "name: 'new_head_controller'"

rosservice call /controller_manager/switch_controller "start_controllers: ['head_controller']
stop_controllers: []
strictness: 2"
```



**End of Tutorial 2 README**
