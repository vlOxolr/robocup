# RoboCup@Home

# 硕哥nb！

# 栋子哥nb！

## How to run our world+tiago in rviz and Gazebo

```bash
# 清理上次仿真残留
pkill -f gzserver; pkill -f gzclient; pkill -f roslaunch || true
rm -rf ~/.ros/log

export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=FBO

roslaunch tmc_wrs_gazebo_worlds tiago_wrs.launch
```

## Create map

```bash
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true world:=wrs2020 gzpose:="-x -1.565665 -y 0.576238 -z 0.0 -Y 0.398122"

rosrun key_teleop key_teleop.py

rosservice call /pal_map_manager/save_map "xxx"
```


## structure

- main_ws: tiago workspace

  docker command：

  ```bash
  docker run -it \
    --name ros_container \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/path/to/the/repository/tiago_ws:/tiago_ws" \
    --workdir="/tiago_ws" \
    palroboticssl/tiago_tutorials:noetic
  ```

- yolo_ws: yolo workspace

## Commit rule

**repair:** fix issues from original code

**forward:** add new runable implementation

**debug:** debug
