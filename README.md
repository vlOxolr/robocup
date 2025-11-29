# RoboCup@Home
# 栋子哥nb！

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
