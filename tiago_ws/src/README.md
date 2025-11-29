# RoboCup@Home Tutorial 4
Authors: Shuowen Li, Bangdong Zhang

## Overview
- End-to-end pipeline for TIAGo: object detection (YOLO), plane segmentation, and 2D→3D point cloud labeling.
- Expected RViz visualization: green table plane, red clustered objects, labeled PointCloud2, and text markers with names (e.g., bottle).
- Preview images: `Ergebnis.png`, `Task4.1.png`, `Task4.2.2.png`, `Task4.3.2.png`.

## Prerequisites
- Docker containers: `tiago_build` (ROS Noetic, workspace at `/home/ros/workspaces/tiago_ws`) and `yolo_cuda` (YOLO workspace at `/root/ros/workspaces/yolo_ws`).
- Ensure `tiago_ws` is built and sourced when running TIAGo nodes.
- Set `ROS_MASTER_URI` and `ROS_HOSTNAME` appropriately when crossing containers.

## Task 1 – Object Detection (YOLO)
Terminal 1 – Start TIAGo simulation
```
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
cd /home/ros/workspaces/tiago_ws
source devel/setup.bash
roslaunch object_detection_world tiago.launch
```

Terminal 2 – Position TIAGo’s head toward the table
```
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
source /home/ros/workspaces/tiago_ws/devel/setup.bash
rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "
joint_names: ['head_1_joint', 'head_2_joint']
points:
- positions: [0.0, -0.6]
  time_from_start: {secs: 2}
" -1
```

Terminal 3 – Start YOLO (CUDA container)
```
docker start yolo_cuda
docker exec -it yolo_cuda /bin/bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
source /opt/ros/noetic/setup.bash
cd /root/ros/workspaces/yolo_ws
source devel/setup.bash
roslaunch object_detection object_detection.launch
```

Verify YOLO output (bounding boxes)
```
docker exec -it yolo_cuda /bin/bash
source /opt/ros/noetic/setup.bash
cd /root/ros/workspaces/yolo_ws
source devel/setup.bash
rostopic echo /darknet_ros/bounding_boxes -n 1
```

## Task 2 – Plane Segmentation
Terminal 1 – Start roscore (cleanup previous runs first)
```
docker exec -it tiago_build /bin/bash
pkill -f rosbag
pkill -f plane_segmentation_node
pkill -f roscore
pkill -f rosmaster
source /opt/ros/noetic/setup.bash
roscore
```

Terminal 2 – Build package and play rosbag
```
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
cd /home/ros/workspaces/tiago_ws
catkin build plane_segmentation
source devel/setup.bash
rosparam set use_sim_time true
rosbag play --clock -l bag/task1_scene.bag
```

Terminal 3 – Run plane segmentation node
```
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
source /home/ros/workspaces/tiago_ws/devel/setup.bash
rosrun plane_segmentation plane_segmentation_node
```

Terminal 4 – Visualize in RViz
```
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
source /home/ros/workspaces/tiago_ws/devel/setup.bash
rviz
```

## Task 3 – Point Cloud Labeling (2D → 3D)
Terminal 1 – Launch labeling node
```
docker exec -it tiago_build /bin/bash
source /opt/ros/noetic/setup.bash
cd /home/ros/workspaces/tiago_ws
source devel/setup.bash
rosrun object_labeling object_labeling_node
```

Terminal 2 – Start RViz for labeling view
```
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
rviz
```
