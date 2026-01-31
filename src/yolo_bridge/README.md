# yolo_bridge

ROS package that bridges YOLOv8 detections to `darknet_ros_msgs/BoundingBoxes`.

## Node
- `yolov8_darknet_bridge.py`

## Topics
Publishes:
- `/darknet_ros/bounding_boxes` (`darknet_ros_msgs/BoundingBoxes`)
- `/darknet_ros/detection_image` (`sensor_msgs/Image`) when `publish_vis: true`

Subscribes:
- `~image_topic` (`sensor_msgs/Image`) default: `/xtion/rgb/image_raw`

## Params (YAML)
See `config/yolo_bridge.yaml` for defaults:
- `image_topic`
- `model`
- `device`
- `conf`, `iou`, `classes`
- `publish_vis`, `line_thickness`, `font_scale`

## Launch
```
roslaunch yolo_bridge yolo_bridge.launch
```

## Build
```
catkin_make
source devel/setup.bash
```
