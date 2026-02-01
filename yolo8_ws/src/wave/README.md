# wave

ROS package for wave-detection greeting based on YOLOv8 pose + person detection.

## Nodes
- `yolo_wave_tiago2.py`

## Topics
Publishes:
- `/guest_visible_count` (`std_msgs/Int32`)
- `/guest_locked_count` (`std_msgs/Int32`)
- `/guest_event` (`std_msgs/String`)
- `/guest_counter/debug_image` (`sensor_msgs/Image`)

Subscribes:
- `~image_topic` (`sensor_msgs/Image`) default: `/xtion/rgb/image_raw`

Uses (optional):
- `/tts` (`pal_interaction_msgs/TtsAction`)

## Params (YAML)
See `config/wave.yaml` for defaults.

## Launch
```
roslaunch wave wave.launch
```

## Build
```
catkin_make
source devel/setup.bash
```
