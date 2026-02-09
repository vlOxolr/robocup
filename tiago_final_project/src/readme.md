# "Tiago Bistro"

This repository contains a modular ROS-based restaurant service system implemented and tested on the **TIAGo** mobile manipulator.

The system is organized around a **SMACH state machine** that coordinates independent perception, navigation, speech, ordering, and manipulation modules.  
Each module runs as an independent ROS node and communicates only through ROS topics, services, or launch files.

---

## 1. System Overview

### High-level service workflow

The system implements the following high-level service flow:

- Navigate to restaurant entrance

- Greet customers with speech

- Detect and count customers using vision

- Decide table assignment

- Guide customers to the table

- Ask customers to place an order

- Receive confirmed order from web system

- Navigate to pickup counter

- Detect and grasp food item

- Return to customer table

- Place food on table

- Repeat grasping and placing steps until all dishes are served

- Announce service completion

- Return to entrance and wait for next customers 

**Design principle:**  
> The state machine only decides *what to do next*.  
> All perception, navigation, speech, and manipulation are handled by independent ROS nodes.

---

## 2. Package Structure

```text
tiago_public_ws/src
├── detection_speech/
│   └── scripts/
│       ├── guest_perception_node.py
│       ├── tts_server_node.py
│       ├── yolo_wave_tiago2.py     
│       └── yolov8_darknet_bridge.py       
│
├── navigation/
│   ├── launch/
│   │   └── simple_navigator.launch
│   └── scripts/
│       ├── simple_navigator.py
│       └── localization.py
│
├── ordering/
│   ├── launch/
│   │   └── tiago_ordering.launch     
│   ├── msg/
│   │   ├── Order.msg
│   │   └── OrderStatusUpdate.msg
│   ├── srv/
│   │   ├── Reserve.srv
│   │   └── SetOrderStatus.srv
│   └── scripts/
│       ├── inventory_manager.py
│       ├── order_client.py
│       ├── order_manager.py
│       └── web_order_server.py
│
├── state_machine/
│   └── scripts/
│       └── state_machine.py          
│
├── plane_segmentation/
│   └── src/
│       └── applications/
│           └── plane_segmentation_node  
│
├── object_labeling/
│   └── src/
│       └── applications/
│           └── object_labeling_node      
│
├── grasp/
│   └── launch/
│       ├── grasp_apple.launch
│       ├── grasp_bottel.launch
│       ├── grasp_burger.launch
│       └── grasp_place.launch
│
└── README.md

```

---

## 3. Runtime Dependencies

To run the full Tiago Restaurant Service System, the following software dependencies are required.

### 3.1 ROS and Core Framework

- **ROS Noetic**
- Python 3  
- `smach`, `smach_ros` – for state machine execution  

### 3.2 Perception Dependencies

- `ultralytics` (YOLOv8) – person and pose detection  
- OpenCV – image processing and visualization  
- `cv_bridge` – conversion between ROS images and OpenCV images  

### 3.3 Navigation and Robot Platform

- TIAGo navigation stack (Move Base / AMCL)  
- Pal Robotics standard TIAGo packages  

### 3.4 Speech

- `pal_interaction_msgs` – for interaction with TIAGo’s `/tts` action server  

### 3.5 Manipulation and Grasping

- Point cloud libraries required by:
  - `plane_segmentation`
  - `object_labeling`  
- Standard ROS C++ toolchain for grasp packages  

---

> **Note:**  
> All packages are expected to be built inside a Catkin workspace (e.g., `tiago_public_ws`) and sourced before running the system:
>
> ```bash
> source /tiago_public_ws/devel/setup.bash
> ```

---

## 4. Launch Order

The following launch order is required for correct system behavior:

```bash
# 1) Use the world map
rosservice call /pal_map_manager/change_map "input: '2025-12-22_153313'"

# 2) Start guest perception
rosrun detection_speech yolo_wave_tiago2.py

# 3) Start ordering system
roslaunch ordering tiago_ordering.launch

# 4) Start grasping and placing
rosrun detection_speech yolov8_darknet_bridge.py
rosrun plane_segmentation plane_segmentation_node.py
rosrun object_labeling object_labeling_node.py

# 7) Start state machine (last)
rosrun state_machine state_machine.py
```

---

## 5. Notes

- Grasping and placing are currently **launch-based placeholders**; success is inferred from the launch return code.
- The state machine triggers navigation and grasping via `rosrun` / `roslaunch` (not ROS actions).
- If perception times out, the system **defaults to one customer**.
- The robot serves **one table at a time** and returns to the entrance after finishing an order.
- Make sure all required nodes are running **before** starting the state machine.

---

## 6. Author

**Author:** Bangdong Zhang, Chenhao Guan, Shuowen Li, Tian Qin  
**Project:** "Tiago Bistro"
