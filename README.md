# dofbot_ros2
Robotic arm with machine vision and its os is ROS2.



## Directory Structure

The directory structure for this repo is as follows

```markdown
|-src
| |-dofbot_classification #Subscribe camera data and item classification using YOLOv5.
| |
| |-dofbot_config #integration with a new robot by moveit_setup_assistance.
| |
| |-dofbot_description #describe and visualize robot using URDF, RViz.
| |
| |-dofbot_moveit #compute forward kinematics and inverse kinematics using MoveIt and KDL library.
| |
| |-dofbot_msgs #Create Imags msg interface by ros2 .
| |
| |-dofbot_sensors #Acquire camera data and publish it by ros2 topic publisher. 
```



## Develop Steps

1. dofbot_description

2. dofbot_config

3. dofbot_moveit

4. dofbot_msgs

5. dofbot_sensors

6. dofbot_classification



## Requirements

```python
# HardWare
Jetson Nano
Jetson Orin Nano

# os
Ubuntu20.04 for ROS2
# Ubuntu20.04 image for Jetson Nano
https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image

# ROS Version
ROS2 Humble

# Packages:
OpenCV
```

