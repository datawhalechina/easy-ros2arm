# easy-ros2Arm
该项目旨在梳理6自由度机械臂相关知识，主要包含内容如下：

1. 运动学与逆运动学
2. ROS2简介
3. MoveIt简介

其中，ROS2与MoveIt会以视频的方式介绍；而运动学和逆运动学将以文本的形式介绍。



## 代码结构

```markdown
|-src
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

