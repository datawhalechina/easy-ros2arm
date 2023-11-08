#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化ROS节点
    rospy.init_node('Set_Scene')
    # 初始化场景对象
    scene = PlanningSceneInterface()
    # 初始化机械臂
    dofbot = MoveGroupCommander('dofbot')
    # 当运动规划失败后，允许重新规划
    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)
    # 尝试规划的次数
    dofbot.set_num_planning_attempts(10)
    # 设置允许目标位置误差
    dofbot.set_goal_position_tolerance(0.01)
    # 设置允许目标姿态误差
    dofbot.set_goal_orientation_tolerance(0.01)
    # 设置允许目标误差
    dofbot.set_goal_tolerance(0.01)
    # 设置最大速度
    dofbot.set_max_velocity_scaling_factor(1.0)
    # 设置最大加速度
    dofbot.set_max_acceleration_scaling_factor(1.0)
    # 设置"up"为目标点
    dofbot.set_named_target("up")
    dofbot.go()
    sleep(0.5)
    target_joints1 = [0, -1.18, -1.17, 0.77, 0.03]
    target_joints2 = [0, -1.21, 0.52, -0.89, 0.08]
    tool_size = [0.03, 0.03, 0.03]
    # 获取终端link的名称
    end_effector_link = dofbot.get_end_effector_link()
    # 设置tool的位姿
    p = PoseStamped()
    p.header.frame_id = end_effector_link
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.10
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1
    # 将tool附着到机械臂的夹爪上
    scene.attach_box(end_effector_link, 'tool', p, tool_size)
    while 1:
        dofbot.set_joint_value_target(target_joints1)
        dofbot.go()
        sleep(0.5)
        dofbot.set_joint_value_target(target_joints2)
        dofbot.go()
        sleep(0.5)
