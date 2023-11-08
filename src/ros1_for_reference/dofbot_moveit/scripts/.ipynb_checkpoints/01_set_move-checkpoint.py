#!/usr/bin/env python
# coding: utf-8
from time import sleep
import rospy
from moveit_commander.move_group import MoveGroupCommander

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("dofbot_set_move")
    # 初始化机械臂
    dofbot = MoveGroupCommander("dofbot")
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
    while (1):
        # 设置随机目标点
        dofbot.set_random_target()
        # 开始运动
        dofbot.go()
        sleep(0.5)
        # 设置"up"为目标点
        #dofbot.set_named_target("up")
        #dofbot.go()
        #sleep(0.5)
        # 设置"down"为目标点
        #dofbot.set_named_target("down")
        #dofbot.go()
        #sleep(0.5)
