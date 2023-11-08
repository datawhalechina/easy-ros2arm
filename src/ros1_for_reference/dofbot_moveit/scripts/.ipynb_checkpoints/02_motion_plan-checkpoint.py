#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# 角度转弧度
DE2RA = pi / 180

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("dofbot_motion_plan_py")
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
    # 设置"down"为目标点
    dofbot.set_named_target("down")
    dofbot.go()
    sleep(0.5)
    # 创建位姿实例
    pos = Pose()
    # 设置具体的位置
    pos.position.x = 0.0
    pos.position.y = 0.0597016
    pos.position.z = 0.168051
    # RPY的单位是角度值
    roll = -140.0
    pitch = 0.0
    yaw = 0.0
    # RPY转四元素
    q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    # pos.orientation.x = 0.940132
    # pos.orientation.y = -0.000217502
    # pos.orientation.z = 0.000375234
    # pos.orientation.w = -0.340811
    pos.orientation.x = q[0]
    pos.orientation.y = q[1]
    pos.orientation.z = q[2]
    pos.orientation.w = q[3]
    # 设置目标点
    dofbot.set_pose_target(pos)
    # 多次执行,提高成功率
    for i in range(5):
        # 运动规划
        plan = dofbot.plan()
        if len(plan.joint_trajectory.points) != 0:
            print ("plan success")
            # 规划成功后运行
            dofbot.execute(plan)
            break
        else:
            print ("plan error")
