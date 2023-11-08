#!/usr/bin/env python3
# coding: utf-8
'''
执行此段代码-->订阅发布话题为"joint_states"的各关节角度,驱动真机移动
'''
import rospy
import Arm_Lib
from math import pi
from sensor_msgs.msg import JointState

# 弧度转角度
RA2DE = 180 / pi


def topic(msg):
    # 如果不是该话题的数据直接返回
    if not isinstance(msg, JointState): return
    # 定义关节角度容器,最后一个是夹爪的角度,默认夹爪不动为90.
    joints = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]
    # 将接收到的弧度[-1.57,1.57]转换成角度[0,180]
    for i in range(5): joints[i] = (msg.position[i] * RA2DE) + 90
    # 调驱动函数
    sbus.Arm_serial_servo_write6_array(joints, 100)


if __name__ == '__main__':
    sbus = Arm_Lib.Arm_Device()
    # ROS节点初始化
    rospy.init_node("ros_dofbot")
    # 创建一个订阅者
    subscriber = rospy.Subscriber("/joint_states", JointState, topic)
    # 设置循环的频率
    rate = rospy.Rate(2)
    # 按照循环频率延时
    rospy.spin()
