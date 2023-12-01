#!/usr/bin/env python3
#coding: utf-8
from dofbot_hardware_control import Arm_Lib

import rclpy
from sensor_msgs.msg import JointState
from math import pi
from rclpy.node import Node

import smbus

class JointStateSubscriber(Node):
    
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            5)
        self.sbus = Arm_Lib.Arm_Device()
        self.pos_arr = [0.0, 0.0, 0.0, 0.0, 0.0, 180.0]
        self.subscription

    def listener_callback(self, msg):
        for i,n in enumerate(msg.name):
            self.get_logger().info('Joint Name: "%s"' % n)
            pos = self.acquire_position(n)
            self.get_logger().info('pos: "%d"' % pos)
            self.pos_arr[pos] = (msg.position[i] * 180 / pi) + 90
        self.sbus.Arm_serial_servo_write6_array(self.pos_arr, 100)
    
    def acquire_position(self, name):
        match name:
            case 'joint1':
                return 0
            case 'joint2':
                return 1
            case 'joint3':
                return 2
            case 'joint4':
                return 3
            case 'joint5':
                return 4
            case 'joint6':
                return 5
        
            
def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
