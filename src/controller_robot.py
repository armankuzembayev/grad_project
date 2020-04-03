#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from grad_project.srv import *



class Controller:
    def __init__(self, ):
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback)
        self.width = 0.381
        self.wheel_radius = 0.111
        self.v_l = 0.0
        self.v_r = 0.0
        self.motorNames = ["front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"];
        self.motors = [0.0]*4

    def callback(self, twist):   
        lin_vel_x = twist.linear.x
        ang_vel_z = twist.angular.z 
        self.v_l = (lin_vel_x - 0.5 * self.width * ang_vel_z) / self.wheel_radius
        self.v_r = (lin_vel_x + 0.5 * self.width * ang_vel_z) / self.wheel_radius
        self.motors[0] = self.v_l
        self.motors[1] = self.v_r
        self.motors[2] = self.v_l
        self.motors[3] = self.v_r
        for motor in range(4):
            set_velocity = rospy.ServiceProxy('pioneer3at/' + str(self.motorNames[motor]) + '/set_velocity', set_float);
            set_velocity_srv = set_velocity(self.motors[motor])


if __name__ == '__main__':
    rospy.init_node('controller')
    Controller()
    rospy.spin()
