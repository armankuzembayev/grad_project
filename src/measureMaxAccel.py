#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import *
from grad_project.srv import *
import time


class Controller:
    def __init__(self):
        self.measureActive = False;
        self.measurements = [];
        self.start = 0;
        self.end = 0;
        self.sub = rospy.Subscriber('/pioneer3at/accelerometer/values', Imu, self.callback)

    def callback(self, twist):   
        if twist.linear_acceleration.x > 0.1 or twist.linear_acceleration.z > 0.1:
            if self.measureActive == False:
                self.measureActive = True;
                print('acc measurement started')
                self.start = time.time();
        else:
            if self.measureActive == True:
                self.measureActive = False;
                print('acc measurement finished')
                self.end = time.time();
                print("total time: " + str(self.end - self.start) + "   total acc: " + str(sum(self.measurements) / len(self.measurements)) + "     speed: " + str((self.end - self.start) * sum(self.measurements) / len(self.measurements)))


        if self.measureActive:
            self.measurements.append(math.sqrt(twist.linear_acceleration.x**2 + twist.linear_acceleration.z**2));

if __name__ == '__main__':
    rospy.init_node('velMeasure')
    Controller()
    rospy.spin()
