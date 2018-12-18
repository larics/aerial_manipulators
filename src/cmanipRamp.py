#!/usr/bin/env python

import numpy as np
import math as m
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
import time as t
from std_msgs.msg import Float64

class CManip:

    def jointState_callback(self,data):
        self.jointState = data

    def __init__(self):

        self.jointState = JointState()

        #Choose joint to publishe to
        self.pub1 = rospy.Publisher('/cmanip/joint2_position_controller/command', Float64, queue_size=1)

        rospy.Subscriber("/cmanip/joint_states", JointState, self.jointState_callback)
        self.pos5 = 0
        self.forward = True
        self.counter = 0

    def run(self):

        while not rospy.is_shutdown():

            t.sleep(0.1)

#limits:
#  upper: [1.57, 2.2, 1.57, 1.57, 2.3]
#  lower: [-1.57, -2.2, -1.57, -1.57, -2.3]

            if self.pos5 > 2:
                self.counter += 1
                self.forward = False
            elif self.pos5 < -2:
                self.counter += 1
                self.forward = True

            if self.counter == 20:
                self.counter = 0

            if self.forward and not self.counter:
                self.pos5 = self.pos5 + 0.1
            elif not self.counter:
                self.pos5 = self.pos5 - 0.1

            self.pub1.publish(self.pos5)


if __name__ == '__main__':
    rospy.init_node('CManip')
    jointctl = CManip()
    jointctl.run()
