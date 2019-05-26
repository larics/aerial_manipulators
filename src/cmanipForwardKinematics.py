#!/usr/bin/env python

import numpy as np
import math as m
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
import time as t


def spin_transform(theta,d,alpha,a):
    # Calculate transfer matrix w.r.t. DH table row |theta | d | alpha | a |
    T = np.matrix(((m.cos(theta), -m.cos(alpha)*m.sin(theta),m.sin(alpha)*m.sin(theta), a*m.cos(theta)),(m.sin(theta), m.cos(alpha)*m.cos(theta), -m.sin(alpha)*m.cos(theta), a*m.sin(theta)),(0, m.sin(alpha), m.cos(alpha), d),(0, 0,0,1)))
    return T


def world2tool(TW0, T01, T12, T23, T34, T4H, TH5):
    # Calculate transformation matrix
    T = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(TW0, T01), T12), T23), T34), T4H), TH5)
    return T


class CManip:


    def poseEnd_callback(self,data):
        self.poseEnd = data

    def jointState_callback(self,data):
        self.jointState = data

    def __init__(self):

        self.poseEnd = LinkStates
        self.jointState = JointState()

        self.check = Point()

        self.pub1 = rospy.Publisher('/forwardkinematiccheck', Point, queue_size=1)

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.poseEnd_callback)
        rospy.Subscriber("/cmanip/joint_states", JointState, self.jointState_callback)

        self.theta0 = np.array([m.pi, m.pi/2, 0, 0, 0, -m.pi/2, 0])
        self.alpha = np.array([m.pi/2, 0, -m.pi/2, 0, 0, -m.pi/2, 0])
        self.a = np.array([0, 0.122490, 0.1365, 0.075511, 0.072489, 0, 0])
        self.d = np.array([0, 0, 0, 0, 0, 0, 0.04526])


    def run(self):

        while not rospy.is_shutdown():

            t.sleep(0.1)

            if len(self.jointState.position) == 5:


                q1 = self.jointState.position[0]
                q2 = self.jointState.position[1]
                q3 = self.jointState.position[2]
                q4 = self.jointState.position[3]
                q5 = self.jointState.position[4]

                TW0 = spin_transform(self.theta0[0], self.d[0], self.alpha[0], self.a[0])
                T01 = spin_transform(q1+self.theta0[1], self.d[1], self.alpha[1], self.a[1])
                T12 = spin_transform(q2+self.theta0[2], self.d[2], self.alpha[2], self.a[2])
                T23 = spin_transform(q3+self.theta0[3], self.d[3], self.alpha[3], self.a[3])
                T34 = spin_transform(q4+self.theta0[4], self.d[4], self.alpha[4], self.a[4])
                T4H = spin_transform(q5+self.theta0[5], self.d[5], self.alpha[5], self.a[5])
                TH5 = spin_transform(self.theta0[6], self.d[6], self.alpha[6], self.a[6])

                TW5 = world2tool(TW0, T01, T12, T23, T34, T4H, TH5)

                self.check.x = TW5[0,3] - self.poseEnd.pose[6].position.x
                self.check.y = TW5[1,3] - self.poseEnd.pose[6].position.y
                self.check.z = TW5[2,3] - self.poseEnd.pose[6].position.z

                #print(TW5)

                self.pub1.publish(self.check)


if __name__ == '__main__':
    rospy.init_node('CManip')
    jointctl = CManip()
    jointctl.run()
