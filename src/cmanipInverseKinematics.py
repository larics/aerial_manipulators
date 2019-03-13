#!/usr/bin/env python

import numpy as np
import math as m
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time as t

def Jacobian(theta):
    q1 = theta[0]
    q2 = theta[1]
    q3 = theta[2]
    q4 = theta[3]
    q5 = theta[4]
    #Calculate Jacobian with given initial angles
    j11 = - 0.037756*m.cos(q1 + q2 + q3) - 0.02263*m.cos(q1 + q2 - q3 - q4 - q5) - 0.036245*m.cos(q1 + q2 - q3 - q4) - 0.037756*m.cos(q1 + q2 - q3) - 0.036245*m.cos(q1 + q2 + q3 + q4) - 0.1365*m.cos(q1 + q2)
    - 0.12249*m.cos(q1) - 0.02263*m.cos(q1 + q2 + q3 + q4 + q5)
    j12 = - 0.037756*m.cos(q1 + q2 + q3) - 0.02263*m.cos(q1 + q2 - q3 - q4 - q5) - 0.036245*m.cos(q1 + q2 - q3 - q4) - 0.037756*m.cos(q1 + q2 - q3) - 0.036245*m.cos(q1 + q2 + q3 + q4) - 0.1365*m.cos(q1 + q2)
    - 0.02263*m.cos(q1 + q2 + q3 + q4 + q5)
    j13 = 0.02263*m.cos(q1 + q2 - q3 - q4 - q5) - 0.037756*m.cos(q1 + q2 + q3) + 0.036245*m.cos(q1 + q2 - q3 - q4) + 0.037756*m.cos(q1 + q2 - q3) - 0.036245*m.cos(q1 + q2 + q3 + q4) - 0.02263*m.cos(q1 + q2 + q3 + q4 + q5)
    j14 = 0.02263*m.cos(q1 + q2 - q3 - q4 - q5) + 0.036245*m.cos(q1 + q2 - q3 - q4) - 0.036245*m.cos(q1 + q2 + q3 + q4) - 0.02263*m.cos(q1 + q2 + q3 + q4 + q5)
    j15 = (2263*m.sin(q3 + q4 + q5)*m.sin(q1 + q2))/50000
    j21 = 0.02263*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4 + q5) - 0.037756*m.sin(q1 + q2 + q3) - 0.02263*m.sin(q1 + q2 + q3 + q4 + q5) + 0.037756*m.sin(q3 - 1.0*q2 - 1.0*q1) - 0.036245*m.sin(q1 + q2 + q3 + q4)
    - 0.1365*m.sin(q1 + q2) + 0.036245*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4) - 0.12249*m.sin(q1)
    j22 = 0.02263*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4 + q5) - 0.037756*m.sin(q1 + q2 + q3) - 0.02263*m.sin(q1 + q2 + q3 + q4 + q5) + 0.037756*m.sin(q3 - 1.0*q2 - 1.0*q1) - 0.036245*m.sin(q1 + q2 + q3 + q4)
    - 0.1365*m.sin(q1 + q2) + 0.036245*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4)
    j23 = - 0.02263*m.sin(q1 + q2 + q3 + q4 + q5) - 0.037756*m.sin(q1 + q2 + q3) - 0.02263*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4 + q5) - 0.037756*m.sin(q3 - 1.0*q2 - 1.0*q1) - 0.036245*m.sin(q1 + q2 + q3 + q4)
    - 0.036245*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4)
    j24 = - 0.02263*m.sin(q1 + q2 + q3 + q4 + q5) - 0.02263*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4 + q5) - 0.036245*m.sin(q1 + q2 + q3 + q4) - 0.036245*m.sin(q3 - 1.0*q2 - 1.0*q1 + q4)
    j25 = -0.04526*m.sin(q3 + q4 + q5)*m.cos(q1 + q2)
    j31 = 0
    j32 = 0
    j33 = - 0.04526*m.cos(q3 + q4 + q5) - 0.072489*m.cos(q3 + q4) - 0.075511*m.cos(q3)
    j34 = - 0.04526*m.cos(q3 + q4 + q5) - 0.072489*m.cos(q3 + q4)
    j35 = - 0.04526*m.cos(q3 + q4 + q5)
    j41 = - m.cos(q3 + q4 + q5)*m.cos(q1 + q2)
    j42 = - m.cos(q3 + q4 + q5)*m.cos(q1 + q2)
    j43 = m.sin(q3 + q4 + q5)*m.sin(q1 + q2)
    j44 = m.sin(q3 + q4 + q5)*m.sin(q1 + q2)
    j45 = m.sin(q3 + q4 + q5)*m.sin(q1 + q2)
    j51 = -m.cos(q3 + q4 + q5)*m.sin(q1 + q2)
    j52 = -m.cos(q3 + q4 + q5)*m.sin(q1 + q2)
    j53 = - m.sin(q3 + q4 + q5)*m.cos(q1 + q2)
    j54 = - m.sin(q3 + q4 + q5)*m.cos(q1 + q2)
    j55 = - m.sin(q3 + q4 + q5)*m.cos(q1 + q2)
    j61 = 0
    j62 = 0
    j63 = - m.cos(q3 + q4 + q5)
    j64 = - m.cos(q3 + q4 + q5)
    j65 = - m.cos(q3 + q4 + q5)
    jacob = np.matrix(((j11, j12, j13, j14, j15), (j21, j22, j23, j24, j25), (j31, j32, j33, j34, j35), (j41, j42, j43, j44, j45), (j51, j52, j53, j54, j55), (j61, j62, j63, j64, j65)))
    return jacob

class CManip:

    def cmanipTwist_callback(self,data):

        self.posOrt = data

    def __init__(self):

        self.posOrt = Twist()
        self.theta0 = np.array([0, 0, 0, 0, 0]).transpose()
        self.position0 = np.array([0, 0, 0.45225, 0, 0, 1]).transpose()
        self.isGivenFirst = False

        rospy.Subscriber("/cmanipTwist", Twist, self.cmanipTwist_callback)

        self.pub1 = rospy.Publisher('/cmanip/joint1_position_controller/command', Float64, queue_size=1)
        self.pub2 = rospy.Publisher('/cmanip/joint2_position_controller/command', Float64, queue_size=1)
        self.pub3 = rospy.Publisher('/cmanip/joint3_position_controller/command', Float64, queue_size=1)
        self.pub4 = rospy.Publisher('/cmanip/joint4_position_controller/command', Float64, queue_size=1)
        self.pub5 = rospy.Publisher('/cmanip/joint5_position_controller/command', Float64, queue_size=1)

    def run(self):

        while not rospy.is_shutdown():
            theta = [1.570796327, 0.0, 0.0, 0.0, 4.71238898, 0]
            print Jacobian(theta)
            #t.sleep(0.1)

            #position1 = np.array([self.posOrt.linear.x, self.posOrt.linear.y, self.posOrt.linear.z, self.posOrt.angular.x, self.posOrt.angular.y, self.posOrt.angular.z]).transpose()

            #if (not self.isGivenFirst):
            #    if (position1[0] != 0 or position1[1] != 0 or position1[2] != 0 or position1[3] != 0 or position1[4] != 0):
            #        self.isGivenFirst = True
            #    else:
            #        position1 = np.array([0, 0, 0.45225, 0, 0, 1]).transpose()


            #theta1 = np.dot(np.linalg.pinv(Jacobian(self.theta0)), (position1 - self.position0)) + self.theta0

            #self.pub1.publish(theta1[0,0])
            #self.pub2.publish(theta1[0,1])
            #self.pub3.publish(theta1[0,2])
            #self.pub4.publish(theta1[0,3])
            #self.pub5.publish(theta1[0,4])

            #for i in range(0,5):
            #    self.theta0[i] = theta1[0,i]
            #self.position0 = position1

if __name__ == '__main__':
    rospy.init_node('CManip')
    jointctl = CManip()
    jointctl.run()
