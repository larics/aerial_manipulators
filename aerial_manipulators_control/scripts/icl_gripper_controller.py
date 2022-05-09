#!/usr/bin/env python

__author__ = 'RobertMilijas'

import rospy
from std_msgs.msg import Float64


class IclGripperController:
    """
    Sends joint commands to simple_dynamixel_controller.py to acchieve the desired gripper aperture.

    Subscriptions:
        icl_gripper_controller/gripper_aperture - std_msgs/Float64

            How much should the gripper be opened on a scale from 0 to 1

    Publications:
        icl_gripper_controller/jonit1/command
        icl_gripper_controller/jonit2/command

            Commands for the individual dynamixel motors as defined by the simple_dynamixel_controller.py

    Parameters:
        ~max_angle - scaling factor for the desired gripper_aperture

    """

    def __init__(self, node_name):
        # Parameters
        self.rate = rospy.get_param('~rate', 100)

        # Set up publishers for JointCommands
        self.joint_command = Float64()
        self.joint_command_1_pub = rospy.Publisher(node_name + '/joint1/command', Float64, queue_size=1)
        self.joint_command_2_pub = rospy.Publisher(node_name + '/joint2/command', Float64, queue_size=1)

        # Set up subscriber
        self.gripper_aperture_sub = rospy.Subscriber(node_name + '/gripper_aperture',
                                                     Float64, self.gripper_aperture_callback, queue_size=1)

        self.max_angle = rospy.get_param("~max_angle")

        self.new_aperture = False
        self.aperture = 0.0

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.new_aperture:
                self.new_aperture = False
                self.joint_command.data = self.aperture * 1.5
                self.joint_command_1_pub.publish(self.joint_command)
                self.joint_command_2_pub.publish(self.joint_command)

    def gripper_aperture_callback(self, msg):
        self.aperture = msg.data
        self.clamp_aperture(0, 1)
        self.new_aperture = True

    def clamp_aperture(self, lower, upper):
        if self.aperture > upper:
            self.aperture = upper
        if self.aperture < lower:
            self.aperture = lower


if __name__ == '__main__':
    node_name = 'icl_gripper_controller'
    rospy.init_node(node_name)
    gripper_controller = IclGripperController(node_name)
    gripper_controller.run()
