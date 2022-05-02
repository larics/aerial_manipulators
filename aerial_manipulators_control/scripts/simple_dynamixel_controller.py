#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from std_msgs.msg import Int32, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
import copy
import yaml

import rospkg


class SimpleDynamixelController:

  def __init__(self):
    # Parameters
    self.rate = rospy.get_param('~rate', 100)

    # Config file, default is from aerial manipulators control
    rospack = rospkg.RosPack()
    path = rospack.get_path('aerial_manipulators_control')
    config_file = rospy.get_param('~config_file', path + \
      '/config/asap_manipulator_4r_controllers.yaml')

    # Set up publisher for JointTrajectory
    self.joint_trajectory = JointTrajectory()
    self.joint_trajectory_pub = rospy.Publisher(
      'simple_dynamixel_controller/joint_trajectory_out', JointTrajectory, 
      queue_size=1)

    # Open the config file and load it into a list of subscribers
    self.dynamixel_motor_list = []
    self.keys = []
    self.dynamixel_motor_references = []
    s = open(config_file, "r")
    config = yaml.safe_load(s)
    self.keys = config.keys()
    self.keys.sort()
    for i in range(len(self.keys)):
      current_handler = JointPositionHandler(self.keys[i])
      self.dynamixel_motor_list.append(copy.copy(current_handler))
      self.dynamixel_motor_references.append(0.0)

  def run(self):
    #rospy.spin()
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      
      # Reset joint trajectory, add frame id and joint names
      self.joint_trajectory = JointTrajectory()
      self.joint_trajectory.header.frame_id = "world"
      self.joint_trajectory.joint_names = self.keys

      # Collect the latest received dynamixel references
      for i in range(len(self.dynamixel_motor_list)):
        self.dynamixel_motor_references[i] = \
          self.dynamixel_motor_list[i].getJointState().data

      # Set up the joint trajectory point
      # TODO: Check if velocities, accelerations and effort are required by the
      # dynamixel workbench.
      joint_trajectory_point = JointTrajectoryPoint()
      joint_trajectory_point.time_from_start = rospy.Duration(0)
      joint_trajectory_point.positions = self.dynamixel_motor_references

      # Append two of the same points to the joint trajectory
      # TODO: Check if two same points can be sent to the dynamixel workbench
      self.joint_trajectory.points.append(copy.deepcopy(joint_trajectory_point))
      self.joint_trajectory.points.append(copy.deepcopy(joint_trajectory_point))

      # Set timestamp of the joint trajectory
      self.joint_trajectory.header.stamp = rospy.Time.now()

      # Finally, publish the trajectory
      self.joint_trajectory_pub.publish(self.joint_trajectory)


class JointPositionHandler:

  def __init__(self, sub_topic):

    # Subscriber for state
    self.joint_state = Float64()
    sub_topic = sub_topic + '/command'
    self.sub = rospy.Subscriber(sub_topic, Float64, self.jointReferenceCallback,
      queue_size=1)

  def jointReferenceCallback(self, msg):
    self.joint_state.data = msg.data

  def getJointState(self):
    return self.joint_state


if __name__ == '__main__':
  rospy.init_node('simple_dynamixel_controller')
  simple_controller = SimpleDynamixelController()
  simple_controller.run()