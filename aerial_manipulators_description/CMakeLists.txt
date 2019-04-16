cmake_minimum_required(VERSION 2.8.3)
project(aerial_manipulators_description)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roslib
    rospack
)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS rospy roslib rospack
)
