cmake_minimum_required(VERSION 2.8.3)
project(aerial_manipulators)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    rospack
    std_msgs
    dynamic_reconfigure
    dynamixel_workbench_msgs
    uav_ros_control
)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)
find_package(Eigen 3 REQUIRED)

generate_dynamic_reconfigure_options(
    config/JointCtlParams.cfg
    )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy roslib rospack
)

MESSAGE(WARNING ${catkin_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_position_control src/joint_position_control.cpp)
target_link_libraries(joint_position_control ${catkin_LIBRARIES} yaml-cpp)