cmake_minimum_required(VERSION 2.8.3)
project(aerial_manipulators_control)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    moveit_visual_tools
    eigen_conversions
    roscpp
    rospy
    roslib
    rospack
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    dynamixel_workbench_msgs
)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
    config/JointCtlParams.cfg
    )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy roslib rospack moveit_core moveit_visual_tools moveit_ros_planning_interface sensor_msgs eigen_conversions
   DEPENDS EIGEN3
   LIBRARIES ManipulatorControl
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

add_executable(JointPositionControl src/JointPositionControl.cpp)
target_link_libraries(JointPositionControl pid ${catkin_LIBRARIES} yaml-cpp)

add_library(ManipulatorControl src/ManipulatorControl.cpp)
target_link_libraries(ManipulatorControl ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

add_executable(wp_manipulator_control src/WPManipulatorControl.cpp)
target_link_libraries(wp_manipulator_control ${catkin_LIBRARIES} ManipulatorControl)

add_executable(aerial_manipulator_control src/AerialManipulatorControl.cpp)
target_link_libraries(aerial_manipulator_control ${catkin_LIBRARIES} yaml-cpp)