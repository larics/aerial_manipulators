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
)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy roslib rospack
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(WPManipulatorDirectKinematics src/WPManipulatorDirectKinematics.cpp)
target_link_libraries(WPManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

add_library(median_filter src/median_filter.cpp)
target_link_libraries(median_filter ${catkin_LIBRARIES})

add_library(KalmanFilter src/KalmanFilter.cpp)
target_link_libraries(KalmanFilter ${catkin_LIBRARIES})

add_library(WPManipulatorInverseKinematics src/WPManipulatorInverseKinematics.cpp)
target_link_libraries(WPManipulatorInverseKinematics WPManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

add_executable(wp_manipulator_control src/WPManipulatorControl.cpp)
target_link_libraries(wp_manipulator_control WPManipulatorDirectKinematics WPManipulatorInverseKinematics KalmanFilter median_filter ${catkin_LIBRARIES} yaml-cpp)
