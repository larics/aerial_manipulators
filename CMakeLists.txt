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
    message_generation
)

add_definitions(-std=c++11)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
    config/JointCtlParams.cfg
    )

add_message_files(
  FILES
  float5.msg
  float6.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy roslib rospack message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})

add_executable(joint_position_control src/joint_position_control.cpp)
target_link_libraries(joint_position_control pid ${catkin_LIBRARIES} yaml-cpp)

add_library(WPManipulatorInverseDynamics src/WPManipulatorInverseDynamics.cpp)
target_link_libraries(WPManipulatorInverseDynamics ${catkin_LIBRARIES})

add_executable(WPManipulatorForceEstimation src/WPManipulatorForceEstimation.cpp)
target_link_libraries(WPManipulatorForceEstimation WPManipulatorInverseDynamics ${catkin_LIBRARIES})

#add_library(WPManipulatorDirectKinematics src/WPManipulatorDirectKinematics.cpp)
#target_link_libraries(WPManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

#add_library(median_filter src/median_filter.cpp)
#target_link_libraries(median_filter ${catkin_LIBRARIES})

#add_library(KalmanFilter src/KalmanFilter.cpp)
#target_link_libraries(KalmanFilter ${catkin_LIBRARIES})

#add_library(WPManipulatorInverseKinematics src/WPManipulatorInverseKinematics.cpp)
#target_link_libraries(WPManipulatorInverseKinematics WPManipulatorDirectKinematics ${catkin_LIBRARIES} yaml-cpp)

#add_executable(wp_manipulator_control src/WPManipulatorControl.cpp)
#target_link_libraries(wp_manipulator_control WPManipulatorDirectKinematics WPManipulatorInverseKinematics KalmanFilter median_filter ${catkin_LIBRARIES} yaml-cpp)
