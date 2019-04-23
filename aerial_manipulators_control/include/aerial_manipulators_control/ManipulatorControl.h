#ifndef MANIPULATORCONTROL_H
#define MANIPULATORCONTROL_H

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class ManipulatorControl {
public:
	ManipulatorControl(void);
	~ManipulatorControl(void);
	void setManipulatorName(std::string robot_model_name, std::string joint_model_group_name);
	std::vector<double> calculateJointSetpoints(geometry_msgs::Pose end_effector_pose);
	geometry_msgs::PoseStamped getEndEffectorPosition(void);
	void LoadParameters(std::string file);
	void publishJointSetpoints(std::vector<double> q);
	std::vector<double> getJointSetpoints(void);
	int init(ros::NodeHandle *n);
	bool isStarted(void);
private:
	void q_cb_ros(const boost::shared_ptr<std_msgs::Float32 const> &msg, int index);
	void joint_controller_state_cb_ros(const sensor_msgs::JointState &msg);

	ros::Publisher dynamixel_sepoint_ros_pub_;
	ros::Subscriber *manipulator_q_set_point_sub_ros_;
	ros::Subscriber joint_state_sub_ros_;
	ros::NodeHandle *n_;

	robot_model_loader::RobotModelLoader *robot_model_loader_;;
	robot_state::JointModelGroup* joint_model_group_;
	robot_state::RobotModelPtr *kinematic_model_;
	robot_state::RobotStatePtr *kinematic_state_;

	std::string robot_model_name_, joint_model_group_name_;
	float *q_torque_meas_;
	std::vector<double> q_pos_meas_, q_setpoint_;
	std::vector<int> q_directions_;
	int is_initialized_, number_of_joints_, setpoint_seq_;
	bool start_flag_;
};

#endif