#ifndef MANIPULATORCONTROL_H
#define MANIPULATORCONTROL_H

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class ManipulatorControl {
public:
	ManipulatorControl(void);
	~ManipulatorControl(void);
	void setManipulatorName(std::string robot_model_name, std::string joint_model_group_name);
	std::vector<double> calculateJointSetpoints(geometry_msgs::Pose end_effector_pose);
	std::vector<double> calculateJointSetpoints(geometry_msgs::Pose end_effector_pose, bool &found_ik_flag, int attempts = 10, double timeout = 1.0);
	Eigen::VectorXd calculateJointSetpoints(Eigen::Affine3d end_effector_transform, bool &found_ik_flag);
	void setJointPositions(Eigen::VectorXd joint_positions);
	geometry_msgs::PoseStamped getEndEffectorPosition(void);
	Eigen::Affine3d getEndEffectorTransform(std::vector<double> q);
	Eigen::Affine3d getEndEffectorTransform(Eigen::VectorXd q);
	std::vector<Eigen::Affine3d> getLinkPositions(Eigen::VectorXd q);
	geometry_msgs::PoseStamped getEndEffectorPositionFromQ(std::vector<double> q);
	geometry_msgs::Pose getEndEffectorReferencePosition(void);
	void LoadParameters(std::string file);
	void set_q_directions(std::vector<int> directions);
	void publishJointSetpoints(std::vector<double> q);
	std::vector<double> getJointSetpoints(void);
	std::vector<double> getJointMeasurements(void);
	bool isPositionFeasible(geometry_msgs::Pose end_effector_pose, int attempts, double timeout);
	int getNumberOfJoints(void);
	int init(ros::NodeHandle *n);
	int init();
	int getControlMode(void);
	void setControlMode(int control_mode);
	bool isStarted(void);
	Eigen::MatrixXd getJacobian(void);
	Eigen::MatrixXd getJacobian(Eigen::VectorXd q);
private:
	void qCbRos(const boost::shared_ptr<std_msgs::Float32 const> &msg, int index);
	void controlModeCbRos(const std_msgs::Int32 &msg);
	void jointControllerStateCbRos(const sensor_msgs::JointState &msg);
	void endEffectorPoseRefCbRos(const geometry_msgs::PoseStamped &msg);

	ros::Publisher dynamixel_sepoint_ros_pub_;
	ros::Publisher *manipulator_q_set_point_pub_ros_;
	std::vector<ros::Publisher> joint_setpoint_ros_pub_;
	ros::Subscriber *manipulator_q_set_point_sub_ros_;
	ros::Subscriber joint_state_sub_ros_, control_mode_sub_ros_;
	ros::Subscriber end_effector_pose_sub_ros_;
	ros::NodeHandle *n_;
	ros::NodeHandle n2_;

	robot_model_loader::RobotModelLoader *robot_model_loader_;;
	robot_state::JointModelGroup* joint_model_group_;
	robot_state::RobotModelPtr *kinematic_model_;
	robot_state::RobotStatePtr *kinematic_state_;

	geometry_msgs::Pose end_effector_pose_ref_;

	std::string robot_model_name_, joint_model_group_name_;
	float *q_torque_meas_;
	std::vector<double> q_pos_meas_, q_setpoint_;
	std::vector<int> q_directions_;
	int is_initialized_, number_of_joints_, setpoint_seq_;
	int control_mode_;
	bool start_flag_;
};

#endif