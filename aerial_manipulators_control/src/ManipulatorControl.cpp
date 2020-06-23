#include <aerial_manipulators_control/ManipulatorControl.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

ManipulatorControl::ManipulatorControl(void):
	robot_model_name_("robot_description"),
	joint_model_group_name_("arm"),
	is_initialized_(0),
	number_of_joints_(0),
	start_flag_(false),
	control_mode_(0),
	setpoint_seq_(0)
{

}

ManipulatorControl::~ManipulatorControl(void) 
{
	if (is_initialized_) {
		delete kinematic_state_;
		delete kinematic_model_;
		delete robot_model_loader_;
		delete[] manipulator_q_set_point_sub_ros_;
		delete[] q_torque_meas_;
	}
}

void ManipulatorControl::setManipulatorName(std::string robot_model_name, std::string joint_model_group_name) 
{
	robot_model_name_ = robot_model_name;
	joint_model_group_name_ = joint_model_group_name; 
}

void ManipulatorControl::qCbRos(const boost::shared_ptr<std_msgs::Float32 const> &msg, int index) 
{
	q_setpoint_[index] = msg->data;
}

void ManipulatorControl::controlModeCbRos(const std_msgs::Int32 &msg)
{
	control_mode_ = msg.data;

	end_effector_pose_ref_ = getEndEffectorPosition().pose;
}

int ManipulatorControl::getControlMode(void)
{
	return control_mode_;
}

void ManipulatorControl::jointControllerStateCbRos(const sensor_msgs::JointState &msg)
{
	/*if (!start_flag_) {
		if (msg.dynamixel_state.size() > 4) {
			start_flag_ = true;
			for (int i = 0; i < 5; i++) {
				if (msg.dynamixel_state[i].id == 1)
					q1_torque_kalman_.initializePosition(q1_torque_median_.filter(msg.dynamixel_state[i].present_torque_real));
				else if (msg.dynamixel_state[i].id == 2)
					q2_torque_kalman_.initializePosition(q2_torque_median_.filter(-msg.dynamixel_state[i].present_torque_real));
				else if (msg.dynamixel_state[i].id == 3)
					q3_torque_kalman_.initializePosition(q3_torque_median_.filter(-msg.dynamixel_state[i].present_torque_real));
				else if (msg.dynamixel_state[i].id == 4)
					q4_torque_kalman_.initializePosition(q4_torque_median_.filter(msg.dynamixel_state[i].present_torque_real));
				else if (msg.dynamixel_state[i].id == 5)
					q5_torque_kalman_.initializePosition(q5_torque_median_.filter(-msg.dynamixel_state[i].present_torque_real));
			}
		}
	}*/

	const std::vector<std::string> &joint_names = joint_model_group_->getActiveJointModelNames();
	if (number_of_joints_ <= msg.name.size()) {
		start_flag_ = true;
		for (int i = 0; i < number_of_joints_; i++) {
			for (int j = 0; j < number_of_joints_; j++) {
				if (joint_names[i].compare(msg.name[j]) == 0) {
					q_pos_meas_[i] = q_directions_[i] * msg.position[j];
					break;
				}
			}
		}
	}
}

bool ManipulatorControl::isStarted(void)
{
	return start_flag_;
}

int ManipulatorControl::init(ros::NodeHandle *n) 
{
	n_ = n;

	is_initialized_ = 0;

	robot_model_loader_ = new robot_model_loader::RobotModelLoader(robot_model_name_);
	kinematic_model_ = new robot_state::RobotModelPtr();

	*kinematic_model_ = robot_model_loader_->getModel();

	if (*kinematic_model_) {
		joint_model_group_ = (*kinematic_model_)->getJointModelGroup(joint_model_group_name_);

		if (joint_model_group_) {
			kinematic_state_ = new robot_state::RobotStatePtr(new robot_state::RobotState(*kinematic_model_));
			(*kinematic_state_)->setToDefaultValues();

			const std::vector<std::string> &joint_names = joint_model_group_->getActiveJointModelNames();

			number_of_joints_ = joint_names.size();

			manipulator_q_set_point_sub_ros_ = new ros::Subscriber[number_of_joints_];
			q_setpoint_ = std::vector<double>(number_of_joints_, 0); 
			q_torque_meas_ = new float[number_of_joints_];
			q_pos_meas_ = std::vector<double>(number_of_joints_, 0);

			for (int i = 0; i < joint_names.size(); i++) {
				manipulator_q_set_point_sub_ros_[i] = n_->subscribe<std_msgs::Float32>(joint_names[i], 1, boost::bind(&ManipulatorControl::qCbRos, this, _1, i));
			}

			joint_state_sub_ros_ = n_->subscribe("joint_states", 1, &ManipulatorControl::jointControllerStateCbRos, this);
			control_mode_sub_ros_ = n_->subscribe("control_mode", 1, &ManipulatorControl::controlModeCbRos, this);
			end_effector_pose_sub_ros_ = n_->subscribe("end_effector/pose_ref", 1, &ManipulatorControl::endEffectorPoseRefCbRos, this);

			dynamixel_sepoint_ros_pub_ = n_->advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);

			is_initialized_ = 1;
		}
	}

	if (!is_initialized_) {
		delete kinematic_model_;
		delete robot_model_loader_;
	}

	return is_initialized_;
}

int ManipulatorControl::init() 
{	
	n2_ = ros::NodeHandle();
	n_ = &n2_;

	is_initialized_ = 0;

	robot_model_loader_ = new robot_model_loader::RobotModelLoader(robot_model_name_);
	kinematic_model_ = new robot_state::RobotModelPtr();
	
	*kinematic_model_ = robot_model_loader_->getModel();

	if (*kinematic_model_) {
		joint_model_group_ = (*kinematic_model_)->getJointModelGroup(joint_model_group_name_);

		if (joint_model_group_) {
			kinematic_state_ = new robot_state::RobotStatePtr(new robot_state::RobotState(*kinematic_model_));
			(*kinematic_state_)->setToDefaultValues();

			const std::vector<std::string> &joint_names = joint_model_group_->getActiveJointModelNames();

			number_of_joints_ = joint_names.size();

			manipulator_q_set_point_sub_ros_ = new ros::Subscriber[number_of_joints_];
			q_setpoint_ = std::vector<double>(number_of_joints_, 0); 
			q_torque_meas_ = new float[number_of_joints_];
			q_pos_meas_ = std::vector<double>(number_of_joints_, 0);

			for (int i = 0; i < joint_names.size(); i++) {
				manipulator_q_set_point_sub_ros_[i] = n_->subscribe<std_msgs::Float32>(joint_names[i], 1, boost::bind(&ManipulatorControl::qCbRos, this, _1, i));
			}

			joint_state_sub_ros_ = n_->subscribe("joint_states", 1, &ManipulatorControl::jointControllerStateCbRos, this);
			control_mode_sub_ros_ = n_->subscribe("control_mode", 1, &ManipulatorControl::controlModeCbRos, this);
			end_effector_pose_sub_ros_ = n_->subscribe("end_effector/pose_ref", 1, &ManipulatorControl::endEffectorPoseRefCbRos, this);

			dynamixel_sepoint_ros_pub_ = n_->advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);

			is_initialized_ = 1;
		}
	}

	if (!is_initialized_) {
		delete kinematic_model_;
		delete robot_model_loader_;
	}

	return is_initialized_;
}


void ManipulatorControl::endEffectorPoseRefCbRos(const geometry_msgs::PoseStamped &msg)
{
	end_effector_pose_ref_ = msg.pose;
}

geometry_msgs::Pose ManipulatorControl::getEndEffectorReferencePosition(void)
{
	return end_effector_pose_ref_;
}

geometry_msgs::PoseStamped ManipulatorControl::getEndEffectorPosition(void)
{
	geometry_msgs::PoseStamped end_effector_pose;

	int number_of_links = (*kinematic_model_)->getLinkModels().size();
	std::string end_effector_name = (*kinematic_model_)->getLinkModels()[number_of_links-1]->getName();

	(*kinematic_state_)->setJointGroupPositions(joint_model_group_, q_pos_meas_);
	const Eigen::Affine3d &end_effector_state = (*kinematic_state_)->getGlobalLinkTransform(end_effector_name);

	end_effector_pose.header.stamp = ros::Time::now();
	end_effector_pose.header.frame_id = (*kinematic_model_)->getModelFrame().c_str();
	end_effector_pose.header.frame_id = (*kinematic_model_)->getModelFrame();

	tf::poseEigenToMsg(end_effector_state, end_effector_pose.pose);

	return end_effector_pose;
}

Eigen::Affine3d ManipulatorControl::getEndEffectorTransform(std::vector<double> q)
{
	geometry_msgs::PoseStamped end_effector_pose;

	int number_of_links = (*kinematic_model_)->getLinkModels().size();
	std::string end_effector_name = (*kinematic_model_)->getLinkModels()[number_of_links-1]->getName();

	(*kinematic_state_)->setJointGroupPositions(joint_model_group_, q);
	const Eigen::Affine3d &end_effector_state = (*kinematic_state_)->getGlobalLinkTransform(end_effector_name);

	end_effector_pose.header.stamp = ros::Time::now();
	end_effector_pose.header.frame_id = (*kinematic_model_)->getModelFrame().c_str();
	end_effector_pose.header.frame_id = (*kinematic_model_)->getModelFrame();

	tf::poseEigenToMsg(end_effector_state, end_effector_pose.pose);

	return end_effector_state;
}

Eigen::Affine3d ManipulatorControl::getEndEffectorTransform(Eigen::VectorXd q)
{
	geometry_msgs::PoseStamped end_effector_pose;

	int number_of_links = (*kinematic_model_)->getLinkModels().size();
	std::string end_effector_name = (*kinematic_model_)->getLinkModels()[number_of_links-1]->getName();

	(*kinematic_state_)->setJointGroupPositions(joint_model_group_, q);
	const Eigen::Affine3d &end_effector_state = (*kinematic_state_)->getGlobalLinkTransform(end_effector_name);

	end_effector_pose.header.stamp = ros::Time::now();
	end_effector_pose.header.frame_id = (*kinematic_model_)->getModelFrame().c_str();
	end_effector_pose.header.frame_id = (*kinematic_model_)->getModelFrame();

	tf::poseEigenToMsg(end_effector_state, end_effector_pose.pose);

	return end_effector_state;
}

std::vector<Eigen::Affine3d> ManipulatorControl::getLinkPositions(Eigen::VectorXd q)
{
	std::vector<Eigen::Affine3d> link_positions;
	geometry_msgs::PoseStamped end_effector_pose;

	// Get number of links
	int number_of_links = (*kinematic_model_)->getLinkModels().size();

	// Loop through all links
	for (int i=0; i<number_of_links; i++){
		std::string link_name = (*kinematic_model_)->getLinkModels()[i]->getName();

		(*kinematic_state_)->setJointGroupPositions(joint_model_group_, q);
		const Eigen::Affine3d &link_state = (*kinematic_state_)->getGlobalLinkTransform(link_name);

		link_positions.push_back(link_state);
	}

	return link_positions;
}

std::vector<double> ManipulatorControl::getJointSetpoints(void)
{
	return q_setpoint_; 	
}

void ManipulatorControl::publishJointSetpoints(std::vector<double> q) {
	trajectory_msgs::JointTrajectory joint_setpoints;
	trajectory_msgs::JointTrajectoryPoint joint_setpoint;

	const std::vector<std::string> &joint_names = joint_model_group_->getActiveJointModelNames();

	joint_setpoints.header.seq = setpoint_seq_++;
	joint_setpoints.header.stamp = ros::Time::now();
	joint_setpoints.header.frame_id = "world";

	(*kinematic_state_)->setJointGroupPositions(joint_model_group_, q);
	(*kinematic_state_)->enforceBounds();
	(*kinematic_state_)->copyJointGroupPositions(joint_model_group_name_, q);

	for (int i = 0; i < q.size(); i++) {
		joint_setpoints.joint_names.push_back(joint_names[i]);
		joint_setpoint.positions.push_back(q_directions_[i] * q[i]);
		joint_setpoint.velocities.push_back(0.0);
		joint_setpoint.accelerations.push_back(0.0);
		joint_setpoint.effort.push_back(0.0);
	}

	joint_setpoint.time_from_start = ros::Duration(1);

	joint_setpoints.points.push_back(joint_setpoint);

	dynamixel_sepoint_ros_pub_.publish(joint_setpoints);
}

void ManipulatorControl::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);

	q_directions_ = config["directions"].as<std::vector<int> >();
}

std::vector<double> ManipulatorControl::calculateJointSetpoints(geometry_msgs::Pose end_effector_pose)
{
	std::vector<double> q(number_of_joints_, 0);

	bool found_ik = (*kinematic_state_)->setFromIK(joint_model_group_, end_effector_pose, 10, 1);

	if (found_ik)
		(*kinematic_state_)->copyJointGroupPositions(joint_model_group_, q);
	else
	{
		ROS_INFO("Did not find IK solution");

		for (int i = 0; i < number_of_joints_; i++)
			q[i] = q_pos_meas_[i];
	}

	return q;
}


std::vector<double> ManipulatorControl::calculateJointSetpoints(geometry_msgs::Pose end_effector_pose, bool &found_ik_flag)
{
	std::vector<double> q(number_of_joints_, 0);

	bool found_ik = (*kinematic_state_)->setFromIK(joint_model_group_, end_effector_pose, 10, 1);

	if (found_ik)
		(*kinematic_state_)->copyJointGroupPositions(joint_model_group_, q);
	else
	{
		ROS_INFO("Did not find IK solution");

		for (int i = 0; i < number_of_joints_; i++)
			q[i] = q_pos_meas_[i];
	}

	found_ik_flag = found_ik;
	return q;
}

Eigen::VectorXd ManipulatorControl::calculateJointSetpoints(Eigen::Affine3d end_effector_transform, bool &found_ik_flag)
{
	std::vector<double> q_vect(number_of_joints_, 0);

	geometry_msgs::Pose end_effector_pose;
	end_effector_pose.position.x = end_effector_transform.translation()[0];
	end_effector_pose.position.y = end_effector_transform.translation()[1];
	end_effector_pose.position.z = end_effector_transform.translation()[2];
	end_effector_pose.orientation.x = Eigen::Quaterniond(end_effector_transform.rotation()).x();
	end_effector_pose.orientation.y = Eigen::Quaterniond(end_effector_transform.rotation()).y();
	end_effector_pose.orientation.z = Eigen::Quaterniond(end_effector_transform.rotation()).z();
	end_effector_pose.orientation.w = Eigen::Quaterniond(end_effector_transform.rotation()).w();

	kinematics::KinematicsQueryOptions kinematics_options;
	kinematics_options.discretization_method = 
		kinematics::DiscretizationMethods::DiscretizationMethod::ALL_DISCRETIZED;
	kinematics_options.return_approximate_solution = true;

	bool found_ik = (*kinematic_state_)->setFromIK(joint_model_group_, end_effector_pose, 10, 0.5, 
		moveit::core::GroupStateValidityCallbackFn(), kinematics_options);

	if (found_ik)
		(*kinematic_state_)->copyJointGroupPositions(joint_model_group_, q_vect);
	else
	{
		ROS_INFO("Did not find IK solution");

		for (int i = 0; i < number_of_joints_; i++)
			q_vect[i] = q_pos_meas_[i];
	}

	Eigen::VectorXd q(number_of_joints_);
	for (int i=0; i<number_of_joints_; i++){
		q(i) = q_vect[i];
	}

	found_ik_flag = found_ik;
	return q;
}
