#include <aerial_manipulators_control/ManipulatorControl.h>

ManipulatorControl::ManipulatorControl(void):
	robot_model_name_("robot_description"),
	joint_model_group_name_("arm"),
	is_initialized_(0),
	number_of_joints_(0),
	start_flag_(false)
{

}

ManipulatorControl::~ManipulatorControl(void) 
{
	if (is_initialized_) delete kinematic_state_;
	delete kinematic_model_;
	delete robot_model_loader_;
	delete[] manipulator_q_set_point_sub_ros_;
	delete[] q_setpoint_;
	delete[] q_torque_meas_;
}

void ManipulatorControl::setManipulatorName(std::string robot_model_name, std::string joint_model_group_name) 
{
	robot_model_name_ = robot_model_name;
	joint_model_group_name_ = joint_model_group_name; 
}

void ManipulatorControl::q_cb_ros(const boost::shared_ptr<std_msgs::Float32 const> &msg, int index) {
	q_setpoint_[index] = /*q_directions_[index] * */msg->data;
}

void ManipulatorControl::joint_controller_state_cb_ros(const sensor_msgs::JointState &msg)
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

	const std::vector<std::string> &joint_names = joint_model_group_->getJointModelNames();

	if (number_of_joints_ <= msg.name.size()) {
		start_flag_ = true;
		for (int i = 0; i < number_of_joints_; i++) {
			for (int j = 0; j < number_of_joints_; j++) {
				if (joint_names[i].compare(msg.name[j]) == 0) {
					q_pos_meas_[i] = /*q_directions_[i] */ msg.position[j];
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

			const std::vector<std::string> &joint_names = joint_model_group_->getJointModelNames();

			number_of_joints_ = joint_names.size();

			manipulator_q_set_point_sub_ros_ = new ros::Subscriber[number_of_joints_];
			q_setpoint_ = new float[number_of_joints_];
			q_torque_meas_ = new float[number_of_joints_];
			q_pos_meas_ = std::vector<double>(number_of_joints_, 0);//new float[number_of_joints_];

			for (int i = 0; i < joint_names.size(); i++) {
				manipulator_q_set_point_sub_ros_[i] = n_->subscribe<std_msgs::Float32>(joint_names[i], 1, boost::bind(&ManipulatorControl::q_cb_ros, this, _1, i));
			}

			joint_state_sub_ros_ = n_->subscribe("joint_states", 1, &ManipulatorControl::joint_controller_state_cb_ros, this);
			manipulator_position_pub_ros_ = n_->advertise<geometry_msgs::PoseStamped>("end_effector/pose", 1);

			is_initialized_ = 1;
		}
	}

	return is_initialized_;
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
	//end_effector_pose.pose.position =
	//end_effector_pose.pose.orientation =
	ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

	return end_effector_pose;
}