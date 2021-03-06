#include <aerial_manipulators/WPManipulatorControl.h>
#include "geometry_msgs/WrenchStamped.h"

WPManipulatorControl::WPManipulatorControl():
	q1_torque_meas_(0),
	q2_torque_meas_(0),
	q3_torque_meas_(0),
	q4_torque_meas_(0),
	q5_torque_meas_(0),
	q1_pos_meas_(0),
	q2_pos_meas_(0),
	q3_pos_meas_(0),
	q4_pos_meas_(0),
	q5_pos_meas_(0), 
	start_flag_(false),
	manipulator_mode_(0),
	q1_setpoint_(1.57),
	q2_setpoint_(-0.4),
	q3_setpoint_(1.57),
	q4_setpoint_(-1.57),
	q5_setpoint_(2.3),
	new_trajectory_(false),
	trajectory_length_(0),
	trajectory_index_(0),
	trajectory_rate_(100)
{
	rate_ = 30;

	new_dynamixel_measurement_ = false;

	joint_state_sub_ros_ = n_.subscribe("dynamixel_state", 1, &WPManipulatorControl::joint_controller_state_cb_ros, this);
	mmuav_position_sub_ros_ = n_.subscribe("pose", 1, &WPManipulatorControl::mmuav_position_cb_ros, this);
	wp_manipulator_end_effector_position_sub_ros_ = n_.subscribe("wp_manipulator/position_set_point", 1, &WPManipulatorControl::wp_manipulator_end_effector_position_cb_ros, this);
	wp_manipulator_end_effector_trajectory_sub_ros_ = n_.subscribe("wp_manipulator/trajectory", 1, &WPManipulatorControl::wp_manipulator_end_effector_trajectory_cb_ros, this);
	
	mode_sub_ros_ = n_.subscribe("mode", 1, &WPManipulatorControl::mode_cb_ros, this);
	wp_manipulator_q1_set_point_sub_ros_ = n_.subscribe("wp_manipulator/q1", 1, &WPManipulatorControl::q1_cb_ros, this);
	wp_manipulator_q2_set_point_sub_ros_ = n_.subscribe("wp_manipulator/q2", 1, &WPManipulatorControl::q2_cb_ros, this);
	wp_manipulator_q3_set_point_sub_ros_ = n_.subscribe("wp_manipulator/q3", 1, &WPManipulatorControl::q3_cb_ros, this);
	wp_manipulator_q4_set_point_sub_ros_ = n_.subscribe("wp_manipulator/q4", 1, &WPManipulatorControl::q4_cb_ros, this);
	wp_manipulator_q5_set_point_sub_ros_ = n_.subscribe("wp_manipulator/q5", 1, &WPManipulatorControl::q5_cb_ros, this);

	manipulator_wrench_ros_pub_ = n_.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
	dynamixel_sepoint_ros_pub_ = n_.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 1);
	manipulator_position_pub_ros_ = n_.advertise<geometry_msgs::PoseStamped>("position", 1);

	wrench_zero_all_srv_ros_ = n_.advertiseService("wrench_zero_all", &WPManipulatorControl::wrench_zero_all_cb, this);

	Tworld_end_effector_dk << 1, 0, 0, 0,
			  			      0, 1, 0, 0, 
			  			      0, 0, 1, 0,
			  			      0, 0, 0, 1;

	Tworld_uav_origin_ << 1, 0, 0, 0,
			  			  0, 1, 0, 0, 
			  			  0, 0, 1, 0,
			  			  0, 0, 0, 1;

	Tuav_origin_world_ << 1, 0, 0, 0,
			  			  0, 1, 0, 0, 
			  			  0, 0, 1, 0,
			  			  0, 0, 0, 1;

	Tworld_wp_end_effector_ref_ << 1, 0, 0, 0,
			  					   0, 1, 0, 0, 
			  					   0, 0, 1, 0,
			  					   0, 0, 0, 1;
	Tuav_origin_arm_inv_ << 1, 0, 0, 0,
			  			    0, 1, 0, 0, 
			  			    0, 0, 1, 0,
			  				0, 0, 0, 1;

	q1_torque_kalman_.setPositionNoise(1);
	q1_torque_kalman_.setVelocityNoise(2);
	q1_torque_kalman_.setMeasureNoise(5);

	q2_torque_kalman_.setPositionNoise(1);
	q2_torque_kalman_.setVelocityNoise(2);
	q2_torque_kalman_.setMeasureNoise(5);

	q3_torque_kalman_.setPositionNoise(1);
	q3_torque_kalman_.setVelocityNoise(2);
	q3_torque_kalman_.setMeasureNoise(5);

	q4_torque_kalman_.setPositionNoise(1);
	q4_torque_kalman_.setVelocityNoise(2);
	q4_torque_kalman_.setMeasureNoise(5);

	q5_torque_kalman_.setPositionNoise(1);
	q5_torque_kalman_.setVelocityNoise(2);
	q5_torque_kalman_.setMeasureNoise(5);

	q1_torque_median_.init(7);
	q2_torque_median_.init(7);
	q3_torque_median_.init(7);
	q4_torque_median_.init(7);
	q5_torque_median_.init(7);

	wrench_.resize(6,1);
}

bool WPManipulatorControl::wrench_zero_all_cb(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) 
{
	wrench_ << 0, 0, 0, 0, 0, 0;
	return true;
}

void WPManipulatorControl::mode_cb_ros(const std_msgs::Int32 &msg) 
{
	if (manipulator_mode_ == 0 && (msg.data == 1 || msg.data == 2)) {
		Tworld_wp_end_effector_ref_ = Tworld_end_effector_dk;
		trajectory_reference_.points.clear();
	}
	manipulator_mode_ = msg.data;
}

double WPManipulatorControl::deadzone(double value, double lower_limit, double upper_limit)
{
	if (value > upper_limit) return (value-upper_limit);
	else if (value < lower_limit) return (value-lower_limit);
	else return 0;
}

void WPManipulatorControl::wp_manipulator_end_effector_trajectory_cb_ros(const trajectory_msgs::MultiDOFJointTrajectory &msg)
{
	if (manipulator_mode_ == 2) {
		new_trajectory_ = true;
		trajectory_index_ = 0;
		trajectory_length_ = msg.points.size();
		trajectory_reference_ = msg;
	}

}

void WPManipulatorControl::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);
	std::vector<double> theta, a, d, arm_origin, arm_upper_limits;
	std::vector<double> arm_lower_limits;
	WPManipulatorDirectKinematics::DH_Parameters_TypeDef dhParams;

	theta = config["theta"].as<std::vector<double> >();
	a = config["a"].as<std::vector<double> >();
	d = config["d"].as<std::vector<double> >();
	arm_origin = config["origin"].as<std::vector<double> >();
	arm_upper_limits = config["limits"]["upper"].as<std::vector<double> >();
	arm_lower_limits = config["limits"]["lower"].as<std::vector<double> >();


	Tuav_origin_arm_ <<  cos(arm_origin[5])*cos(arm_origin[4]),  cos(arm_origin[5])*sin(arm_origin[4])*sin(arm_origin[3])-sin(arm_origin[5])*cos(arm_origin[3]),  cos(arm_origin[5])*sin(arm_origin[4])*cos(arm_origin[3])+sin(arm_origin[5])*sin(arm_origin[3]), arm_origin[0],
					     sin(arm_origin[5])*cos(arm_origin[4]),  sin(arm_origin[5])*sin(arm_origin[4])*sin(arm_origin[3])+cos(arm_origin[5])*cos(arm_origin[3]),  sin(arm_origin[5])*sin(arm_origin[4])*cos(arm_origin[3])-cos(arm_origin[5])*sin(arm_origin[3]), arm_origin[1],
					    -sin(arm_origin[4]),                          cos(arm_origin[4])*sin(arm_origin[3]),                                      									 cos(arm_origin[4])*cos(arm_origin[3]),                   													  arm_origin[2],
					     0,                                                0,                                                																	     0,                           																							  1;
	

	for (int i=0; i<6; i++)
	{
		dhParams.theta[i] = theta[i];
		dhParams.alpha[i] = 0;
		dhParams.d[i] = d[i];
		dhParams.a[i] = a[i];
		arm_upper_limits_[i] = arm_upper_limits[i];
		arm_lower_limits_[i] = arm_lower_limits[i];
	}

	for (int i=0; i<5; i++) 
	{
		arm_upper_limits_[i] = arm_upper_limits[i];
		arm_lower_limits_[i] = arm_lower_limits[i];
	}

	Tuav_origin_arm_inv_ = Tuav_origin_arm_.inverse();
	
	manipulator_inverse.LoadParameters(file);
	manipulator_direct.LoadParameters(file);
}

void WPManipulatorControl::start()
{
	ros::Rate loop_rate(rate_);

	geometry_msgs::PoseStamped manipulator_pose;
	geometry_msgs::WrenchStamped manipulator_wrench;
	sensor_msgs::JointState  dynamixel_setpoint;

	dynamixel_setpoint.position.push_back(1.57);
	dynamixel_setpoint.position.push_back(-0.4);
	dynamixel_setpoint.position.push_back(1.57);
	dynamixel_setpoint.position.push_back(-1.57);
	dynamixel_setpoint.position.push_back(2.3);

	Eigen::Matrix4d T16_dk, T16_ref, T12_dk, T26_ref;
	Eigen::MatrixXd J, Tau(6,1), dX(6,1), dq(6,1);
	int nbr_of_solutions;

	float orientationEuler[3], orientationQuaternion[4], Q[5], position[3], q[4];
	float rot_T26_z[4], feasible_angle;

	/*while (!start_flag_ && ros::ok()) {
		ros::spinOnce();
		printf("Waiting for torque measurements.\n");
		ros::Duration(0.5).sleep();
	}*/


	while (ros::ok())
	{
		ros::spinOnce();

		//direct kinematics
		T16_dk = manipulator_direct.dk_calculate(q1_pos_meas_, q2_pos_meas_, q3_pos_meas_, q4_pos_meas_, q5_pos_meas_);
		Tworld_end_effector_dk = Tworld_uav_origin_ * Tuav_origin_arm_ * T16_dk;

		getAnglesFromRotationTranslationMatrix(Tworld_end_effector_dk, orientationEuler);
		euler2quaternion(orientationEuler, orientationQuaternion);

		manipulator_pose.header.stamp = ros::Time::now();
		manipulator_pose.header.frame_id = "wp_manipulator";
		manipulator_pose.pose.position.x = Tworld_end_effector_dk(0,3);
		manipulator_pose.pose.position.y = Tworld_end_effector_dk(1,3);
		manipulator_pose.pose.position.z = Tworld_end_effector_dk(2,3);
		manipulator_pose.pose.orientation.x = orientationQuaternion[1];
		manipulator_pose.pose.orientation.y = orientationQuaternion[2];
		manipulator_pose.pose.orientation.z = orientationQuaternion[3];
		manipulator_pose.pose.orientation.w = orientationQuaternion[0];

		//force calculation
		q1_torque_kalman_.modelUpdate(1.0/rate_);
		q2_torque_kalman_.modelUpdate(1.0/rate_);
		q3_torque_kalman_.modelUpdate(1.0/rate_);
		q4_torque_kalman_.modelUpdate(1.0/rate_);
		q5_torque_kalman_.modelUpdate(1.0/rate_);

		if (new_dynamixel_measurement_) {
			new_dynamixel_measurement_ = false;
			q1_torque_kalman_.measureUpdate(q1_torque_meas_);
			q2_torque_kalman_.measureUpdate(q2_torque_meas_);
			q3_torque_kalman_.measureUpdate(q3_torque_meas_);
			q4_torque_kalman_.measureUpdate(q4_torque_meas_);
			q5_torque_kalman_.measureUpdate(q5_torque_meas_);
		}

		J = manipulator_inverse.getJacobian(q1_pos_meas_, q2_pos_meas_, q3_pos_meas_, q4_pos_meas_, q5_pos_meas_);
		Tau << deadzone(q1_torque_kalman_.getVelocity(), 0, 0), 
			   deadzone(q2_torque_kalman_.getVelocity(), 0, 0), 
			   deadzone(q3_torque_kalman_.getVelocity(), 0, 0),
			   deadzone(q4_torque_kalman_.getVelocity(), 0, 0),
			   deadzone(q5_torque_kalman_.getVelocity(), 0, 0),
			   0;

		wrench_ += J*Tau;

		if (manipulator_mode_ == 0) //dk mode
		{
			Q[0] = q1_setpoint_;
			Q[1] = q2_setpoint_;
			Q[2] = q3_setpoint_;
			Q[3] = q4_setpoint_;
			Q[4] = q5_setpoint_;

			limitJointsPosition(Q);

			dynamixel_setpoint.position[0] = Q[0];
			dynamixel_setpoint.position[1] = Q[1];
			dynamixel_setpoint.position[2] = Q[2];
			dynamixel_setpoint.position[3] = Q[3];
			dynamixel_setpoint.position[4] = Q[4];
			dynamixel_sepoint_ros_pub_.publish(dynamixel_setpoint);

		}
		else if (manipulator_mode_ == 1) //ik mode
		{
			T16_ref = Tuav_origin_arm_inv_ * Tuav_origin_world_ * Tworld_wp_end_effector_ref_;
			nbr_of_solutions = manipulator_inverse.ik_T12_calculate(T16_ref(1,3), M_PI);

			if (joint12_criterion_function(manipulator_inverse.getQ1(), manipulator_inverse.getQ2(), q1_pos_meas_, q2_pos_meas_, Q, nbr_of_solutions))
			{
				limitJointsPosition(Q);

				dynamixel_setpoint.position[0] = Q[0];
				dynamixel_setpoint.position[1] = -Q[1];
			}

			T12_dk = manipulator_direct.dk_T12_calculate(dynamixel_setpoint.position[0], dynamixel_setpoint.position[1]);
			T26_ref = T12_dk.inverse() * T16_ref;

			std::cout<<T16_ref<<std::endl;
			std::cout<<T26_ref<<std::endl;
			std::cout<<T12_dk<<std::endl;

			nbr_of_solutions = manipulator_inverse.getFeasibleRotationT26(T26_ref(0,3), T26_ref(1,3), rot_T26_z);

			if (nbr_of_solutions) 
			{
				feasible_angle = angle_criterion_function(rot_T26_z, q3_pos_meas_, q4_pos_meas_, q5_pos_meas_);

				std::cout<<feasible_angle<<std::endl;

				nbr_of_solutions = manipulator_inverse.ik_T26_calculate(T26_ref(0,3), T26_ref(1,3), feasible_angle);

				/*for (int i=0; i <nbr_of_solutions; i++)
				{
					std::cout<<manipulator_inverse.getQ3()[i]<<std::endl;
					std::cout<<manipulator_inverse.getQ4()[i]<<std::endl;
					std::cout<<manipulator_inverse.getQ5()[i]<<std::endl;
					std::cout<<""<<std::endl;
				}*/
				

				if (joint26_criterion_function(manipulator_inverse.getQ3(), manipulator_inverse.getQ4(), manipulator_inverse.getQ5(), 
					q3_pos_meas_, q4_pos_meas_, q5_pos_meas_, Q, nbr_of_solutions))
				{
					limitJointsPosition(Q);
					dynamixel_setpoint.position[2] = -Q[0];
					dynamixel_setpoint.position[3] = Q[1];
					dynamixel_setpoint.position[4] = -Q[2];
				}

			}

			std::cout<<"Q1: "<<dynamixel_setpoint.position[0]<<std::endl;
			std::cout<<"Q2: "<<dynamixel_setpoint.position[1]<<std::endl;
			std::cout<<"Q3: "<<dynamixel_setpoint.position[2]<<std::endl;
			std::cout<<"Q4: "<<dynamixel_setpoint.position[3]<<std::endl;
			std::cout<<"Q5: "<<dynamixel_setpoint.position[4]<<std::endl;
			std::cout<<""<<std::endl;

			dynamixel_sepoint_ros_pub_.publish(dynamixel_setpoint);


			//Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(J);

			/*T16_ref = Tuav_origin_arm_inv_ * Tuav_origin_world_ * Tworld_wp_end_effector_ref_;

			dX << T16_ref(0,3) - T16_dk(0,3),
			      T16_ref(1,3) - T16_dk(1,3),
			      T16_ref(2,3) - T16_dk(2,3),
			      0,
			      0,
			      0;

			dq = J.transpose()*dX;

			//Eigen::MatrixXd psdJ = Eigen::pseudoinverse(J)

			Q[0] = q1_pos_meas_ + dq(0,0);
			Q[1] = q2_pos_meas_ + dq(1,0);
			Q[2] = q3_pos_meas_ + dq(2,0);
			Q[3] = q4_pos_meas_ + dq(3,0);
			Q[4] = q5_pos_meas_ + dq(4,0);

			limitJointsPosition(Q);

			std::cout<<"Q1: "<<Q[0]<<std::endl;
			std::cout<<"Q2: "<<Q[1]<<std::endl;
			std::cout<<"Q3: "<<Q[2]<<std::endl;
			std::cout<<"Q4: "<<Q[3]<<std::endl;
			std::cout<<"Q5: "<<Q[4]<<std::endl;
			std::cout<<std::endl;

			dynamixel_setpoint.position[0] = Q[0];
			dynamixel_setpoint.position[1] = -Q[1];
			dynamixel_setpoint.position[2] = -Q[2];
			dynamixel_setpoint.position[3] = Q[3];
			dynamixel_setpoint.position[4] = -Q[4];
			dynamixel_sepoint_ros_pub_.publish(dynamixel_setpoint);*/



			//inverse kinematics
			/*T16_ref = Tuav_origin_arm_inv_ * Tuav_origin_world_ * Tworld_wp_end_effector_ref_;
			getAnglesFromRotationTranslationMatrix(T16_ref, orientationEuler);
			nbr_of_solutions = manipulator_inverse.ik_calculate(T16_ref(0,3), T16_ref(1,3), T16_ref(2,3), orientationEuler[1], orientationEuler[2]);

			if (joint_criterion_function(manipulator_inverse.getQ1(), manipulator_inverse.getQ2(), manipulator_inverse.getQ3(), 
			manipulator_inverse.getQ4(), manipulator_inverse.getQ5(), q1_pos_meas_, q2_pos_meas_, q3_pos_meas_, q4_pos_meas_, q5_pos_meas_, Q, nbr_of_solutions)) 
			{

				std::cout<<"Q1: "<<Q[0]<<std::endl;
				std::cout<<"Q2: "<<Q[1]<<std::endl;
				std::cout<<"Q3: "<<Q[2]<<std::endl;
				std::cout<<"Q4: "<<Q[3]<<std::endl;
				std::cout<<"Q5: "<<Q[4]<<std::endl;
				std::cout<<std::endl;

				limitJointsPosition(Q);

				dynamixel_setpoint.position[0] = Q[0];
				dynamixel_setpoint.position[1] = -Q[1];
				dynamixel_setpoint.position[2] = -Q[2];
				dynamixel_setpoint.position[3] = Q[3];
				dynamixel_setpoint.position[4] = -Q[4];
				dynamixel_sepoint_ros_pub_.publish(dynamixel_setpoint);
			}*/

		}
		else if (manipulator_mode_ == 2) {

			if (new_trajectory_) 
			{
				if (trajectory_index_ < trajectory_length_) 
				{

					q[0] = trajectory_reference_.points[trajectory_index_].transforms[0].rotation.w;
					q[1] = trajectory_reference_.points[trajectory_index_].transforms[0].rotation.x;
					q[2] = trajectory_reference_.points[trajectory_index_].transforms[0].rotation.y;
					q[3] = trajectory_reference_.points[trajectory_index_].transforms[0].rotation.z;

					position[0] = trajectory_reference_.points[trajectory_index_].transforms[0].translation.x;
					position[1] = trajectory_reference_.points[trajectory_index_].transforms[0].translation.y;
					position[2] = trajectory_reference_.points[trajectory_index_].transforms[0].translation.z;

					quaternion2euler(q, orientationEuler);

					getRotationTranslationMatrix(Tworld_wp_end_effector_ref_, orientationEuler, position);

					trajectory_index_ += trajectory_rate_/rate_;
				}
				else
				{
					new_trajectory_ = false;
				}
			}

			//inverse kinematics
			/*T16_ref = Tuav_origin_arm_inv_ * Tuav_origin_world_ * Tworld_wp_end_effector_ref_;
			getAnglesFromRotationTranslationMatrix(T16_ref, orientationEuler);
			nbr_of_solutions = manipulator_inverse.ik_calculate(T16_ref(0,3), T16_ref(1,3), T16_ref(2,3), orientationEuler[1], orientationEuler[2]);

			if (joint_criterion_function(manipulator_inverse.getQ1(), manipulator_inverse.getQ2(), manipulator_inverse.getQ3(), 
			manipulator_inverse.getQ4(), manipulator_inverse.getQ5(), q1_pos_meas_, q2_pos_meas_, q3_pos_meas_, q4_pos_meas_, q5_pos_meas_, Q, nbr_of_solutions)) 
			{

				limitJointsPosition(Q);

				dynamixel_setpoint.position[0] = Q[0];
				dynamixel_setpoint.position[1] = -Q[1];
				dynamixel_setpoint.position[2] = -Q[2];
				dynamixel_setpoint.position[3] = Q[3];
				dynamixel_setpoint.position[4] = -Q[4];
				dynamixel_sepoint_ros_pub_.publish(dynamixel_setpoint);
			}*/

			
		}

		manipulator_wrench.header.stamp = ros::Time::now();
		manipulator_wrench.header.frame_id = "wp_manipulator";
		manipulator_wrench.wrench.force.x = wrench_(0,0);
		manipulator_wrench.wrench.force.y = wrench_(1,0);
		manipulator_wrench.wrench.force.z = wrench_(2,0);
		manipulator_wrench.wrench.torque.x = wrench_(3,0);
		manipulator_wrench.wrench.torque.y = wrench_(4,0);
		manipulator_wrench.wrench.torque.z = wrench_(5,0);
		manipulator_wrench_ros_pub_.publish(manipulator_wrench);

		manipulator_position_pub_ros_.publish(manipulator_pose);

		loop_rate.sleep();
	}
}

void WPManipulatorControl::q1_cb_ros(const std_msgs::Float32 &msg)
{
	q1_setpoint_ = msg.data;
}

void WPManipulatorControl::q2_cb_ros(const std_msgs::Float32 &msg)
{
	q2_setpoint_ = -msg.data;
}
void WPManipulatorControl::q3_cb_ros(const std_msgs::Float32 &msg)
{
	q3_setpoint_ = -msg.data;
}

void WPManipulatorControl::q4_cb_ros(const std_msgs::Float32 &msg)
{
	q4_setpoint_ = msg.data;
}

void WPManipulatorControl::q5_cb_ros(const std_msgs::Float32 &msg)
{
	q5_setpoint_ = -msg.data;
}


void WPManipulatorControl::joint_controller_state_cb_ros(const dynamixel_workbench_msgs::DynamixelStateList &msg)
{
	if (!start_flag_) {
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
	}

	if (msg.dynamixel_state.size() > 4) {
		new_dynamixel_measurement_ = true;
		for (int i = 0; i < 5; i++) {
			if (msg.dynamixel_state[i].id == 1) {
				q1_pos_meas_ = msg.dynamixel_state[i].present_position_real;
				q1_torque_meas_ = q1_torque_median_.filter(msg.dynamixel_state[i].present_torque_real);
			}
			else if (msg.dynamixel_state[i].id == 2) {
				q2_pos_meas_ = -msg.dynamixel_state[i].present_position_real;
				q2_torque_meas_ = q2_torque_median_.filter(-msg.dynamixel_state[i].present_torque_real);
			}
			else if (msg.dynamixel_state[i].id == 3) {
				q3_pos_meas_ = -msg.dynamixel_state[i].present_position_real;
				q3_torque_meas_ = q3_torque_median_.filter(-msg.dynamixel_state[i].present_torque_real);
			}
			else if (msg.dynamixel_state[i].id == 4) {
				q4_pos_meas_ = msg.dynamixel_state[i].present_position_real;
				q4_torque_meas_ = q4_torque_median_.filter(msg.dynamixel_state[i].present_torque_real);
			}
			else if (msg.dynamixel_state[i].id == 5) {
				q5_pos_meas_ = -msg.dynamixel_state[i].present_position_real;
				q5_torque_meas_ = q5_torque_median_.filter(-msg.dynamixel_state[i].present_torque_real);
			}
		}
	}
}

void WPManipulatorControl::wp_manipulator_end_effector_position_cb_ros(const geometry_msgs::PoseStamped &msg)
{
	float orientationEuler[3], position[3];
	float q[4];

	q[0] = msg.pose.orientation.w;
	q[1] = msg.pose.orientation.x;
	q[2] = msg.pose.orientation.y;
	q[3] = msg.pose.orientation.z;

	position[0] = msg.pose.position.x;
	position[1] = msg.pose.position.y;
	position[2] = msg.pose.position.z;

	quaternion2euler(q, orientationEuler);

	getRotationTranslationMatrix(Tworld_wp_end_effector_ref_, orientationEuler, position);

}

void WPManipulatorControl::mmuav_position_cb_ros(const geometry_msgs::PoseStamped &msg)
{
	ros::Time t;
	ros::Duration dt;
	float position[3], q[4], orientationEuler[3];

	position[0] = msg.pose.position.x;
	position[1] = msg.pose.position.y;
	position[2] = msg.pose.position.z;

	q[0] = msg.pose.orientation.w;
    q[1] = msg.pose.orientation.x;
    q[2] = msg.pose.orientation.y;
    q[3] = msg.pose.orientation.z;

    quaternion2euler(q, orientationEuler);

    orientationEuler[0] = 0.0;
    orientationEuler[1] = 0.0;//orientationEuler

	getRotationTranslationMatrix(Tworld_uav_origin_, orientationEuler, position);

	Tuav_origin_world_ = Tworld_uav_origin_.inverse();
}

void WPManipulatorControl::quaternion2euler(float *quaternion, float *euler)
{
  euler[0] = atan2(2 * (quaternion[0] * quaternion[1] + 
    quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1]
    + quaternion[2] * quaternion[2]));

  euler[1] = asin(2 * (quaternion[0] * quaternion[2] -
    quaternion[3] * quaternion[1]));

  euler[2] = atan2(2 * (quaternion[0]*quaternion[3] +
    quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] +
    quaternion[3] * quaternion[3]));
}

void WPManipulatorControl::euler2quaternion(float *euler, float *quaternion)
{
	float cy = cos(euler[2] * 0.5);
	float sy = sin(euler[2] * 0.5);
	float cr = cos(euler[0] * 0.5);
	float sr = sin(euler[0] * 0.5);
	float cp = cos(euler[1] * 0.5);
	float sp = sin(euler[1] * 0.5);

	quaternion[0] = cy * cr * cp + sy * sr * sp; //w
	quaternion[1] = cy * sr * cp - sy * cr * sp; //x
	quaternion[2] = cy * cr * sp + sy * sr * cp; //y
	quaternion[3] = sy * cr * cp - cy * sr * sp; //z
}

void WPManipulatorControl::getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
  float *orientationEuler, float *position)
{
  float r11, r12, r13, t1, r21, r22, r23, t2;
  float r31, r32, r33, t3;

  float x, y, z;

  x = orientationEuler[0];
  y = orientationEuler[1];
  z = orientationEuler[2];

  r11 = cos(y)*cos(z);

  r12 = cos(z)*sin(x)*sin(y) - cos(x)*sin(z);

  r13 = sin(x)*sin(z) + cos(x)*cos(z)*sin(y);

  r21 = cos(y)*sin(z);

  r22 = cos(x)*cos(z) + sin(x)*sin(y)*sin(z);

  r23 = cos(x)*sin(y)*sin(z) - cos(z)*sin(x);

  r31 = -sin(y);

  r32 = cos(y)*sin(x);

  r33 = cos(x)*cos(y);

  t1 = position[0];
  t2 = position[1];
  t3 = position[2];


  rotationTranslationMatrix << 
    r11, r12, r13, t1,
    r21, r22, r23, t2,
    r31, r32, r33, t3,
    0,   0,   0,   1;
}

void WPManipulatorControl::getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
 float *angles)
{
  double r11, r21, r31, r32, r33, r22, r23;
  double roll, pitch, yaw;

  r11 = rotationTranslationMatrix(0,0);
  r21 = rotationTranslationMatrix(1,0);
  r22 = rotationTranslationMatrix(1,1);
  r23 = rotationTranslationMatrix(1,2);
  r31 = rotationTranslationMatrix(2,0);
  r31 = rotationTranslationMatrix(2,0);
  r32 = rotationTranslationMatrix(2,1);
  r33 = rotationTranslationMatrix(2,2);

  if (fabs(fabs(r31) - 1.0) < 0.01) {
  	if (r31 > 0) r31 = 1;
  	else if (r31 < 0) r31 = -1;
  }

  if (r31 < 1) {
  	if (r31 > -1) {
  		// Solution with positive sign. It limits the range of the values
        // of theta_y to (-pi/2, pi/2):
  		yaw = atan2(r21, r11);
  		pitch = asin(-r31);
  		roll = atan2(r32, r33);
  	}
  	else {
  		// roll and yaw are linked --> Gimbal lock:
        // There are infinity number of solutions for roll - yaw = atan2(-r23, r22).
        // To find a solution, set roll = 0 by convention.
        yaw = -atan2(-r23, r22);
        pitch = M_PI / 2.0;
        roll = 0.0;
  	}
  }
  else {
  	    // Gimbal lock: There is not a unique solution for
        // roll + yaw = atan2(-r23, r22), by convention, set roll = 0.
  		yaw = atan2(-r23, r22);
  		pitch = - M_PI / 2.0;
  		roll = 0.0;
  }

  angles[0] = roll;
  angles[1] = pitch;
  angles[2] = yaw;
}

int WPManipulatorControl::joint12_criterion_function(float *q1_in, float *q2_in, float q1_old, float q2_old, float *q_out, int nbr_of_solutions)
{
	float min_distance = -1, temp;
	int there_is_solution = 0;

	for (int i = 0; i < nbr_of_solutions; i++)
	{
		if (!std::isnan(q1_in[i]) && !std::isnan(q2_in[i]))
		{
			float distance = 0.0;

			temp = fabs(atan2(sin(q1_in[i] - q1_old), cos(q1_in[i] - q1_old)));
			distance += temp;
			temp = fabs(atan2(sin(q2_in[i] - q2_old), cos(q2_in[i] - q2_old)));
			distance += temp;

			there_is_solution = 1;

			if (min_distance < 0.0)
			{
				min_distance = distance;
				q_out[0] = q1_in[i];
				q_out[1] = q2_in[i];
			}
			else if (distance < min_distance)
			{
				min_distance = distance;
				q_out[0] = q1_in[i];
				q_out[1] = q2_in[i];
			}

		}

	}

	return there_is_solution;
}

float WPManipulatorControl::angle_criterion_function(float *angle, float q3_meas, float q4_meas, float q5_meas)
{
	float angle_meas, middle, return_angle, dist;

	angle_meas = q3_meas + q4_meas + q5_meas;
	angle_meas = atan2(sin(angle_meas), cos(angle_meas));

	if (angle_meas > angle[0] && angle_meas < angle[1]) return_angle = angle_meas;
	else if (angle_meas > angle[2] && angle_meas < angle[3]) return_angle = angle_meas;

	dist = abs(angle_meas-angle[0]);
	return_angle = angle[0];

	if (abs(angle_meas-angle[1]) < dist) return_angle = angle[1];
	else if (abs(angle_meas-angle[2]) < dist) return_angle = angle[2];
	else if (abs(angle_meas-angle[3]) < dist) return_angle = angle[3];

	return return_angle;
}

int WPManipulatorControl::joint26_criterion_function(float *q3_in, float *q4_in, float *q5_in, float q3_old, float q4_old, float q5_old, float *q_out, int nbr_of_solutions)
{
	float min_distance = -1, temp;
	int there_is_solution = 0;

	for (int i = 0; i < nbr_of_solutions; i++)
	{
		if (!std::isnan(q3_in[i]) && !std::isnan(q4_in[i]) && !std::isnan(q5_in[i]))
		{
			float distance = 0.0;

			temp = fabs(atan2(sin(q3_in[i] - q3_old), cos(q3_in[i] - q3_old)));
			distance += temp;
			temp = fabs(atan2(sin(q4_in[i] - q4_old), cos(q4_in[i] - q4_old)));
			distance += temp;
			temp = fabs(atan2(sin(q5_in[i] - q5_old), cos(q5_in[i] - q5_old)));
			distance += temp;

			there_is_solution = 1;

			if (min_distance < 0.0)
			{
				min_distance = distance;
				q_out[0] = q3_in[i];
				q_out[1] = q4_in[i];
				q_out[2] = q5_in[i];
			}
			else if (distance < min_distance)
			{
				min_distance = distance;
				q_out[0] = q3_in[i];
				q_out[1] = q4_in[i];
				q_out[2] = q5_in[i];
			}
		}
	}

	return there_is_solution;
}

int WPManipulatorControl::joint_criterion_function(float *q1_in, float *q2_in, float *q3_in, float *q4_in, float *q5_in, 
	float q1_old, float q2_old, float q3_old, float q4_old, float q5_old, float *q_out, int nbr_of_solutions)
{
	float min_distance = -1, temp;
	int there_is_solution = 0;
	int temp2;

	for (int i = 0; i < nbr_of_solutions; i++) {
		if (!std::isnan(q1_in[i]) && !std::isnan(q2_in[i]) && !std::isnan(q3_in[i]) && !std::isnan(q4_in[i]) && !std::isnan(q5_in[i])) {
			float distance = 0.0;

			temp = fabs(atan2(sin(q1_in[i] - q1_old), cos(q1_in[i] - q1_old)));
			distance += temp;
			temp = fabs(atan2(sin(q2_in[i] - q2_old), cos(q2_in[i] - q2_old)));
			distance += temp;
			temp = fabs(atan2(sin(q3_in[i] - q3_old), cos(q3_in[i] - q3_old)));
			distance += temp;
			temp = fabs(atan2(sin(q4_in[i] - q4_old), cos(q4_in[i] - q4_old)));
			distance += temp;
			temp = fabs(atan2(sin(q5_in[i] - q5_old), cos(q5_in[i] - q5_old)));
			distance += temp;

			there_is_solution = 1;

			if (min_distance < 0.0) {
				min_distance = distance;
				q_out[0] = q1_in[i];
				q_out[1] = q2_in[i];
				q_out[2] = q3_in[i];
				q_out[3] = q4_in[i];
				q_out[4] = q5_in[i];
			}
			else if (distance < min_distance) {
				min_distance = distance;
				q_out[0] = q1_in[i];
				q_out[1] = q2_in[i];
				q_out[2] = q3_in[i];
				q_out[3] = q4_in[i];
				q_out[4] = q5_in[i];
			}
		} 
	}

	return there_is_solution;
}

void WPManipulatorControl::limitJointsPosition(float *q) {

	for (int i=0; i<5; i++) 
	{
		if (q[i] > arm_upper_limits_[i]) q[i] = arm_upper_limits_[i];
		else if (q[i] < arm_lower_limits_[i]) q[i] = arm_lower_limits_[i];
	}
}

void WPManipulatorControl::set_rate(int rate)
{
	rate_ = rate;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "WP_manipulator_control_node");
	ros::NodeHandle private_node_handle_("~");
	ros::NodeHandle n;
	int rate;

	std::string path = ros::package::getPath("aerial_manipulators");
	std::string dh_parameters_file;

	WPManipulatorControl wpm_control;

	private_node_handle_.param("dh_parameters_file", dh_parameters_file, std::string("/cfg/wp_manipulator_dh_parameters.yaml"));
	private_node_handle_.param("rate", rate, int(30));

	wpm_control.LoadParameters(path+dh_parameters_file);
	wpm_control.set_rate(rate);
	wpm_control.start();

	return 0;
}