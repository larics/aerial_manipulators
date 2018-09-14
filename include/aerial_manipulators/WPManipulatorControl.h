#ifndef WPMANIPULATORCONTROL_H
#define WPMANIPULATORCONTROL_H

#include <aerial_manipulators/WPManipulatorDirectKinematics.h>
#include <aerial_manipulators/WPManipulatorInverseKinematics.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <geometry_msgs/PoseStamped.h>
#include <aerial_manipulators/KalmanFilter.h>
#include <aerial_manipulators/median_filter.h>
#include <std_srvs/Empty.h>
#include "ros/ros.h"
#include <ros/package.h>

class WPManipulatorControl
{
	public:
		WPManipulatorControl();
		void start();
		void set_rate(int rate);
		void LoadParameters(std::string file);

	private:
		void quaternion2euler(float *quaternion, float *euler);
		void getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix, 
			float *orientationEuler, float *position);
		void getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
			float *angles);
		double deadzone(double value, double lower_limit, double upper_limit);
		void wp_manipulator_end_effector_position_cb_ros(const geometry_msgs::PoseStamped &msg);
		void euler2quaternion(float *euler, float *quaternion);
		void mmuav_position_cb_ros(const geometry_msgs::PoseStamped &msg);
		int joint_criterion_function(float *q1_in, float *q2_in, float *q3_in, float *q4_in, float *q5_in, 
			float q1_old, float q2_old, float q3_old, float q4_old, float q5_old, float *q_out, int nbr_of_solutions);
		void joint_controller_state_cb_ros(const dynamixel_workbench_msgs::DynamixelStateList &msg);
		bool wrench_zero_all_cb(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
		void limitJointsPosition(float *q);

		
		WPManipulatorDirectKinematics manipulator_direct;
		WPManipulatorInverseKinematics manipulator_inverse;

		Eigen::Matrix4d Tuav_origin_arm_inv_, Tuav_origin_arm_, Tworld_end_effector_dk;
		Eigen::Matrix4d Tworld_uav_origin_, Tuav_origin_world_, Tworld_wp_end_effector_ref_;
		Eigen::MatrixXd wrench_;

		int rate_;
		float q1_pos_meas_, q2_pos_meas_, q3_pos_meas_, q4_pos_meas_, q5_pos_meas_;
		float q1_torque_meas_, q2_torque_meas_, q3_torque_meas_;
		float q4_torque_meas_, q5_torque_meas_;

		bool new_dynamixel_measurement_, start_flag_;

		ros::Subscriber joint_state_sub_ros_, mmuav_position_sub_ros_;
		ros::Subscriber wp_manipulator_end_effector_position_sub_ros_;
		ros::Publisher manipulator_position_pub_ros_, manipulator_wrench_ros_pub_;

		ros::ServiceServer wrench_zero_all_srv_ros_;

		KalmanFilter q1_torque_kalman_, q2_torque_kalman_, q3_torque_kalman_;
		KalmanFilter q4_torque_kalman_, q5_torque_kalman_; 

		median_filter q1_torque_median_, q2_torque_median_, q3_torque_median_;
		median_filter q4_torque_median_, q5_torque_median_;

		ros::NodeHandle n_;

};

#endif