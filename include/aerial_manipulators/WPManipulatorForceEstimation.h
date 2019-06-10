#ifndef WPMANIPULATORFORCEESTIMATION_H
#define WPMANIPULATORFORCEESTIMATION_H

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <array>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <aerial_manipulators/WPManipulatorInverseDynamics.h>
#include <aerial_manipulators/float5.h>
#include <aerial_manipulators/float6.h>
#include <geometry_msgs/WrenchStamped.h>




class WPManipulatorForceEstimation {
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Subscriber sub2_;
	ros::Publisher pub1_;
	ros::Publisher pub2_;
	ros::Publisher pub3_;
	ros::Publisher pub4_;

	sensor_msgs::JointState js_;
	aerial_manipulators::float6 force_;

	float q_pos_[6];
	int rate_;
	float tauget_[6];
	float tauki_[5];
	float f_sensor_[3];

	float massh_[6] = { 0.0, 0.01715, 0.10663, 0.17449, 0.01133, 0.08200 };

	float dch_[6][3] = { {0 / 1000, 0 / 1000, 0 / 1000}, {-61.24 / 1000, 0.0 / 1000, -0.09 / 1000}, {-110.80 / 1000, -0.63 / 1000, -0.59 / 1000}, {-37.76 / 1000, 0.0 / 1000, 0.18 / 1000}, {-36.24 / 1000, 0.0 / 1000, -0.59 / 1000}, {-0.21 / 1000, -0.83 / 1000, -33.46 / 1000} };

	float D_h_[6][3][3] = { {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
	{ {8656.5 / 1000000000, 0.18 / 1000000000, 96.42 / 1000000000}, {0.18 / 1000000000, 100511.76 / 1000000000, 0.00 / 1000000000}, {96.42 / 1000000000, 0.00 / 1000000000, 94741.01 / 1000000000} },
	{ {25494.33 / 1000000000, 8639.22 / 1000000000, 6994.05 / 1000000000}, {8639.22 / 1000000000, 1441278.11 / 1000000000, 40.06 / 1000000000}, {6994.05 / 1000000000, 40.06 / 1000000000, 1443420.56 / 1000000000} },
	{ {30962.85 / 1000000000, 0.59 / 1000000000, -1210.86 / 1000000000}, {0.59 / 1000000000, 416023.24 / 1000000000, 0.01 / 1000000000}, {-1210.86 / 1000000000, 0.01 / 1000000000, 408445.34 / 1000000000} },
	{ {5406.00 / 1000000000, 0.07 / 1000000000, 243.11 / 1000000000}, {0.07 / 1000000000, 27148.80 / 1000000000, 0.00 / 1000000000}, {243.11 / 1000000000, 0.00 / 1000000000, 23472.02 / 1000000000} },
	{ {114783.85 / 1000000000, 14.13 / 1000000000, 571.94 / 1000000000}, {14.13 / 1000000000, 112108.36 / 1000000000, 2462.51 / 1000000000}, {571.94 / 1000000000, 2462.51 / 1000000000, 13291.56 / 1000000000} } };



public:
	WPManipulatorForceEstimation(void);
	~WPManipulatorForceEstimation(void);

	Eigen::Matrix<float, 6, 1> forceEstimate(float *q, float *v0, float *w0, float dtime);

	Eigen::Matrix<float, 6, 5> jacobian(float *q);

	void joint_states_Callback(const sensor_msgs::JointState& msg);
	void ft_sensor_Callback(const geometry_msgs::WrenchStamped& msg);

	void run(void);

};



#endif
