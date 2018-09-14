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
	manipulator_mode_(0)

{
	rate_ = 30;

	new_dynamixel_measurement_ = false;

	joint_state_sub_ros_ = n_.subscribe("dynamixel_state", 1, &WPManipulatorControl::joint_controller_state_cb_ros, this);
	mmuav_position_sub_ros_ = n_.subscribe("pose", 1, &WPManipulatorControl::mmuav_position_cb_ros, this);
	wp_manipulator_end_effector_position_sub_ros_ = n_.subscribe("wp_manipulator/position_set_point", 1, &WPManipulatorControl::wp_manipulator_end_effector_position_cb_ros, this);
	mode_sub_ros_ = n_.subscribe("mode", 1, &WPManipulatorControl::mode_cb_ros, this);

	manipulator_wrench_ros_pub_ = n_.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
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
	manipulator_mode_ = msg.data;
}

double WPManipulatorControl::deadzone(double value, double lower_limit, double upper_limit)
{
	if (value > upper_limit) return (value-upper_limit);
	else if (value < lower_limit) return (value-lower_limit);
	else return 0;
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
	arm_lower_limits = config["limits"]["upper"].as<std::vector<double> >();


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

	Eigen::Matrix4d T16_dk, T16_ref;
	Eigen::MatrixXd J, Tau(6,1);
	int nbr_of_solutions;

	float orientationEuler[3], orientationQuaternion[4], Q[5];

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

		//inverse kinematics
		T16_ref = Tuav_origin_arm_inv_ * Tuav_origin_world_ * Tworld_end_effector_dk;/*Tworld_wp_end_effector_ref_;*/
		getAnglesFromRotationTranslationMatrix(T16_ref, orientationEuler);
		//std::cout<<"Roll: "<<orientationEuler[0]<<"Pitch: "<<orientationEuler[1]<<" Yaw: "<<orientationEuler[2]<<std::endl;
		nbr_of_solutions = manipulator_inverse.ik_calculate(T16_ref(0,3), T16_ref(1,3), T16_ref(2,3), orientationEuler[1], orientationEuler[2]);

		/*for (int i=0; i <nbr_of_solutions; i ++) {
			std::cout<<"Q1: "<<manipulator_inverse.getQ1()[i]<<std::endl;
			std::cout<<"Q2: "<<manipulator_inverse.getQ2()[i]<<std::endl;
			std::cout<<"Q3: "<<manipulator_inverse.getQ3()[i]<<std::endl;
			std::cout<<"Q4: "<<manipulator_inverse.getQ4()[i]<<std::endl;
			std::cout<<"Q5: "<<manipulator_inverse.getQ5()[i]<<std::endl;
			std::cout<<std::endl;
		}*/

		if (joint_criterion_function(manipulator_inverse.getQ1(), manipulator_inverse.getQ2(), manipulator_inverse.getQ3(), 
			manipulator_inverse.getQ4(), manipulator_inverse.getQ5(), q1_pos_meas_, q2_pos_meas_, q3_pos_meas_, q4_pos_meas_, q5_pos_meas_, Q, nbr_of_solutions)) {

			limitJointsPosition(Q);

			/*std::cout<<"Q1: "<<Q[0]<<std::endl;
			std::cout<<"Q2: "<<Q[1]<<std::endl;
			std::cout<<"Q3: "<<Q[2]<<std::endl;
			std::cout<<"Q4: "<<Q[3]<<std::endl;
			std::cout<<"Q5: "<<Q[4]<<std::endl;
			std::cout<<std::endl;*/
		}
		//std::cout<<std::endl;


		if (manipulator_mode_ == 0) //dk mode
		{

		}
		else if (manipulator_mode_ == 1) //ik mode
		{

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
	private_node_handle_.param("rate", rate, int(200));

	wpm_control.LoadParameters(path+dh_parameters_file);
	wpm_control.set_rate(rate);
	wpm_control.start();

	return 0;
}