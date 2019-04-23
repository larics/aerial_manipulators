#include <aerial_manipulators_control/WPManipulatorControl.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "WP_manipulator_control_node");

	ros::NodeHandle private_node_handle_("~");
	ros::NodeHandle n;
	
	int rate;
	std::string robot_model_name, joint_group_name;
	std::string parameters_file;

	std::string path = ros::package::getPath("aerial_manipulators_control");

	private_node_handle_.param("rate", rate, int(30));
	private_node_handle_.param("robot_name", robot_model_name, std::string("wp_manipulator"));
	private_node_handle_.param("joint_group_name", joint_group_name, std::string("wp_manipulator_arm"));
	private_node_handle_.param("parameters_file", parameters_file, std::string("/config/wp_manipulator_dh_parameters.yaml"));

	ros::Publisher manipulator_position_pub_ros_ = n.advertise<geometry_msgs::PoseStamped>("end_effector/pose", 1);

	ManipulatorControl wp_control;

	ros::Rate loop_rate(rate);

	wp_control.setManipulatorName(robot_model_name, joint_group_name);
	wp_control.LoadParameters(path+parameters_file);
	wp_control.init(&n);

	while(ros::ok() && !wp_control.isStarted())
	{
		ros::spinOnce();
		printf("Waiting for measurements.\n");
		ros::Duration(0.5).sleep();
	}

	while(ros::ok())
	{
		ros::spinOnce();

		manipulator_position_pub_ros_.publish(wp_control.getEndEffectorPosition());
	
		geometry_msgs::Pose posa;


	    posa.position.x = 0;
	    posa.position.y = 0.35;
	    posa.position.z = 0;

	    posa.orientation.x = -0.5;
	    posa.orientation.y = -0.5;
	    posa.orientation.z = 0.5;
	    posa.orientation.w = 0.5;

		//wp_control.publishJointSetpoints(wp_control.getJointSetpoints());
		wp_control.publishJointSetpoints(wp_control.calculateJointSetpoints(posa));

		loop_rate.sleep();
	}

	return 0;
}