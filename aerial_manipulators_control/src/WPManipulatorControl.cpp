#include <aerial_manipulators_control/WPManipulatorControl.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "WP_manipulator_control_node");

	ros::NodeHandle private_node_handle_("~");
	ros::NodeHandle n;
	
	int rate;
	std::string robot_model_name, joint_group_name;

	private_node_handle_.param("rate", rate, int(30));
	private_node_handle_.param("robot_name", robot_model_name, std::string("wp_manipulator"));
	private_node_handle_.param("joint_group_name", joint_group_name, std::string("wp_manipulator_arm"));

	ManipulatorControl wp_control;

	ros::Rate loop_rate(rate);

	wp_control.setManipulatorName(robot_model_name, joint_group_name);
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

		wp_control.getEndEffectorPosition();

		loop_rate.sleep();
	}

	//wpm_control.LoadParameters(path+dh_parameters_file);
	//wpm_control.set_rate(rate);
	//wpm_control.start();

	return 0;
}