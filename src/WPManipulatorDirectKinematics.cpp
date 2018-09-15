#include <aerial_manipulators/WPManipulatorDirectKinematics.h>


WPManipulatorDirectKinematics::WPManipulatorDirectKinematics(void)
{
	isInit = false;
}

Eigen::Matrix4d WPManipulatorDirectKinematics::dk_calculate(float q1, float q2, float q3, float q4, float q5)
{
	if (isInit)
	{
		float theta[6], alpha[6], d[6], a[6];
		Eigen::Matrix4d T16;

		theta[0] = dhParams_.theta[0] + q1;
		theta[1] = dhParams_.theta[1] + q2;
		theta[2] = dhParams_.theta[2] + q3;
		theta[3] = dhParams_.theta[3] + q4;
		theta[4] = dhParams_.theta[4] + q5;

		alpha[0] = dhParams_.alpha[0];
		alpha[1] = dhParams_.alpha[1];
		alpha[2] = dhParams_.alpha[2];
		alpha[3] = dhParams_.alpha[3];
		alpha[4] = dhParams_.alpha[4];
		alpha[5] = dhParams_.alpha[5];

		d[0] = dhParams_.d[0];
		d[1] = dhParams_.d[1];
		d[2] = dhParams_.d[2];
		d[3] = dhParams_.d[3];
		d[4] = dhParams_.d[4];
		d[5] = dhParams_.d[5];

		a[0] = dhParams_.a[0];
		a[1] = dhParams_.a[1];
		a[2] = dhParams_.a[2];
		a[3] = dhParams_.a[3];
		a[4] = dhParams_.a[4];
		a[5] = dhParams_.a[5];

		T16 << cos(theta[2] + theta[3] + theta[4])*cos(theta[0] + theta[1]),  sin(theta[0] + theta[1]), -sin(theta[2] + theta[3] + theta[4])*cos(theta[0] + theta[1]), a[1]*cos(theta[0] + theta[1]) - d[5]*(cos(theta[0] + theta[1])*cos(theta[2] + theta[3])*sin(theta[4]) + cos(theta[0] + theta[1])*sin(theta[2] + theta[3])*cos(theta[4])) + a[0]*cos(theta[0]) + a[3]*cos(theta[0] + theta[1])*cos(theta[2] + theta[3]) + a[2]*cos(theta[0] + theta[1])*cos(theta[2]),
	   		   cos(theta[2] + theta[3] + theta[4])*sin(theta[0] + theta[1]), -cos(theta[0] + theta[1]), -sin(theta[2] + theta[3] + theta[4])*sin(theta[0] + theta[1]), a[1]*sin(theta[0] + theta[1]) - d[5]*(cos(theta[2] + theta[3])*sin(theta[0] + theta[1])*sin(theta[4]) + sin(theta[0] + theta[1])*sin(theta[2] + theta[3])*cos(theta[4])) + a[0]*sin(theta[0]) + a[3]*cos(theta[2] + theta[3])*sin(theta[0] + theta[1]) + a[2]*sin(theta[0] + theta[1])*cos(theta[2]),
	          -sin(theta[2] + theta[3] + theta[4]),                           0,                        -cos(theta[2] + theta[3] + theta[4]),                         -a[3]*sin(theta[2] + theta[3]) - a[2]*sin(theta[2]) - d[5]*cos(theta[2] + theta[3] + theta[4]),
               0,                                                             0,                         0,                                                            1;
 
		return T16;
	}

}

Eigen::Matrix4d WPManipulatorDirectKinematics::dk_T12_calculate(float q1, float q2) 
{
	if (isInit)
	{
		float theta[2], alpha[2], d[2], a[2];
		Eigen::Matrix4d T12;

		theta[0] = dhParams_.theta[0] + q1;
		theta[1] = dhParams_.theta[1] + q2;

		alpha[0] = dhParams_.alpha[0];
		alpha[1] = dhParams_.alpha[1];

		d[0] = dhParams_.d[0];
		d[1] = dhParams_.d[1];

		a[0] = dhParams_.a[0];
		a[1] = dhParams_.a[1];

		T12 << cos(theta[0] + theta[1]), 0, -sin(theta[0] + theta[1]), a[1]*cos(theta[0] + theta[1]) + a[0]*cos(theta[0]),
			   sin(theta[0] + theta[1]), 0,  cos(theta[0] + theta[1]), a[1]*sin(theta[0] + theta[1]) + a[0]*sin(theta[0]),
			   0,                       -1,  0,                        0,
			   0,                        0,  0,                        1;

		return T12;                    
	}
}

Eigen::Matrix4d WPManipulatorDirectKinematics::dk_T26_calculate(float q3, float q4, float q5) 
{
	if (isInit)
	{
		float theta[4], alpha[4], d[4], a[4];
		Eigen::Matrix4d T26;

		theta[0] = dhParams_.theta[2] + q3;
		theta[1] = dhParams_.theta[3] + q4;
		theta[2] = dhParams_.theta[4] + q5;

		alpha[0] = dhParams_.alpha[2];
		alpha[1] = dhParams_.alpha[3];
		alpha[2] = dhParams_.alpha[4];
		alpha[3] = dhParams_.alpha[5];

		d[0] = dhParams_.d[2];
		d[1] = dhParams_.d[3];
		d[2] = dhParams_.d[4];
		d[3] = dhParams_.d[5];

		a[0] = dhParams_.a[2];
		a[1] = dhParams_.a[3];
		a[2] = dhParams_.a[4];
		a[3] = dhParams_.a[5];


		T26 << cos(theta[0] + theta[1] + theta[2]), 0, -sin(theta[0] + theta[1] + theta[2]), a[1]*cos(theta[0] + theta[1]) + a[0]*cos(theta[0]) - d[3]*sin(theta[0] + theta[1] + theta[2]),
			   sin(theta[0] + theta[1] + theta[2]), 0,  cos(theta[0] + theta[1] + theta[2]), a[1]*sin(theta[0] + theta[1]) + a[0]*sin(theta[0]) + d[3]*cos(theta[0] + theta[1] + theta[2]),
			   0,                                  -1,  0,                        0,
			   0,                                   0,  0,                        1;

		return T26;                    
	}
}

void WPManipulatorDirectKinematics::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);
	std::vector<double> theta, alpha, d, a;

	theta = config["theta"].as<std::vector<double> >();
	alpha = config["alpha"].as<std::vector<double> >();
	d = config["d"].as<std::vector<double> >();
	a = config["a"].as<std::vector<double> >();

	for (int i=0; i<6; i++)
	{
		dhParams_.theta[i] = theta[i];
		dhParams_.alpha[i] = alpha[i];
		dhParams_.d[i] = d[i];
		dhParams_.a[i] = a[i];
	}

	isInit = true;
}