#include <aerial_manipulators/WPManipulatorInverseKinematics.h>

WPManipulatorInverseKinematics::WPManipulatorInverseKinematics(void)
{
	isInit = false;

	for (int i = 0; i < 8; i++) 
	{
		q1_[i] = 0;
		q2_[i] = 0;
		q3_[i] = 0;
		q4_[i] = 0;
		q5_[i] = 0;
	}
}

void WPManipulatorInverseKinematics::LoadParameters(std::string file)
{
	YAML::Node config = YAML::LoadFile(file);
	std::vector<double> theta, a, alpha, d;

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

void WPManipulatorInverseKinematics::setDHparams(WPManipulatorDirectKinematics::DH_Parameters_TypeDef dhParams)
{
	dhParams_ = dhParams;
	isInit = true;
}

int WPManipulatorInverseKinematics::getMaxNumberOfSolutions(void) {
	return 32;
}

float *WPManipulatorInverseKinematics::getQ1(void) {
	return q1_;
}

float *WPManipulatorInverseKinematics::getQ2(void) {
	return q2_;
}

float *WPManipulatorInverseKinematics::getQ3(void) {
	return q3_;
}

float *WPManipulatorInverseKinematics::getQ4(void) {
	return q4_;
}

float *WPManipulatorInverseKinematics::getQ5(void) {
	return q5_;
}

Eigen::MatrixXd WPManipulatorInverseKinematics::getJacobian(double q1, double q2, double q3, double q4, double q5) {
	Eigen::MatrixXd J(6,6);//*J = new Eigen::MatrixXd(6,6);
	float theta[6], alpha[6], d[6], a[6];

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

	J <<  d[5]*(cos(theta[2] + theta[3])*sin(theta[0] + theta[1])*sin(theta[4]) + sin(theta[0] + theta[1])*sin(theta[2] + theta[3])*cos(theta[4])) - a[1]*sin(theta[0] + theta[1]) - a[0]*sin(theta[0]) - a[3]*cos(theta[2] + theta[3])*sin(theta[0] + theta[1]) - a[2]*sin(theta[0] + theta[1])*cos(theta[2]), -sin(theta[0] + theta[1])*(a[1] + a[3]*cos(theta[2] + theta[3]) + a[2]*cos(theta[2]) - d[5]*sin(theta[2] + theta[3] + theta[4])), -cos(theta[0] + theta[1])*(a[3]*sin(theta[2] + theta[3]) + a[2]*sin(theta[2]) + d[5]*cos(theta[2] + theta[3] + theta[4])), -cos(theta[0] + theta[1])*(a[3]*sin(theta[2] + theta[3]) + d[5]*cos(theta[2] + theta[3] + theta[4])), -d[5]*cos(theta[2] + theta[3] + theta[4])*cos(theta[0] + theta[1]),           0,
		   a[1]*cos(theta[0] + theta[1]) - d[5]*(cos(theta[0] + theta[1])*cos(theta[2] + theta[3])*sin(theta[4]) + cos(theta[0] + theta[1])*sin(theta[2] + theta[3])*cos(theta[4])) + a[0]*cos(theta[0]) + a[3]*cos(theta[0] + theta[1])*cos(theta[2] + theta[3]) + a[2]*cos(theta[0] + theta[1])*cos(theta[2]),  cos(theta[0] + theta[1])*(a[1] + a[3]*cos(theta[2] + theta[3]) + a[2]*cos(theta[2]) - d[5]*sin(theta[2] + theta[3] + theta[4])), -sin(theta[0] + theta[1])*(a[3]*sin(theta[2] + theta[3]) + a[2]*sin(theta[2]) + d[5]*cos(theta[2] + theta[3] + theta[4])), -sin(theta[0] + theta[1])*(a[3]*sin(theta[2] + theta[3]) + d[5]*cos(theta[2] + theta[3] + theta[4])), -d[5]*cos(theta[2] + theta[3] + theta[4])*sin(theta[0] + theta[1]),           0,
           0,                                                                                                                                                                                                                                                                                                     0,                                                                                                                                d[5]*sin(theta[2] + theta[3] + theta[4]) - a[2]*cos(theta[2]) - a[3]*cos(theta[2] + theta[3]),                             d[5]*sin(theta[2] + theta[3] + theta[4]) - a[3]*cos(theta[2] + theta[3]),                             d[5]*sin(theta[2] + theta[3] + theta[4]),                                    0,
           0,                                                                                                                                                                                                                                                                                                    -sin(theta[0] + theta[1]),                                                                                                        -sin(theta[0] + theta[1]),                                                                                                 -sin(theta[0] + theta[1]),                                                                            -sin(theta[2] + theta[3] + theta[4])*cos(theta[0] + theta[1]),               -sin(theta[2] + theta[3] + theta[4])*cos(theta[0] + theta[1]),
           0,                                                                                                                                                                                                                                                                                                     cos(theta[0] + theta[1]),                                                                                                         cos(theta[0] + theta[1]),                                                                                                  cos(theta[0] + theta[1]),                                                                            -sin(theta[2] + theta[3] + theta[4])*sin(theta[0] + theta[1]),               -sin(theta[2] + theta[3] + theta[4])*sin(theta[0] + theta[1]),
           1,                                                                                                                                                                                                                                                                                                     0,                                                                                                                                0,                                                                                                                         0,                                                                                                   -cos(theta[2] + theta[3] + theta[4]),                                        -cos(theta[2] + theta[3] + theta[4]);

    return J;
}

int WPManipulatorInverseKinematics::ik_T12_calculate(float y, float rot_z) {
	int number_of_solutions = 0;
	float w6, w2;

	if (isInit)
	{
		w2 = y;
		w6 = rot_z;

		q1_[0] = asin((w2 - dhParams_.a[1] * sin(w6)) / dhParams_.a[0]);
		q1_[1] = M_PI - asin((w2 - dhParams_.a[1] * sin(w6)) / dhParams_.a[0]);

		q2_[0] = w6 - q1_[0];
		q2_[1] = w6 - q1_[1];

		q1_[0] = atan2(sin(q1_[0] - dhParams_.theta[0]), cos(q1_[0] - dhParams_.theta[0]));
		q2_[0] = atan2(sin(q2_[0] - dhParams_.theta[1]), cos(q2_[0] - dhParams_.theta[1]));
		q1_[1] = atan2(sin(q1_[1] - dhParams_.theta[0]), cos(q1_[1] - dhParams_.theta[0]));
		q2_[1] = atan2(sin(q2_[1] - dhParams_.theta[1]), cos(q2_[1] - dhParams_.theta[1]));

		number_of_solutions = 2;
	}

	return number_of_solutions;
}

int WPManipulatorInverseKinematics::getFeasibleRotationT26(float x, float y, float *rot_z) {
	float temp1, temp2, z, det, s345[2];
	int solution;

	temp1 = dhParams_.a[3]*dhParams_.a[3] + dhParams_.a[2]*dhParams_.a[2] + 2*dhParams_.a[3]*dhParams_.a[2] - dhParams_.d[5]*dhParams_.d[5] - x*x - y*y;
	temp2 = 2*dhParams_.d[5];

	z = temp1/temp2;

	det = 4*x*x*z*z - 4*(x*x + y*y)*(z*z - y*y);

	if (det < 0) solution = 0;
	else if (det == 0) {
		solution = 1;
		s345[0] = (x*z) / ((x*x + y*y));
		s345[1] = (x*z) / ((x*x + y*y));

	}
	else {
		temp1 = (2.0*x*z + sqrt(det)) / (2.0*(x*x + y*y));
		temp2 = (2.0*x*z - sqrt(det)) / (2.0*(x*x + y*y));

		if (abs(temp1) > 1 || abs(temp2) > 1) solution = 0;
		else solution = 1;
		
		if (temp1 < temp2)
		{
			s345[0] = temp1;
			s345[1] = temp2;
		}
		else
		{
			s345[0] = temp2;
			s345[1] = temp1;
		}
	}

	rot_z[0] = asin(s345[0]);
	rot_z[1] = asin(s345[1]);

	rot_z[2] = M_PI - asin(s345[0]);
	rot_z[3] = M_PI - asin(s345[1]);

	return solution;

}

int WPManipulatorInverseKinematics::ik_T26_calculate(float x, float y, float rot_z) 
{
	int number_of_solutions = 0;
	float w6, w1, w2, t1, t2, temp1, temp2;
	bool isS4Zero;

	if (isInit)
	{
		w1 = x;
		w2 = y;
		w6 = rot_z;

		t1 = w1 + dhParams_.d[5]*sin(w6);
		t2 = w2 - dhParams_.d[5]*cos(w6);

		//q4
		temp1 = t1*t1 + t2*t2 - dhParams_.a[3]*dhParams_.a[3] - dhParams_.a[2]*dhParams_.a[2];
		temp2 = 2*dhParams_.a[3]*dhParams_.a[2];

		if (fabs(temp1 - temp2) < 0.001) temp1 = temp2;

		q4_[0] = acos(temp1/temp2);
		q4_[1] = acos(temp1/temp2);

		q4_[2] = -acos(temp1/temp2);
		q4_[3] = -acos(temp1/temp2);

		//q3
		if (fabs(sin(q4_[0])) < 0.001) {
			isS4Zero = true;
		}
		else {
			isS4Zero = false;
		}

		if (!isS4Zero) {
			temp1 = dhParams_.a[3]*sin(q4_[0])*t2 + dhParams_.a[3]*cos(q4_[0])*t1 + dhParams_.a[2]*t1;
			temp2 = dhParams_.a[3]*dhParams_.a[3] + 2*dhParams_.a[3]*dhParams_.a[2]*cos(q4_[0]) + dhParams_.a[2]*dhParams_.a[2];
			if (fabs(temp1 - temp2) < 0.001) temp1 = temp2;

			q3_[0] = acos(temp1/temp2);
			q3_[1] = -acos(temp1/temp2);
		}
		else {
			temp1 = dhParams_.a[3]*cos(q4_[0])*t2 + dhParams_.a[2]*t2 - dhParams_.a[3]*sin(q4_[0])*t1;
			temp2 = dhParams_.a[3]*dhParams_.a[3] + 2*dhParams_.a[3]*dhParams_.a[2]*cos(q4_[0]) + dhParams_.a[2]*dhParams_.a[2];
			if (fabs(temp1 - temp2) < 0.001) temp1 = temp2;

			q3_[0] = asin(temp1/temp2);
			q3_[1] = M_PI - asin(temp1/temp2);
		}

		if (fabs(sin(q4_[2])) < 0.001) {
			isS4Zero = true;
		}
		else {
			isS4Zero = false;
		}

		if (!isS4Zero) {
			temp1 = dhParams_.a[3]*sin(q4_[2])*t2 + dhParams_.a[3]*cos(q4_[2])*t1 + dhParams_.a[2]*t1;
			temp2 = dhParams_.a[3]*dhParams_.a[3] + 2*dhParams_.a[3]*dhParams_.a[2]*cos(q4_[2]) + dhParams_.a[2]*dhParams_.a[2];
			if (fabs(temp1 - temp2) < 0.001) temp1 = temp2;

			q3_[2] = acos(temp1/temp2);
			q3_[3] = -acos(temp1/temp2);
		}
		else {
			temp1 = dhParams_.a[3]*cos(q4_[2])*t2 + dhParams_.a[2]*t2 - dhParams_.a[3]*sin(q4_[2])*t1;
			temp2 = dhParams_.a[3]*dhParams_.a[3] + 2*dhParams_.a[3]*dhParams_.a[2]*cos(q4_[2]) + dhParams_.a[2]*dhParams_.a[2];
			if (fabs(temp1 - temp2) < 0.001) temp1 = temp2;

			q3_[2] = asin(temp1/temp2);
			q3_[3] = M_PI - asin(temp1/temp2);
		}
		
		for (int i = 0; i < 4; i++) 
		{
			q5_[i] = w6 - q3_[i] - q4_[i];

			q3_[i] = atan2(sin(q3_[i] - dhParams_.theta[2]), cos(q3_[i] - dhParams_.theta[2]));
			q4_[i] = atan2(sin(q4_[i] - dhParams_.theta[3]), cos(q4_[i] - dhParams_.theta[3]));
			q5_[i] = atan2(sin(q5_[i] - dhParams_.theta[4]), cos(q5_[i] - dhParams_.theta[4]));
		}
		

		number_of_solutions = 4;
	}

	return number_of_solutions;
}

int WPManipulatorInverseKinematics::ik_calculate(float x, float y, float z, float rot_y, float rot_z)
{
	int number_of_solutions = 0;

	if (isInit)
	{
		float t1, t2[32], t3, t4, w1, w2, w3, w5[2], w6, c12, s12, q12[4];
		float temp1, temp2, temp3;
		bool isS12Zero, isC12Zero, isGimbalLock;

		w1 = x;
		w2 = y;
		w3 = z;
		w6 = rot_z;

		w5[0] = rot_y;
		w5[1] = M_PI -rot_y;

		for (int j = 0; j < 2; j++) {

			if (fabs(fabs(w5[j])-M_PI/2.0) < 0.01) isGimbalLock = true;
			else isGimbalLock = false;

			if (!isGimbalLock) {
				if (cos(w5[j]) < 0.0) {
					q12[j*2] = atan2(sin(w6 + M_PI), cos(w6 + M_PI));
					q12[j*2 + 1] = atan2(sin(w6 + M_PI), cos(w6 + M_PI));
				}
				else {
					q12[j*2] = atan2(sin(w6), cos(w6));
					q12[j*2 + 1] = atan2(sin(w6), cos(w6));
				}
			}
			else {
				if (w5[j] > 0.0) {
					q12[j*2] = atan2(sin(-M_PI - w6), cos(-M_PI - w6));
					q12[j*2 + 1] = atan2(sin(w6 + M_PI), cos(w6 + M_PI));
				} 
				else {
					q12[j*2] = atan2(sin(w6 - M_PI), cos(w6 - M_PI));
					q12[j*2 + 1] = atan2(sin(M_PI - w6), cos(M_PI - w6));
				}
			}

			for (int i = 0; i < 2; i++)
			{
				t1 = w3 + dhParams_.d[5]*cos(w5[j]);
				t3 = w1 - cos(q12[j*2 + i]) * (dhParams_.a[1] - dhParams_.d[5]*sin(w5[j]));
				t4 = w2 - sin(q12[j*2 + i]) * (dhParams_.a[1] - dhParams_.d[5]*sin(w5[j]));
				c12 = cos(q12[j*2 + i]);
				s12 = sin(q12[j*2 + i]);

				if (fabs(s12) < 0.001) {
					isS12Zero = true;
				}
				else {
					isS12Zero = false;
				}

				if (fabs(c12) < 0.001) {
					isC12Zero = true;
				}
				else {
					isC12Zero = false;
				}

				if (fabs(dhParams_.a[0]) > 0.0 && !isC12Zero && isS12Zero && fabs(t4) > 0.0) {
					temp1 = dhParams_.a[0] - sqrt((dhParams_.a[0] + t4) * (dhParams_.a[0] - t4));
					temp2 = t4;

					q1_[16*j+8*i + 0] = 2*atan2(temp1, temp2);
					q1_[16*j+8*i + 1] = q1_[16*j+8*i];
					q1_[16*j+8*i + 2] = q1_[16*j+8*i];
					q1_[16*j+8*i + 3] = q1_[16*j+8*i];

					temp1 = dhParams_.a[0] + sqrt((dhParams_.a[0] + t4) * (dhParams_.a[0] - t4));

					q1_[16*j+8*i + 4] = 2*atan2(temp1, temp2);
					q1_[16*j+8*i + 5] = q1_[16*j+8*i + 4];
					q1_[16*j+8*i + 6] = q1_[16*j+8*i + 4];
					q1_[16*j+8*i + 7] = q1_[16*j+8*i + 4];

					temp1 = t3 - sqrt((dhParams_.a[0] + t4) * (dhParams_.a[0] - t4));
					temp2 = c12;
					t2[16*j+8*i + 0] = temp1 / temp2;
					t2[16*j+8*i + 1] = t2[16*j+8*i + 0];
					t2[16*j+8*i + 2] = t2[16*j+8*i + 0];
					t2[16*j+8*i + 3] = t2[16*j+8*i + 0];

					temp1 = t3 + sqrt((dhParams_.a[0] + t4) * (dhParams_.a[0] - t4));
					t2[16*j+8*i + 4] = temp1 / temp2;
					t2[16*j+8*i + 5] = t2[16*j+8*i + 4];
					t2[16*j+8*i + 6] = t2[16*j+8*i + 4];
					t2[16*j+8*i + 7] = t2[16*j+8*i + 4];

				}
				else if (fabs(dhParams_.a[0] + t3) > 0.0 && fabs(dhParams_.a[0]) > 0.0 && isC12Zero && !isS12Zero) {
					temp1 = sqrt((dhParams_.a[0] + t3) * (dhParams_.a[0] - t3));
					temp2 = dhParams_.a[0] + t3;

					q1_[16*j+8*i + 0] = -2*atan2(temp1, temp2);
					q1_[16*j+8*i + 1] = q1_[16*j+8*i + 0];
					q1_[16*j+8*i + 2] = q1_[16*j+8*i + 0];
					q1_[16*j+8*i + 3] = q1_[16*j+8*i + 0];

					q1_[16*j+8*i + 4] = 2*atan2(temp1, temp2);
					q1_[16*j+8*i + 5] = q1_[16*j+8*i + 4];
					q1_[16*j+8*i + 6] = q1_[16*j+8*i + 4];
					q1_[16*j+8*i + 7] = q1_[16*j+8*i + 4];

					temp1 = t4 + sqrt((dhParams_.a[0] + t3) * (dhParams_.a[0] - t3));
					temp2 = s12;

					t2[16*j+8*i + 0] = temp1 / temp2;
					t2[16*j+8*i + 1] = t2[16*j+8*i + 0];
					t2[16*j+8*i + 2] = t2[16*j+8*i + 0];
					t2[16*j+8*i + 3] = t2[16*j+8*i + 0];

					temp1 = t4 - sqrt((dhParams_.a[0] + t3) * (dhParams_.a[0] - t3));

					t2[16*j+8*i + 4] = temp1 / temp2;
					t2[16*j+8*i + 5] = t2[16*j+8*i + 4];
					t2[16*j+8*i + 6] = t2[16*j+8*i + 4];
					t2[16*j+8*i + 7] = t2[16*j+8*i + 4];
				}
				else if (fabs(dhParams_.a[0]) > 0.0 && !isC12Zero && !isS12Zero && dhParams_.a[0]*s12 + s12*t3 != c12*t4) {
					temp1 = dhParams_.a[0] * c12;
					temp2 = sqrt(dhParams_.a[0]*dhParams_.a[0] - c12*c12*t4*t4 + 2*c12*s12*t3*t4 - s12*s12*t3*t3);
					temp3 = dhParams_.a[0]*s12 - c12*t4 + s12*t3;

					q1_[16*j+8*i + 0] = -2*atan2(temp1 - temp2, temp3);
					q1_[16*j+8*i + 1] = q1_[16*j+8*i + 0];
					q1_[16*j+8*i + 2] = q1_[16*j+8*i + 0];
					q1_[16*j+8*i + 3] = q1_[16*j+8*i + 0];

					q1_[16*j+8*i + 4] = -2*atan2(temp1 + temp2, temp3);
					q1_[16*j+8*i + 5] = q1_[16*j+8*i + 4];
					q1_[16*j+8*i + 6] = q1_[16*j+8*i + 4];
					q1_[16*j+8*i + 7] = q1_[16*j+8*i + 4];

					temp1 = c12*t3 + s12*t4;

					t2[16*j+8*i + 0] = temp1 - temp2;
					t2[16*j+8*i + 1] = t2[16*j+8*i + 0];
					t2[16*j+8*i + 2] = t2[16*j+8*i + 0];
					t2[16*j+8*i + 3] = t2[16*j+8*i + 0];

					t2[16*j+8*i + 4] = temp1 + temp2;
					t2[16*j+8*i + 5] = t2[16*j+8*i + 4];
					t2[16*j+8*i + 6] = t2[16*j+8*i + 4];
					t2[16*j+8*i + 7] = t2[16*j+8*i + 4];
				}
				else return number_of_solutions;

				temp1 = t1*t1 + t2[16*j+8*i + 0]*t2[16*j+8*i + 0] - dhParams_.a[3]*dhParams_.a[3] - dhParams_.a[2]*dhParams_.a[2];
				temp2 = 2*dhParams_.a[3]*dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q4_[16*j+8*i + 0] =  acos(temp1 / temp2);
				q4_[16*j+8*i + 1] =  q4_[16*j+8*i + 0];

				q4_[16*j+8*i + 2] = -acos(temp1 / temp2);
				q4_[16*j+8*i + 3] =  q4_[16*j+8*i + 2];

				temp1 = t1*t1 + t2[16*j+8*i + 4]*t2[16*j+8*i + 4] - dhParams_.a[3]*dhParams_.a[3] - dhParams_.a[2]*dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q4_[16*j+8*i + 4] =  acos(temp1 / temp2);
				q4_[16*j+8*i + 5] =  q4_[16*j+8*i + 4];

				q4_[16*j+8*i + 6] = -acos(temp1 / temp2);
				q4_[16*j+8*i + 7] =  q4_[16*j+8*i + 6];

				temp1 = t1*(dhParams_.a[3]*cos(q4_[16*j+8*i + 0]) + dhParams_.a[2]) + dhParams_.a[3]*sin(q4_[16*j+8*i + 0])*t2[16*j+8*i + 0];
				temp2 = dhParams_.a[3] * dhParams_.a[3] + 2*dhParams_.a[3]*cos(q4_[16*j+8*i + 0])*dhParams_.a[2] + dhParams_.a[2] * dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[16*j+8*i + 0] = asin(-temp1/temp2);
				q3_[16*j+8*i + 1] = M_PI - asin(-temp1/temp2);

				temp1 = t1*(dhParams_.a[3]*cos(q4_[16*j+8*i + 2]) + dhParams_.a[2]) + dhParams_.a[3]*sin(q4_[16*j+8*i + 2])*t2[8*i + 2];
				temp2 = dhParams_.a[3] * dhParams_.a[3] + 2*dhParams_.a[3]*cos(q4_[16*j+8*i + 2])*dhParams_.a[2] + dhParams_.a[2] * dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[16*j+8*i + 2] = asin(-temp1/temp2);
				q3_[16*j+8*i + 3] = M_PI - asin(-temp1/temp2);

				temp1 = t1*(dhParams_.a[3]*cos(q4_[16*j+8*i + 4]) + dhParams_.a[2]) + dhParams_.a[3]*sin(q4_[16*j+8*i + 4])*t2[16*j+8*i + 4];
				temp2 = dhParams_.a[3] * dhParams_.a[3] + 2*dhParams_.a[3]*cos(q4_[16*j+8*i + 4])*dhParams_.a[2] + dhParams_.a[2] * dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[16*j+8*i + 4] = asin(-temp1/temp2);
				q3_[16*j+8*i + 5] = M_PI - asin(-temp1/temp2);

				temp1 = t1*(dhParams_.a[3]*cos(q4_[16*j+8*i + 6]) + dhParams_.a[2]) + dhParams_.a[3]*sin(q4_[16*j+8*i + 6])*t2[16*j+8*i + 6];
				temp2 = dhParams_.a[3] * dhParams_.a[3] + 2*dhParams_.a[3]*cos(q4_[16*j+8*i + 6])*dhParams_.a[2] + dhParams_.a[2] * dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[16*j+8*i + 6] = asin(-temp1/temp2);
				q3_[16*j+8*i + 7] = M_PI - asin(-temp1/temp2);

				//for q4[0]
				/*temp1 = dhParams_.a[2]*t2[16*j+8*i + 0] + dhParams_.a[3]*cos(q4_[16*j+8*i + 0])*t2[16*j+8*i + 0] - dhParams_.a[3]*sin(q4_[16*j+8*i + 0])*t1;
				temp2 = dhParams_.a[3]*dhParams_.a[3]*cos(q4_[16*j+8*i + 0])*cos(q4_[16*j+8*i + 0]) + dhParams_.a[3]*cos(q4_[16*j+8*i + 0])*dhParams_.a[2] 
						+ dhParams_.a[3]*sin(q4_[16*j+8*i + 0]) + dhParams_.a[2]*dhParams_.a[3]*cos(q4_[16*j+8*i + 0]) + dhParams_.a[2]*dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[16*j+8*i + 0] =  acos(temp1 / temp2);
				q3_[16*j+8*i + 1] = -acos(temp1 / temp2);

				//for q4[2]
				temp1 = dhParams_.a[2]*t2[16*j+8*i + 2] + dhParams_.a[3]*cos(q4_[16*j+8*i + 2])*t2[16*j+8*i + 2] - dhParams_.a[3]*sin(q4_[16*j+8*i + 2])*t1;
				temp2 = dhParams_.a[3]*dhParams_.a[3]*cos(q4_[16*j+8*i + 2])*cos(q4_[16*j+8*i + 2]) + dhParams_.a[3]*cos(q4_[16*j+8*i + 2])*dhParams_.a[2] 
						+ dhParams_.a[3]*sin(q4_[16*j+8*i + 2]) + dhParams_.a[2]*dhParams_.a[3]*cos(q4_[16*j+8*i + 2]) + dhParams_.a[2]*dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[8*i + 2] =  acos(temp1 / temp2);
				q3_[8*i + 3] = -acos(temp1 / temp2);

				//for q4[4]
				temp1 = dhParams_.a[2]*t2[16*j+8*i + 4] + dhParams_.a[3]*cos(q4_[16*j+8*i + 4])*t2[16*j+8*i + 4] - dhParams_.a[3]*sin(q4_[16*j+8*i + 4])*t1;
				temp2 = dhParams_.a[3]*dhParams_.a[3]*cos(q4_[16*j+8*i + 4])*cos(q4_[16*j+8*i + 4]) + dhParams_.a[3]*cos(q4_[16*j+8*i + 4])*dhParams_.a[2] 
						+ dhParams_.a[3]*sin(q4_[16*j+8*i + 4]) + dhParams_.a[2]*dhParams_.a[3]*cos(q4_[16*j+8*i + 4]) + dhParams_.a[2]*dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[16*j+8*i + 4] =  acos(temp1 / temp2);
				q3_[16*j+8*i + 5] = -acos(temp1 / temp2);

				//for q4[6]
				temp1 = dhParams_.a[2]*t2[16*j+8*i + 6] + dhParams_.a[3]*cos(q4_[16*j+8*i + 6])*t2[16*j+8*i + 6] - dhParams_.a[3]*sin(q4_[16*j+8*i + 6])*t1;
				temp2 = dhParams_.a[3]*dhParams_.a[3]*cos(q4_[16*j+8*i + 6])*cos(q4_[16*j+8*i + 6]) + dhParams_.a[3]*cos(q4_[16*j+8*i + 6])*dhParams_.a[2] 
						+ dhParams_.a[3]*sin(q4_[16*j+8*i + 6]) + dhParams_.a[2]*dhParams_.a[3]*cos(q4_[16*j+8*i + 6]) + dhParams_.a[2]*dhParams_.a[2];
				if (fabs(temp1 - temp2) < 0.01) temp1 = temp2;

				q3_[16*j+8*i + 6] =  acos(temp1 / temp2);
				q3_[16*j+8*i + 7] = -acos(temp1 / temp2);*/
			}
		}

		for (int i = 0; i < 32; i++) {
			q2_[i] = q12[(int)(i/8)] - q1_[i];
			q5_[i] = w5[(int)(i/16)] - q3_[i] - q4_[i];

			q1_[i] = atan2(sin(q1_[i] - dhParams_.theta[0]), cos(q1_[i] - dhParams_.theta[0]));
			q2_[i] = atan2(sin(q2_[i] - dhParams_.theta[1]), cos(q2_[i] - dhParams_.theta[1]));
			q3_[i] = atan2(sin(q3_[i] - dhParams_.theta[2]), cos(q3_[i] - dhParams_.theta[2]));
			q4_[i] = atan2(sin(q4_[i] - dhParams_.theta[3]), cos(q4_[i] - dhParams_.theta[3]));
			q5_[i] = atan2(sin(q5_[i] - dhParams_.theta[4]), cos(q5_[i] - dhParams_.theta[4]));
		}

		number_of_solutions = 32;
	}

	return number_of_solutions;

}