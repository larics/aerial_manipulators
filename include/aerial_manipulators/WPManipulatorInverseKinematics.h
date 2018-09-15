#ifndef WPMANIPULATORINVERSEINEMATICS_H
#define WPMANIPULATORINVERSEINEMATICS_H

#include <cmath>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>
#include <aerial_manipulators/WPManipulatorDirectKinematics.h>

class WPManipulatorInverseKinematics
{
	public:
		WPManipulatorInverseKinematics(void);
		void LoadParameters(std::string file);
		int ik_calculate(float x, float y, float z, float rot_y, float rot_z);
		void setDHparams(WPManipulatorDirectKinematics::DH_Parameters_TypeDef dhParams);
		Eigen::MatrixXd getJacobian(double q1, double q2, double q3, double q4, double q5);
		int ik_T12_calculate(float y, float rot_z);
		int ik_T26_calculate(float x, float y, float rot_z);
		int getFeasibleRotationT26(float x, float y, float *rot_z);
		int getMaxNumberOfSolutions(void);
		float *getQ1(void);
		float *getQ2(void);
		float *getQ3(void);
		float *getQ4(void);
		float *getQ5(void);
	private:
		WPManipulatorDirectKinematics::DH_Parameters_TypeDef dhParams_;
		float q1_[32], q2_[32], q3_[32], q4_[32], q5_[32];
		bool isInit;
};


#endif