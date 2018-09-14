#ifndef WPMANIPULATORDIRECTKINEMATICS_H
#define WPMANIPULATORDIRECTKINEMATICS_H

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>

class WPManipulatorDirectKinematics
{
	public:
		typedef struct
		{
			float theta[6];
			float alpha[6];
			float d[6];
			float a[6];
		} DH_Parameters_TypeDef;

		WPManipulatorDirectKinematics(void);
		Eigen::Matrix4d dk_calculate(float q1, float q2, float q3, float q4, float q5);
		void LoadParameters(std::string file);

	private:
		WPManipulatorDirectKinematics::DH_Parameters_TypeDef dhParams_;
		bool isInit;
};

#endif