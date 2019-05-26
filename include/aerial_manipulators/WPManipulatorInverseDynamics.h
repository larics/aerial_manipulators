#ifndef WPMANIPULATORINVERSEDYNAMICS_H
#define WPMANIPULATORINVERSEDYNAMICS_H

#include <math.h>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <array>
#include <chrono>

class WPManipulatorInverseDynamics {

public:

	WPManipulatorInverseDynamics(float *massset, float dcset[6][3], float D_set[6][3][3]);

	// Set functions
	void setMass(float *x);

	void setDc(float x[6][3]);

	void setD_(float x[6][3][3]);

	// Get functions
	void getTau(float x[6]);

	// Calculate tau
	Eigen::Matrix<float, 5, 1> calculateID(float *q, float *v0, float *w0, float dtime);

private:

	// Constants
	float mass_[6];
	float dc_[6][3];
	float Dh_[6][3][3];

	float theta0_[6] = { 1.570796327, 0, 0, 0, -1.570796327, 0 };
	float alpha_[6] = { 0, -1.570796327, 0, 0, -1.570796327, 0 };
	float d_[6] = { 0, 0, 0, 0, 0, 0.04526 };
	float a_[6] = { 0.12249, 0.1365, 0.075511, 0.072489, 0, 0 };

	// Variables
	float qOld_[6];
	float dq_[6];
	float dqOld_[6] = {0, 0, 0, 0, 0, 0};
	float ddq_[6] = {0, 0, 0, 0, 0, 0};
	float T_[6][4][4];

	float v_[6][3];
	float w_[6][3];
	float v0Old_[3] = {0, 9.81, 0};
	float w0Old_[3] = {0, 0, 0};
	float dv_[6][3];
	float dw_[6][3];

	float z_[5][3];
	float ds_[6][3];

	float f_[7][3];
	float n_[7][3];
	float tau_[6] = {0, 0, 0, 0, 0, 0};

	float dr_[6][3];
	float D_[6][3][3];

	std::chrono::time_point<std::chrono::system_clock> timeOld_ = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> timeNew_;
};



#endif
