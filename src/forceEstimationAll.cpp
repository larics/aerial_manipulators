#include "pch.h"
#include <iostream>
#include <array>
#include <Eigen/Dense>
#include <math.h>
#include <chrono>

// Utility function

// Transpose of 2D array
void transpose(float mtxA[][3], float res[][3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			res[j][i] = mtxA[i][j];
		}
	}
}

// Assign 3x1 vector from 3xk matrix
void assignVec3(float vecA[][3], float res[], int k) {
	for (int i = 0; i < 3; i++){
		res[i] = vecA[k][i];
	}
}

// Assign 3x3 matrix from kx3x3 3D array
void assignMtx3x3(float mtxA[][3][3], float res[][3], int k) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			res[i][j] = mtxA[k][i][j];
		}
	}
}

// Assign 4x4 matrix from kx4x4 3D array
void assignMtx4x4(float mtxA[][4][4], float res[][3], int k) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			res[i][j] = mtxA[k][i][j];
		}
	}
}

// Reassign kx3 matrix collumn from 3x1 vector
void reassignVec3(float vecA[], float res[][3], int k) {
	for (int i = 0; i < 3; i++){
		res[k][i] = vecA[i];
	}
}

// Reassign kx3x3 3D array matrix from 3x3 matrix
void reassignMtx3x3(float mtxA[][3], float res[][3][3], int k) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			res[k][i][j] = mtxA[i][j];
		}
	}
}

// Multipy two 3x3 2D arrays
void mtxMul3x3(float mtx1[][3], float mtx2[][3], float mtxRez[][3])
{
	float help = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++) { help = help + mtx1[i][k] * mtx2[k][j]; }
			mtxRez[i][j] = help;
			help = 0;
		}
	}
}

// Calculate cross product of 2 3x1 array vectors
void crossProduct(float vecA[], float vecB[], float res[])
{
	res[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
	res[1] = vecA[0] * vecB[2] - vecA[2] * vecB[0];
	res[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

// Assign nx4x4 2D array with values from Eigen::Matrix4d
void matrixTo2DArray(Eigen::Matrix4d mtxA, float res[][4][4], int n) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			res[n][i][j] = mtxA(i, j);
		}
	}
}

// Calculation of transfomation matrix
Eigen::Matrix4d transformCalculate(float thetak, float dk, float ak, float alphak, float theta0k) {
	Eigen::Matrix4d T(4, 4);
	T << cos(thetak + theta0k), -cos(alphak) * sin(thetak + theta0k), sin(alphak) * sin(thetak + theta0k), ak * cos(thetak + theta0k),
		sin(thetak + theta0k), cos(alphak) * cos(thetak + theta0k), -sin(alphak) * cos(thetak + theta0k), ak * sin(thetak + theta0k),
		0, sin(alphak), cos(alphak), dk,
		0, 0, 0, 1;
	return T;
}

// These aren't needed -> Should be assigned from .yaml file
void setNx3x3(float mtxA[][3][3], float res[][3][3], int n_) { for (int i = 0; i < n_; i++) { for (int j = 0; j < 3; j++) { for (int k = 0; k < 3; k++) { res[i][j][k] = mtxA[i][j][k]; } } } }
void setNx4x4(float mtxA[][4][4], float res[][4][4], int n_) { for (int i = 0; i < n_; i++) { for (int j = 0; j < 4; j++) { for (int k = 0; k < 4; k++) { res[i][j][k] = mtxA[i][j][k]; } } } }
void setNx3(float mtxA[][3], float res[][3], int n_) { for (int i = 0; i < n_; i++) { for (int j = 0; j < 3; j++) { res[i][j] = mtxA[i][j]; } } }

// Implementation of Inverse Dynamics inside of the InverseDynamics class
// Class constructor takes mass_, dc_ and Dh_ as they are constant throughout calculations
// Function setAll takes dq_ and ddq_ vectors of velocity and acceleration with dq_[0] = ddq_[0] = anything, since dq0 and ddq0 aren'T_ assigned to manipulator,
// T_ is an array of transformation matrices: T00, T01, T02, T03, T04, T05 and v0, w0, dv0, dw0, f6, n6 are initial conditions
// Function calculateID calculates torques of joints using values set with setAll function and saves it in variable "tau"
// To get tau use function getTau
// IMPORTANT: All values are indexed as they are indexed in reality, so, tau[1] is the torque of the first joint and so on. tau[0] is always equal to 0 as it isn'T_ assigned to the manipulator
class InverseDynamics {

public:

	// Constructor -- > this should read from .yaml file !!
	InverseDynamics(float *massset, float dcset[6][3], float D_set[6][3][3]) {
		setMass(massset);
		setDc(dcset);
		setD_(D_set);
	}

	// Set functions
	void setMass(float *x) {
		std::copy(x, x+6, std::begin(mass_));
	}

	void setDc(float x[6][3]) {
		setNx3(x, dc_, 6);
	}

	void setD_(float x[6][3][3]) {
		setNx3x3(x, Dh_, 6);
	}

	// Get functions
	void getTau(float x[6]) {
		std::copy(tau, tau + 6, x);
	}

	// Calculate tau
	void calculateID(float *q, float *v0, float *w0, float dtime) {

		// Get time
		timeNew_ = std::chrono::system_clock::now();
		std::chrono::duration<float> elapsedTime = timeOld_ - timeNew_;
		float time = elapsedTime.count();
		timeOld_ = timeNew_;

		if (dtime ~= 0) {
			dq_[i] = (q[i] - qOld_[i]) / dtime;
			qOld_[i] = q[i];
			ddq_[i] = (dq_[i] - dqOld_[i]) / dtime;
			dqOld_[i] = dq_[i];
		} else {
			for (int i = 0; i < 6; i++) {
				dq_[i] = 0;
				ddq_[i] = 0;
				qOld_[i] = dq[i];
				dqOld[i] = dq_[i];
		}
		// Calculate dq and ddq

		// Assign appropriate vectors to arrays (implementation of dv_ dw_ assignment should be done here)
		for (int i = 0; i < 3; i++) {
			v_[0][i] = v0[i];
			w_[0][i] = w0[i];
			f_[6][i] = 0;
			n_[6][i] = 0;
		}

		// This shouldn't be needed online, should be implemented above
		dv_[0][0] = 0;
		dv_[0][1] = 9.81;
		dv_[0][2] = 0;
		dw_[0][0] = 0;
		dw_[0][1] = 0;
		dw_[0][2] = 0;

		// Calculation of transformation matrices using Eigne::Matrix4d
		Eigen::Matrix4d T00(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4), T56(4, 4);

		T00 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		T01 = transformCalculate(q[1], d_[0], a_[0], alpha_[0], theta0_[0]);
		T12 = transformCalculate(q[2], d_[1], a_[1], alpha_[1], theta0_[1]);
		T23 = transformCalculate(q[3], d_[2], a_[2], alpha_[2], theta0_[2]);
		T34 = transformCalculate(q[4], d_[3], a_[3], alpha_[3], theta0_[3]);
		T45 = transformCalculate(q[5], d_[4], a_[4], alpha_[4], theta0_[4]);
		T56 = transformCalculate(0, d_[5], a_[5], alpha_[5], theta0_[5]);

		Eigen::Matrix4d T02(4, 4), T03(4, 4), T04(4, 4), T05(4, 4);
		T02 = T01 * T12;
		T03 = T02 * T23;
		T04 = T03 * T34;
		T05 = T04 * T45 * T56;

		matrixTo2DArray(T00, T_, 0);
		matrixTo2DArray(T01, T_, 1);
		matrixTo2DArray(T02, T_, 2);
		matrixTo2DArray(T03, T_, 3);
		matrixTo2DArray(T04, T_, 4);
		matrixTo2DArray(T05, T_, 5);

		// Forward for loop
		for (int i = 1; i < 6; i++) {
			z_[i - 1][0] = T_[i - 1][0][2];
			z_[i - 1][1] = T_[i - 1][1][2];
			z_[i - 1][2] = T_[i - 1][2][2];

			w_[i][0] = w_[i - 1][0] + dq_[i] * z_[i - 1][0];
			w_[i][1] = w_[i - 1][1] + dq_[i] * z_[i - 1][1];
			w_[i][2] = w_[i - 1][2] + dq_[i] * z_[i - 1][2];

			float dqz[3] = { dq_[i] * z_[i - 1][0] , dq_[i] * z_[i - 1][1] ,dq_[i] * z_[i - 1][2] };
			float crosswdqz[3];
			float wHelp1[3];
			assignVec3(w_, wHelp1, i - 1);
			crossProduct(wHelp1, dqz, crosswdqz);

			dw_[i][0] = dw_[i - 1][0] + ddq_[i] * z_[i - 1][0] + crosswdqz[0];
			dw_[i][1] = dw_[i - 1][1] + ddq_[i] * z_[i - 1][1] + crosswdqz[1];
			dw_[i][2] = dw_[i - 1][2] + ddq_[i] * z_[i - 1][2] + crosswdqz[2];

			ds_[i][0] = T_[i][0][3] - T_[i - 1][0][3];
			ds_[i][1] = T_[i][1][3] - T_[i - 1][1][3];
			ds_[i][2] = T_[i][2][3] - T_[i - 1][2][3];

			float crosswds[3];
			float dsHelp[3];
			float wHelp2[3];
			assignVec3(w_, wHelp2, i);
			assignVec3(ds_, dsHelp, i);
			crossProduct(wHelp2, dsHelp, crosswds);
			float crosswcrosswds[3];
			crossProduct(wHelp2, crosswds, crosswcrosswds);
			float dwHelp[3];
			assignVec3(dw_, dwHelp, i);
			float crossdwds[3];
			crossProduct(dwHelp, dsHelp, crossdwds);

			dv_[i][0] = dv_[i - 1][0] + crossdwds[0] + crosswcrosswds[0];
			dv_[i][1] = dv_[i - 1][1] + crossdwds[1] + crosswcrosswds[1];
			dv_[i][2] = dv_[i - 1][2] + crossdwds[2] + crosswcrosswds[2];
		}

		// Backward for loop
		for (int i = 5; i > 0; i--) {
			dr_[i][0] = T_[i][0][0] * dc_[i][0] + T_[i][0][1] * dc_[i][1] + T_[i][0][2] * dc_[i][2];
			dr_[i][1] = T_[i][1][0] * dc_[i][0] + T_[i][1][1] * dc_[i][1] + T_[i][1][2] * dc_[i][2];
			dr_[i][2] = T_[i][2][0] * dc_[i][0] + T_[i][2][1] * dc_[i][1] + T_[i][2][2] * dc_[i][2];

			float drHelp[3];
			assignVec3(dr_, drHelp, i);
			float dwHelp[3];
			assignVec3(dw_, dwHelp, i);
			float crossdwdr[3];
			crossProduct(dwHelp, drHelp, crossdwdr);
			float crossdwcrossdwdr[3];
			crossProduct(dwHelp, crossdwdr, crossdwcrossdwdr);

			f_[i][0] = f_[i + 1][0] + mass_[i] * (dv_[i][0] + crossdwdr[0]) + crossdwcrossdwdr[0];
			f_[i][1] = f_[i + 1][1] + mass_[i] * (dv_[i][1] + crossdwdr[1]) + crossdwcrossdwdr[1];
			f_[i][2] = f_[i + 1][2] + mass_[i] * (dv_[i][2] + crossdwdr[2]) + crossdwcrossdwdr[2];

			float D_help[3][3];
			assignMtx3x3(Dh_, D_help, i);
			float Rhelp[3][3];
			assignMtx4x4(T_, Rhelp, i);
			float Rt[3][3];
			transpose(Rhelp, Rt);
			float D_Rt[3][3];
			mtxMul3x3(D_help, Rt, D_Rt);
			float RD_Rt[3][3];
			mtxMul3x3(Rhelp, D_Rt, RD_Rt);
			reassignMtx3x3(RD_Rt, D_, i);

			float Dw[3];
			Dw[0] = D_[i][0][0] * w_[i][0] + D_[i][0][1] * w_[i][1] + D_[i][0][2] * w_[i][2];
			Dw[1] = D_[i][1][0] * w_[i][0] + D_[i][1][1] * w_[i][1] + D_[i][1][2] * w_[i][2];
			Dw[2] = D_[i][2][0] * w_[i][0] + D_[i][2][1] * w_[i][1] + D_[i][2][2] * w_[i][2];
			float crossdwDw[3];
			crossProduct(dwHelp, Dw, crossdwDw);
			float Ddw[3];
			Ddw[0] = D_[i][0][0] * dw_[i][0] + D_[i][0][1] * dw_[i][1] + D_[i][0][2] * dw_[i][2];
			Ddw[1] = D_[i][1][0] * dw_[i][0] + D_[i][1][1] * dw_[i][1] + D_[i][1][2] * dw_[i][2];
			Ddw[2] = D_[i][2][0] * dw_[i][0] + D_[i][2][1] * dw_[i][1] + D_[i][2][2] * dw_[i][2];
			float fHelp2[3];
			float fHelp1[3];
			assignVec3(f_, fHelp1, i);
			assignVec3(f_, fHelp2, i + 1);
			float dsHelp[3];
			assignVec3(ds_, dsHelp, i);
			float crossdrf2[3];
			crossProduct(drHelp, fHelp2, crossdrf2);
			float dsdr[3];
			dsdr[0] = dsHelp[0] + drHelp[0];
			dsdr[1] = dsHelp[1] + drHelp[1];
			dsdr[2] = dsHelp[2] + drHelp[2];
			float crossdsdrf1[3];
			crossProduct(dsdr, fHelp1, crossdsdrf1);
			n_[i][0] = n_[i + 1][0] + crossdsdrf1[0] - crossdrf2[0] + Ddw[0] + crossdwDw[0];
			n_[i][1] = n_[i + 1][1] + crossdsdrf1[1] - crossdrf2[1] + Ddw[1] + crossdwDw[1];
			n_[i][2] = n_[i + 1][2] + crossdsdrf1[2] - crossdrf2[2] + Ddw[2] + crossdwDw[2];

			tau[i] = n_[i][0] * z_[i - 1][0] + n_[i][1] * z_[i - 1][1] + n_[i][2] * z_[i - 1][2];
		}
	}

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
	float tau[6] = {0, 0, 0, 0, 0, 0};

	float dr_[6][3];
	float D_[6][3][3];

	std::chrono::time_point<std::chrono::system_clock> timeOld_ = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> timeNew_;

	bool isFirst = true;
};


class ForceEstimation {

public:

	Eigen::Matrix<float, 6, 5> jacobian(float *q) {

		Eigen::Matrix<float, 6, 5> jacob;

		jacob(0, 0) = 0.072489*cos(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.12249*cos(q[1]) - 0.04526*cos(q[3] + q[4] + q[5])*cos(q[1] + q[2]) - 0.075511*cos(q[1] + q[2])*cos(q[3]) - 0.072489*cos(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*cos(q[1] + q[2]);
		jacob(0, 1) = -cos(q[1] + q[2])*(0.04526 * cos(q[3] + q[4] + q[5]) + 0.072489 * cos(q[3] + q[4]) + 0.075511 * cos(q[3]) + 0.1365);
		jacob(0, 2) = sin(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]) + 0.075511 * sin(q[3]));
		jacob(0, 3) = sin(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]));
		jacob(0, 4) = 0.04526 * sin(q[3] + q[4] + q[5])*sin(q[1] + q[2]);

		jacob(1, 0) = 0.072489*sin(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.12249*sin(q[1]) - 0.04526*cos(q[3] + q[4] + q[5])*sin(q[1] + q[2]) - 0.075511*sin(q[1] + q[2])*cos(q[3]) - 0.072489*sin(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*sin(q[1] + q[2]);
		jacob(1, 1) = -sin(q[1] + q[2])*(0.4526 * cos(q[3] + q[4] + q[5]) + 0.72489 * cos(q[3] + q[4]) + 0.75511 * cos(q[3]) + 1.365);
		jacob(1, 2) = -cos(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]) + 0.075511 * sin(q[3]));
		jacob(1, 3) = -cos(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]));
		jacob(1, 4) = -0.04526*sin(q[3] + q[4] + q[5])*cos(q[1] + q[2]);

		jacob(2, 0) = 0;
		jacob(2, 1) = 0;
		jacob(2, 2) = -0.04526*cos(q[3] + q[4] + q[5]) - 0.072489*cos(q[3] + q[4]) - 0.075511*cos(q[3]);
		jacob(2, 3) = -0.04526*cos(q[3] + q[4] + q[5]) - 0.072489*cos(q[3] + q[4]);
		jacob(2, 4) = -0.04526*cos(q[3] + q[4] + q[5]);

		jacob(3, 0) = 0;
		jacob(3, 1) = 0;
		jacob(3, 2) = -cos(q[1] + q[2]);
		jacob(3, 3) = -cos(q[1] + q[2]);
		jacob(3, 4) = -cos(q[1] + q[2]);

		jacob(4, 0) = 0;
		jacob(4, 1) = 0;
		jacob(4, 2) = -sin(q[1] + q[2]);
		jacob(4, 3) = -sin(q[1] + q[2]);
		jacob(4, 4) = -sin(q[1] + q[2]);

		jacob(5, 0) = 1;
		jacob(5, 1) = 1;
		jacob(5, 2) = 0;
		jacob(5, 3) = 0;
		jacob(5, 4) = 0;

		return jacob;
	}

	Eigen::Matrix<float, 5, 1> currentToTorque(float *ia) {
		Eigen::Matrix<float, 5, 1> torque;
		for (int i = 0; i < 5; i++) {
			torque(i) = k_[i] * ia[i];
		}
		return torque;
	}

	Eigen::Matrix<float, 5, 1> frictionTorque(float *dq) {
		float sign[5];
		for (int i = 0; i < 5; i++) {
			if (dq[i] > 0) {
				sign[i] = 1;
			}
			else if (dq[i] < 0) {
				sign[i] = -1;
			}
			else {
				sign[i] = 0;
			}
		}
		Eigen::Matrix<float, 5, 1> torque;
		for (int i = 0; i < 5; i++) {
			torque(i) = vc_[i] * sign[i] + vv_[i] * dq[i];
		}
		return torque;
	}

	Eigen::Matrix<float, 6, 1> forceEstimate(float *q, float *tau, float *ia) {
		Eigen::Matrix<float, 6, 1> Fn;

		// Get time
		timeNew_ = std::chrono::system_clock::now();
		std::chrono::duration<float> elapsedTime = timeOld_ - timeNew_;
		float time = elapsedTime.count();
		timeOld_ = timeNew_;

		// Calculate dq
		for (int i = 0; i < 5; i++) {
			// Doesn't work offline:
			//dq_[i] = (q[i] - qOld_[i]) / time;
			//qOld_[i] = q[i];
			// ____________________
			dq_[i] = 0;
		}

		Eigen::Matrix<float, 6, 5> jacob;
		Eigen::Matrix<float, 5, 6> jacobt;
		Eigen::Matrix<float, 5, 1> tauki, taufr, taumi;
		for (int i = 0; i < 5; i++) {
			taumi(i) = tau[i + 1];
		}
		tauki = currentToTorque(ia);
		taufr = frictionTorque(dq_);
		jacob = jacobian(q);

		Eigen::Matrix<float, 6, 1> force;
		jacobt = jacob.transpose();
		force = jacobt.completeOrthogonalDecomposition().pseudoInverse() * (tauki - taumi - taufr);

		return force;
	}

private:

	float k_[5] = { 0.1, 0.1, 0.1, 0.1, 0.1 };
	float vc_[5] = { 0.1, 0.1, 0.1, 0.1, 0.1 };
	float vv_[5] = { 0.1, 0.1, 0.1, 0.1, 0.1 };

	float dq_[5];
	float dqOld_[5] = { 0, 0, 0, 0, 0 };

	std::chrono::time_point<std::chrono::system_clock> timeOld_ = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> timeNew_;

};


int main()
{
	const int numOfJoints = 5;
	float massh[numOfJoints + 1] = { 0.0, 0.01715, 0.10663, 0.17449, 0.01133, 0.08200 };
	float dqh[numOfJoints + 1] = { 0,0,0,0,0,0 };
	float ddqh[numOfJoints + 1] = { 0,0,0,0,0,0 };

	float v0[3];
	float w0[3];
	float dv0[3];
	float dw0[3];

	float Th[numOfJoints + 1][4][4] = { {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}},
	{{0, -1, 0, 0}, {1, 0, 0, 0.12249}, {0, 0, 1, 0}, {0, 0, 0, 1}},
	{{0, 0, -1, 0}, {1, 0, 0, 0.25899}, {0, -1, 0, 0}, {0, 0, 0, 1}},
	{{0, 0, -1, 0}, {1, 0, 0, 0.334501}, {0, -1, 0, 0}, {0, 0, 0, 1}},
	{{0, 0, -1, 0}, {1, 0, 0, 0.40699}, {0, -1, 0, 0}, {0, 0, 0, 1}},
	{{0, 1, 0, 0}, {0, 0, 1, 0.45225}, {1, 0, 0, 0}, {0, 0, 0, 1}} };

	v0[0] = 0;
	v0[1] = 0;
	v0[2] = 0;
	w0[0] = 0;
	w0[1] = 0;
	w0[2] = 0;

	float dch[6][3] = { {0 / 1000, 0 / 1000, 0 / 1000}, {-61.24 / 1000, 0.0 / 1000, -0.09 / 1000}, {-110.80 / 1000, -0.63 / 1000, -0.59 / 1000}, {-37.76 / 1000, 0.0 / 1000, 0.18 / 1000}, {-36.24 / 1000, 0.0 / 1000, -0.59 / 1000}, {-0.21 / 1000, -0.83 / 1000, -33.46 / 1000} };

	float D_h[numOfJoints + 1][3][3] = { {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
	{ {8656.5 / 1000000000, 0.18 / 1000000000, 96.42 / 1000000000}, {0.18 / 1000000000, 100511.76 / 1000000000, 0.00 / 1000000000}, {96.42 / 1000000000, 0.00 / 1000000000, 94741.01 / 1000000000} },
	{ {25494.33 / 1000000000, 8639.22 / 1000000000, 6994.05 / 1000000000}, {8639.22 / 1000000000, 1441278.11 / 1000000000, 40.06 / 1000000000}, {6994.05 / 1000000000, 40.06 / 1000000000, 1443420.56 / 1000000000} },
	{ {30962.85 / 1000000000, 0.59 / 1000000000, -1210.86 / 1000000000}, {0.59 / 1000000000, 416023.24 / 1000000000, 0.01 / 1000000000}, {-1210.86 / 1000000000, 0.01 / 1000000000, 408445.34 / 1000000000} },
	{ {5406.00 / 1000000000, 0.07 / 1000000000, 243.11 / 1000000000}, {0.07 / 1000000000, 27148.80 / 1000000000, 0.00 / 1000000000}, {243.11 / 1000000000, 0.00 / 1000000000, 23472.02 / 1000000000} },
	{ {114783.85 / 1000000000, 14.13 / 1000000000, 571.94 / 1000000000}, {14.13 / 1000000000, 112108.36 / 1000000000, 2462.51 / 1000000000}, {571.94 / 1000000000, 2462.51 / 1000000000, 13291.56 / 1000000000} } };

	float f6[3];
	float n6[3];

	float tauh[numOfJoints + 1];

	f6[0] = 0;
	f6[1] = 0;
	f6[2] = 0;

	n6[0] = 0;
	n6[1] = 0;
	n6[2] = 0;

	float theta[6] = {0, 0, 0, 0, 0, 0};

	InverseDynamics ID(massh, dch, D_h);
	ID.calculateID(theta, v0, w0, f6, n6);
	ID.getTau(tauh);

	for (int i = 1; i < 6; i++) {
		std::cout << tauh[i] << "\n";
	}

}




































#include <math.h>
#include "yaml-cpp/yaml.h"
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

class WPManipulatorForceEstimation {
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	sensor_msgs::JointState js_;
	aerial_manipulators::float6 force_;

	float q_pos_[6];
	int rate_;
	float tauget_[6];
	float tauki_[5];

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

	Eigen::Matrix<float, 6, 1> forceEstimate(float *q, float *v0, float *w0, float *taukih, float dtime);

	Eigen::Matrix<float, 6, 5> jacobian(float *q);

	void joint_states_Callback(const sensor_msgs::JointState& msg);

	void run(void);

};

Eigen::Matrix<float, 6, 1> WPManipulatorForceEstimation::forceEstimate(float *q, float *v0, float *w0, float *taukih, float dtime) {

	WPManipulatorInverseDynamics WPMID_(massh_, dch_, D_h_);
	Eigen::Matrix<float, 6, 5> jacob;
	// Eigen::Matrix<float, 5, 6> jacobt;
	Eigen::Matrix<float, 5, 1> tauki, taumi;

	taumi = WPMID_.calculateID(q, v0, w0, dtime);

	for (int i = 0; i < 5; i++) {
		taumi(i) = taukih[i + 1];
	}
	jacob = jacobian(q);

	Eigen::Matrix<float, 6, 1> force;
	Eigen::MatrixXd jacobt = jacob.transpose();
	//Eigen::CompleteOrthogonalDecomposition<float, 5, 6> cod(jacobt);
	// force = jacobt.completeOrthogonalDecomposition().pseudoInverse() * (tauki - taumi);
	//force = cod.pseudoInverse * (tauki - taumi);
	return force;
}


Eigen::Matrix<float, 6, 5> WPManipulatorForceEstimation::jacobian(float *q) {

	Eigen::Matrix<float, 6, 5> jacob;

	jacob(0, 0) = 0.072489*cos(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.12249*cos(q[1]) - 0.04526*cos(q[3] + q[4] + q[5])*cos(q[1] + q[2]) - 0.075511*cos(q[1] + q[2])*cos(q[3]) - 0.072489*cos(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*cos(q[1] + q[2]);
	jacob(0, 1) = -cos(q[1] + q[2])*(0.04526 * cos(q[3] + q[4] + q[5]) + 0.072489 * cos(q[3] + q[4]) + 0.075511 * cos(q[3]) + 0.1365);
	jacob(0, 2) = sin(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]) + 0.075511 * sin(q[3]));
	jacob(0, 3) = sin(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]));
	jacob(0, 4) = 0.04526 * sin(q[3] + q[4] + q[5])*sin(q[1] + q[2]);

	jacob(1, 0) = 0.072489*sin(q[1] + q[2])*sin(q[3])*sin(q[4]) - 0.12249*sin(q[1]) - 0.04526*cos(q[3] + q[4] + q[5])*sin(q[1] + q[2]) - 0.075511*sin(q[1] + q[2])*cos(q[3]) - 0.072489*sin(q[1] + q[2])*cos(q[3])*cos(q[4]) - 0.1365*sin(q[1] + q[2]);
	jacob(1, 1) = -sin(q[1] + q[2])*(0.4526 * cos(q[3] + q[4] + q[5]) + 0.72489 * cos(q[3] + q[4]) + 0.75511 * cos(q[3]) + 1.365);
	jacob(1, 2) = -cos(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]) + 0.075511 * sin(q[3]));
	jacob(1, 3) = -cos(q[1] + q[2])*(0.04526 * sin(q[3] + q[4] + q[5]) + 0.072489 * sin(q[3] + q[4]));
	jacob(1, 4) = -0.04526*sin(q[3] + q[4] + q[5])*cos(q[1] + q[2]);

	jacob(2, 0) = 0;
	jacob(2, 1) = 0;
	jacob(2, 2) = -0.04526*cos(q[3] + q[4] + q[5]) - 0.072489*cos(q[3] + q[4]) - 0.075511*cos(q[3]);
	jacob(2, 3) = -0.04526*cos(q[3] + q[4] + q[5]) - 0.072489*cos(q[3] + q[4]);
	jacob(2, 4) = -0.04526*cos(q[3] + q[4] + q[5]);

	jacob(3, 0) = 0;
	jacob(3, 1) = 0;
	jacob(3, 2) = -cos(q[1] + q[2]);
	jacob(3, 3) = -cos(q[1] + q[2]);
	jacob(3, 4) = -cos(q[1] + q[2]);

	jacob(4, 0) = 0;
	jacob(4, 1) = 0;
	jacob(4, 2) = -sin(q[1] + q[2]);
	jacob(4, 3) = -sin(q[1] + q[2]);
	jacob(4, 4) = -sin(q[1] + q[2]);

	jacob(5, 0) = 1;
	jacob(5, 1) = 1;
	jacob(5, 2) = 0;
	jacob(5, 3) = 0;
	jacob(5, 4) = 0;

	return jacob;
}


WPManipulatorForceEstimation::WPManipulatorForceEstimation(void) {
	sub_ = nh_.subscribe("joint_states", 1, &WPManipulatorForceEstimation::joint_states_Callback, this);
	rate_ = 10;
	pub_ = nh_.advertise<aerial_manipulators::float6>("force", 1);
}

WPManipulatorForceEstimation::~WPManipulatorForceEstimation(void) {}

void WPManipulatorForceEstimation::joint_states_Callback(const sensor_msgs::JointState &msg)
{
	for (int i = 0; i < 5; i++) {
		q_pos_[i+1] = msg.position[i];
		tauki_[i] = msg.effort[i];
	}
}


void WPManipulatorForceEstimation::run(void)
{

	int count = 0;
	ros::Rate loop_rate(rate_);
	while(ros::ok())
	{
		ros::spinOnce();

		float v0[3] = {0, 0, 0}, w0[3] = {0, 0, 0}, dtime;
		dtime = 1/rate_;

		WPManipulatorInverseDynamics WPMID_(massh_, dch_, D_h_);
		if (count == 0) {
			dtime = 0;
		}

		Eigen::Matrix<float, 6, 1> force;
		force = forceEstimate(q_pos_, v0, w0, tauki_, dtime);

		force_.a = force[0];
		force_.b = force[1];
		force_.c = force[2];
		force_.d = force[3];
		force_.e = force[4];
		force_.f = force[5];

		pub_.publish(force_);

		loop_rate.sleep();
		++count;
	}

	return;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	WPManipulatorForceEstimation WPMFE_node;
	WPMFE_node.run();
	return 0;
}
