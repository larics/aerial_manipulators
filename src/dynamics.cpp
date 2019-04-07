#include "pch.h"
#include <iostream>
#include <array>

// Utility functions
void transpose(float mtxA[][3], float res[][3]) {for (int i = 0; i < 3; i++) {for (int j = 0; j < 3; j++) {res[j][i] = mtxA[i][j];}}}

void scaleD_(float mtx[][3][3]) {for (int i = 0; i < 6; i++){for (int j = 0; j < 3; j++){for (int k = 0; k < 3; k++){mtx[i][j][k] = mtx[i][j][k] / 1000000000;}}}}

void scaledc(float mtx[][3]) {for (int i = 0; i < 6; i++){for (int j = 0; j < 3; j++){mtx[i][j] = mtx[i][j] / 1000;}}}

void printVec(float vecA[][3], int k) {std::cout << vecA[k][0] << ", " << vecA[k][1] << ", " << vecA[k][2] << "\n";}

void assignVec3(float vecA[][3], float res[], int k) {for (int i = 0; i < 3; i++){res[i] = vecA[k][i];}}

void reassignVec3(float vecA[], float res[][3], int k) {for (int i = 0; i < 3; i++){res[k][i] = vecA[i];}}

void reassignMtx3x3(float mtxA[][3], float res[][3][3], int k) {for (int i = 0; i < 3; i++){for (int j = 0; j < 3; j++){res[k][i][j] = mtxA[i][j];}}}

void assignMtx3x3(float mtxA[][3][3], float res[][3], int k) {for (int i = 0; i < 3; i++){for (int j = 0; j < 3; j++){res[i][j] = mtxA[k][i][j];}}}

void assignMtx4x4(float mtxA[][4][4], float res[][3], int k) {for (int i = 0; i < 3; i++){for (int j = 0; j < 3; j++){res[i][j] = mtxA[k][i][j];}}}

void setNx3x3(float mtxA[][3][3], float res[][3][3], int n) { for (int i = 0; i < n; i++) { for (int j = 0; j < 3; j++) { for (int k = 0; k < 3; k++) { res[i][j][k] = mtxA[i][j][k]; } } } }

void setNx4x4(float mtxA[][4][4], float res[][4][4], int n) { for (int i = 0; i < n; i++) { for (int j = 0; j < 4; j++) { for (int k = 0; k < 4; k++) { res[i][j][k] = mtxA[i][j][k]; } } } }

void setNx3(float mtxA[][3], float res[][3], int n) {for (int i = 0; i < n; i++) {for (int j = 0; j < 3; j++) {res[i][j] = mtxA[i][j];}}}

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

void crossProduct(float vecA[], float vecB[], float res[])
{
	res[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
	res[1] = vecA[0] * vecB[2] - vecA[2] * vecB[0];
	res[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

// Implementation of Inverse Dynamics inside of the InverseDynamics class
// Class constructor takes mass, dc and D_ as they are constant throughout calculations
// Function setAll takes dq and ddq vectors of velocity and acceleration with dq[0] = ddq[0] = anything, since dq0 and ddq0 aren't assigned to manipulator,
// T is an array of transformation matrices: T00, T01, T02, T03, T04, T05 and v0, w0, dv0, dw0, f6, n6 are initial conditions
// Function calculateID calculates torques of joints using values set with setAll function and saves it in variable "tau"
// To get tau use function getTau
// IMPORTANT: All values are indexed as they are indexed in reality, so, tau[1] is the torque of the first joint and so on. tau[0] is always equal to 0 as it isn't assigned to the manipulator

class InverseDynamics {

public:

	// Constructor
	InverseDynamics(float massset[6], float dcset[6][3], float D_set[6][3][3])
	{
		setMass(massset);
		setDc(dcset);
		setD_(D_set);
	}

	// Set functions
	void setMass(float x[6]) {std::copy(x, x+6, std::begin(mass));}
	void setDc(float x[6][3]) {setNx3(x, dc, 6);}
	void setD_(float x[6][3][3]) {setNx3x3(x, D_, 6);}
	void setAll(float dqx[], float ddqx[], float Tx[][4][4], float v0[], float w0[], float dv0[], float dw0[], float f6[], float n6[]) 
	{
		std::copy(dqx, dqx + 6, std::begin(dq));
		std::copy(ddqx, ddqx + 6, std::begin(ddq));	
		setNx4x4(Tx, T, 6);
		reassignVec3(v0, v, 0);
		reassignVec3(w0, w, 0);
		reassignVec3(dv0, dv, 0);
		reassignVec3(dw0, dw, 0);
		reassignVec3(f6, f, 6);
		reassignVec3(n6, n, 6);
	}

	// Get functions
	void getTau(float x[6]){ std::copy(tau, tau + 6, x); }

	// Calculate tau
	void calculateID()
	{
		for (int i = 1; i < 6; i++)
		{
			z[i - 1][0] = T[i - 1][0][2];
			z[i - 1][1] = T[i - 1][1][2];
			z[i - 1][2] = T[i - 1][2][2];

			w[i][0] = w[i - 1][0] + dq[i] * z[i - 1][0];
			w[i][1] = w[i - 1][1] + dq[i] * z[i - 1][1];
			w[i][2] = w[i - 1][2] + dq[i] * z[i - 1][2];

			float dqz[3] = { dq[i] * z[i - 1][0] , dq[i] * z[i - 1][1] ,dq[i] * z[i - 1][2] };
			float crosswdqz[3];
			float wHelp1[3];
			assignVec3(w, wHelp1, i - 1);
			crossProduct(wHelp1, dqz, crosswdqz);

			dw[i][0] = dw[i - 1][0] + ddq[i] * z[i - 1][0] + crosswdqz[0];
			dw[i][1] = dw[i - 1][1] + ddq[i] * z[i - 1][1] + crosswdqz[1];
			dw[i][2] = dw[i - 1][2] + ddq[i] * z[i - 1][2] + crosswdqz[2];

			ds[i][0] = T[i][0][3] - T[i - 1][0][3];
			ds[i][1] = T[i][1][3] - T[i - 1][1][3];
			ds[i][2] = T[i][2][3] - T[i - 1][2][3];

			float crosswds[3];
			float dsHelp[3];
			float wHelp2[3];
			assignVec3(w, wHelp2, i);
			assignVec3(ds, dsHelp, i);
			crossProduct(wHelp2, dsHelp, crosswds);
			float crosswcrosswds[3];
			crossProduct(wHelp2, crosswds, crosswcrosswds);
			float dwHelp[3];
			assignVec3(dw, dwHelp, i);
			float crossdwds[3];
			crossProduct(dwHelp, dsHelp, crossdwds);

			dv[i][0] = dv[i - 1][0] + crossdwds[0] + crosswcrosswds[0];
			dv[i][1] = dv[i - 1][1] + crossdwds[1] + crosswcrosswds[1];
			dv[i][2] = dv[i - 1][2] + crossdwds[2] + crosswcrosswds[2];

		}
		for (int i = 5; i > 0; i--)
		{
			dr[i][0] = T[i][0][0] * dc[i][0] + T[i][0][1] * dc[i][1] + T[i][0][2] * dc[i][2];
			dr[i][1] = T[i][1][0] * dc[i][0] + T[i][1][1] * dc[i][1] + T[i][1][2] * dc[i][2];
			dr[i][2] = T[i][2][0] * dc[i][0] + T[i][2][1] * dc[i][1] + T[i][2][2] * dc[i][2];

			float drHelp[3];
			assignVec3(dr, drHelp, i);
			float dwHelp[3];
			assignVec3(dw, dwHelp, i);
			float crossdwdr[3];
			crossProduct(dwHelp, drHelp, crossdwdr);
			float crossdwcrossdwdr[3];
			crossProduct(dwHelp, crossdwdr, crossdwcrossdwdr);

			f[i][0] = f[i + 1][0] + mass[i] * (dv[i][0] + crossdwdr[0]) + crossdwcrossdwdr[0];
			f[i][1] = f[i + 1][1] + mass[i] * (dv[i][1] + crossdwdr[1]) + crossdwcrossdwdr[1];
			f[i][2] = f[i + 1][2] + mass[i] * (dv[i][2] + crossdwdr[2]) + crossdwcrossdwdr[2];

			float D_help[3][3];
			assignMtx3x3(D_, D_help, i);
			float Rhelp[3][3];
			assignMtx4x4(T, Rhelp, i);
			float Rt[3][3];
			transpose(Rhelp, Rt);
			float D_Rt[3][3];
			mtxMul3x3(D_help, Rt, D_Rt);
			float RD_Rt[3][3];
			mtxMul3x3(Rhelp, D_Rt, RD_Rt);
			reassignMtx3x3(RD_Rt, D, i);

			float Dw[3];
			Dw[0] = D[i][0][0] * w[i][0] + D[i][0][1] * w[i][1] + D[i][0][2] * w[i][2];
			Dw[1] = D[i][1][0] * w[i][0] + D[i][1][1] * w[i][1] + D[i][1][2] * w[i][2];
			Dw[2] = D[i][2][0] * w[i][0] + D[i][2][1] * w[i][1] + D[i][2][2] * w[i][2];
			float crossdwDw[3];
			crossProduct(dwHelp, Dw, crossdwDw);
			float Ddw[3];
			Ddw[0] = D[i][0][0] * dw[i][0] + D[i][0][1] * dw[i][1] + D[i][0][2] * dw[i][2];
			Ddw[1] = D[i][1][0] * dw[i][0] + D[i][1][1] * dw[i][1] + D[i][1][2] * dw[i][2];
			Ddw[2] = D[i][2][0] * dw[i][0] + D[i][2][1] * dw[i][1] + D[i][2][2] * dw[i][2];
			float fHelp2[3];
			float fHelp1[3];
			assignVec3(f, fHelp1, i);
			assignVec3(f, fHelp2, i + 1);
			float dsHelp[3];
			assignVec3(ds, dsHelp, i);
			float crossdrf2[3];
			crossProduct(drHelp, fHelp2, crossdrf2);
			float dsdr[3];
			dsdr[0] = dsHelp[0] + drHelp[0];
			dsdr[1] = dsHelp[1] + drHelp[1];
			dsdr[2] = dsHelp[2] + drHelp[2];
			float crossdsdrf1[3];
			crossProduct(dsdr, fHelp1, crossdsdrf1);
			n[i][0] = n[i + 1][0] + crossdsdrf1[0] - crossdrf2[0] + Ddw[0] + crossdwDw[0];
			n[i][1] = n[i + 1][1] + crossdsdrf1[1] - crossdrf2[1] + Ddw[1] + crossdwDw[1];
			n[i][2] = n[i + 1][2] + crossdsdrf1[2] - crossdrf2[2] + Ddw[2] + crossdwDw[2];

			tau[i] = n[i][0] * z[i - 1][0] + n[i][1] * z[i - 1][1] + n[i][2] * z[i - 1][2];
		}
	}

private:

	// Constants
	float mass[6];
	float dc[6][3];
	float D_[6][3][3];

	// Variables
	float dq[6];
	float ddq[6];
	float T[6][4][4];

	float v[6][3];
	float w[6][3];
	float dv[6][3];
	float dw[6][3];

	float z[5][3];
	float ds[6][3];

	float f[7][3];
	float n[7][3];
	float tau[6] = {0, 0, 0, 0, 0, 0};

	float dr[6][3];
	float D[6][3][3];

};