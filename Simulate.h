
#ifndef _SIMULATE_H_
#define _SIMULATE_H_

#include <iostream>
#include <vector>

#include <Eigen/Dense>

//#define MAX_DATA 40000
#define MAX_DATA 64

using namespace Eigen;

class Simulate{
private:
	std::vector<VectorXd> pos;	// position w.r.t. inertia frame
	std::vector<Vector3d> ori;	// Euler angle w.r.t. inertia frame
	std::vector<Vector3d> vel;	// Euler angle w.r.t. inertia frame
	std::vector<double> time;

	std::vector<VectorXd> acc_I;
	std::vector<VectorXd> avel_B;
	std::vector<VectorXd> acc_B;

	std::vector<Vector3d> simPos;
	std::vector<Vector3d> simVel;
	std::vector<Vector3d> simOri;
public:
	Simulate();
	~Simulate();
	
	void emulateIMU(int);

	void simulateForCK();
};

#endif
