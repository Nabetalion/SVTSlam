// Estimation State by variable number of state EKF 

#ifndef _ESTIMATESTATE_H_
#define _ESTIMATESTATE_H_

#include <Eigen\dense>
#include <vector>

using namespace Eigen;

typedef struct ImuData{
	Vector3d imu;
	Vector3d gyro;
}ImuData;


enum OriDataMode{
	EULER,
	QUATERNION,
};

//typedef struct KindOfReceivedData{
//	unsigned char gps;
//};

class EstimateState{
private:
	// Define Mode(Euler / Quaternion)
	int oriDataMode;

	// variables
	double dt;

	// Received Data
	ImuData imu;
	VectorXd gps;
	VectorXd posSensor;
	VectorXd attitudeSensor;
	Matrix3d R_5pt;
	Vector3d t_5pt;

//	int kindOfReveivedData;
public:
	EstimateState();
	~EstimateState();

	// Variables
	VectorXd state;
	MatrixXd P;
	std::vector<VectorXd> stateHist;
	std::vector<MatrixXd> PHist;
	
	Vector3d pos;	// temporary
	Matrix3d rot;	// temporary
	double lambda;

	// Functions
	void estimate();
	void propagate();
	void update();

	// Set Date = Receive data
	void setDt(double);
	void setImu(ImuData);
	void setGps(VectorXd);
	void setAttitude(VectorXd);
	void setExtransic(Matrix3d, Vector3d);

	// for other objective
	void setState(VectorXd);	// For initialize
	void setP(MatrixXd);
	void storeIntoHist();

	//
	int getOriMode();

};

#endif
