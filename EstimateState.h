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

	// Received Data
	VectorXd gps;
	VectorXd pos;
	VectorXd attitude;

//	int kindOfReveivedData;
public:
	EstimateState();
	~EstimateState();

	// Variables
	VectorXd state;

	// Functions
	void estimate();
	void propagate();
	void update();

	// Set Date = Receive data
	void setGps(VectorXd);
	void setAttitude(VectorXd);

	//
	int getOriMode();

};

#endif
