
#ifndef _SVTSLAM_H_
#define _SVTSLAM_H_

#include "iostream"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "Eigen\Dense"
//#include "Eigen\geometory"

#include "ManageFP.h"
#include "EstimateState.h"
#include "EstimateMap.h"

using namespace Eigen;



#define NUMSTATE 9	// x,y,z, phi,th,psi, u,v,w (,bias a x3 ,bias w x3)



class SVTSlam{
private:
	// Define Class
	ManageFp manageFp;
	EstimateState estimateState;
	//EstimateMap estimateMap;

	// Sensor Data
	cv::Mat image;
	ImuData imuData;
	VectorXd gpsData;

	//void estimateLSm(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd preState, VectorXd curState);
	//void EstimateRLS();
	//void EstimateNpt();	// Estimate probability of N point for trajectory generation


public:
	SVTSlam();
	~SVTSlam();

	EstimateMap estimateMap;

	// Variables
	VectorXd state;

	// Functions
	void setCameraIntrinsic(MatrixXd);
	void setImage(cv::Mat);
	void setIMU(ImuData);
	void setGPS(VectorXd);
	void update();

	// tempcode
	MatrixXd pose;
	Vector3d attitude;
	void setPose(MatrixXd);	// Temporary code until completing to create state estimation
};

#endif
