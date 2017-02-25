
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
//#include "EstimateMap.h"

using namespace Eigen;

enum OriDataMode{
	EULER,
	QUATERNION,
};

#define NUMSTATE 3
#define NUMLSINIT 2	// only 2, history file does not correspont with over 3 (inside estimateLS is OK)


typedef struct ImuData{
	Vector3d imu;
	Vector3d gyro;
}ImuData;

typedef struct Fp3D{
	Vector3d pos;
	Matrix3d P;
	int color[3];	// R G B
}Fp3D;

class SVTSlam{
private:
	// Define Class
	ManageFp manageFp;
	EstimateState estimateState;
	//EstimateMap estimateMap;

	// Camera intransic
	MatrixXd cameraIntrinsic;
	double fx, fy, cx, cy;

	// Define Mode(Euler / Quaternion)
	int oriDataMode;

	// Define State
	VectorXd state;

	// Sensor Data
	cv::Mat image;
	ImuData imuData;
	VectorXd gpsData;

	void estimateLS2pt(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd preState, VectorXd curState);
	void estimateLS(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd preState, VectorXd curState);
	//void EstimateRLS();
	//void EstimateNpt();	// Estimate probability of N point for trajectory generation
	
public:
	SVTSlam();
	~SVTSlam();

	// Map Data
	std::vector<std::vector<cv::Point2f>> fp3DInit;	// 2D feature point for Initial Estimation
	std::vector<Fp3D> fp3DRLS;		// 3D feature point during Recursive Least Square
	std::vector<cv::Mat> fpProbRLS;	// Probability matrix of feature point during Recursive Least Square
	std::vector<Fp3D> fp3DEKF;		// 3D feature point during Recursive Least Square
	std::vector<cv::Mat> fpProbEKF;	// Probability matrix of feature point during EKF
	std::vector<Fp3D> fp3DFix;		// 3D feature point during EKF
	std::vector<cv::Mat> fpProbFix;	// Probability matrix of feature point after EKF
	std::vector<VectorXd> posHist;	// Vehicle position History

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
