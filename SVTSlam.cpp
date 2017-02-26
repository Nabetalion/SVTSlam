#include <iostream>
#include "SVTSlam.h"

SVTSlam::SVTSlam(){
	oriDataMode = EULER;

	switch (oriDataMode){
	case EULER:
		state = VectorXd(9);	// pos,ori,vel
		break;
	case QUATERNION:
		state = VectorXd(10);	// pos,ori,vel
	default:
		break;
	}
#ifdef WITH_VIZ
	estimateMap.initMapWindow();
#endif
}

SVTSlam::~SVTSlam(){

}

void SVTSlam::setCameraIntrinsic(MatrixXd receivedData){
	this->estimateMap.setCameraIntrinsic(receivedData);
}

void SVTSlam::setImage(cv::Mat receivedImage){
	this->image = receivedImage;
}

void SVTSlam::setIMU(ImuData receivedImuData){
	this->imuData = receivedImuData;
}

void SVTSlam::setGPS(VectorXd receivedGPSData){
	this->gpsData = receivedGPSData;
}


void SVTSlam::update(){
	// Extract Feature Point
	//manageFp.detectFp(image);	// Detect
	//manageFp.DrawFp(image);		// Draw FP

	// Tracking
	//manageFp.Tracking(image);

	//
	manageFp.TrackingAndDetectFp(image);
	manageFp.DrawFp(image);
	manageFp.DrawTracking(image);

	// 5 pt

	// Estimate State
	//estimateState.estimate();
	this->state(0) = this->gpsData(0);
	this->state(1) = this->gpsData(1);
	this->state(2) = this->gpsData(2);
	this->state(3) = this->attitude(0);
	this->state(4) = this->attitude(1);
	this->state(5) = this->attitude(2);
	//std::cout << state << std::endl;

	/*static std::vector<std::vector<cv::Point2f>> test;
	test.push_back(std::vector<cv::Point2f>(0));
	std::cout << test.size() << std::endl;
	std::cout << test.back().size() << std::endl;
	test.back().push_back(1);
	test.back().push_back(2);
	test.back().push_back(3);
	test.back().push_back(4);
	std::cout << test.size() << std::endl;
	std::cout << test[0].size() << std::endl;
	if (test.size() > 10){
		std::cout << test[0][0] << std::endl;
	}*/

	// Estimate map
	estimateMap.setfp2dHistPointer(&(manageFp.fp2dHist));	// Set the pointer of 2d tracking history 
	estimateMap.decideFpState();							// to make estimation smooth, compute fp state in advance
	//estimateLS2(manageFp.fpPreLS,manageFp.fpLS,this->state,this->state);

#ifdef WITH_VIZ
	estimateMap.drawMap();
#endif
}

void SVTSlam::estimateLS2(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd preState, VectorXd curState){

}

void SVTSlam::estimateLSm(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd preState, VectorXd curState){
}

// Temporary code until completing to create state estimation
void SVTSlam::setPose(MatrixXd poseData){
	pose = poseData;

	Matrix3d rot;
	rot = poseData.block(0, 0, 3, 3);

	//std::cout << rot << std::endl;

	attitude = rot.eulerAngles(0, 1, 2);
}