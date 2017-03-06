#include <iostream>
#include "SVTSlam.h"

SVTSlam::SVTSlam(){
	switch (estimateState.getOriMode()){
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


void SVTSlam::update(double time){
	// Extract Feature Point
	//manageFp.detectFp(image);	// Detect
	//manageFp.DrawFp(image);		// Draw FP

	// Tracking
	//manageFp.Tracking(image);

	// Fp detection and 5 pt
	estimateMap.TrackingAndDetectFp(image);


	// Estimate State
	estimateState.setImu(this->imuData);
	//estimateState.propagate();
	estimateState.setGps(this->gpsData);
	estimateState.setAttitude(this->attitude);
	estimateState.setExtransic(estimateMap.R_5pt, estimateMap.t_5pt);
	estimateState.update();

	//std::cout << state << std::endl;


	// Estimate map
	// Preliminary of Estimate map
	Vector3d pos;
	Matrix3d rot;
	rot << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	Quaterniond quat(rot);
	//std::cout << "---------------------" << std::endl;
	estimateMap.estimateFpState(pos, quat,time);							// to make estimation smooth, compute fp state in advance
	// Estimate
	//estimateLS2(manageFp.fpPreLS,manageFp.fpLS,this->state,this->state);

	// Copy State of Vehicle
	this->state = estimateState.state;
	//std::cout << estimateState.state << "\n" << state << std::endl;

	// Draw Result
	estimateMap.DrawFp(image);
	estimateMap.DrawTracking(image);

#ifdef WITH_VIZ
	estimateMap.drawMap();
#endif
}




// Temporary code until completing to create state estimation
void SVTSlam::setPose(MatrixXd poseData){
	pose = poseData;

	Matrix3d rot;
	rot = poseData.block(0, 0, 3, 3);

	//std::cout << rot << std::endl;

	attitude = rot.eulerAngles(0, 1, 2);
}