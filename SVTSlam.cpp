#include <iostream>
#include "SVTSlam.h"

SVTSlam::SVTSlam(){

	state = VectorXd(3);


}

SVTSlam::~SVTSlam(){

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

	// Estimate State
	//estimateState.estimate();
	this->state=this->gpsData;
	//std::cout << state << std::endl;

	// Estimate map

}