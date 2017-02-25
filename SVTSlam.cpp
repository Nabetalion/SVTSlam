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
	
}

SVTSlam::~SVTSlam(){

}

void SVTSlam::setCameraIntrinsic(MatrixXd receivedData){
	this->cameraIntrinsic = receivedData;
	fx = cameraIntrinsic(0, 0);
	fy = cameraIntrinsic(1, 1);
	cx = cameraIntrinsic(0, 2);
	cy = cameraIntrinsic(1, 2);
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

	// Estimate map
	estimateLS(manageFp.fpPreLS,manageFp.fpLS,this->state,this->state);
}

void SVTSlam::estimateLS2pt(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd preState, VectorXd curState){
	Matrix3d preRot,curRot;

	// Create Rot Matrix
	switch (oriDataMode){
	case EULER:
		preRot = AngleAxisd(preState(3), Vector3d::UnitX())
			* AngleAxisd(preState(4), Vector3d::UnitY())
			* AngleAxisd(preState(5), Vector3d::UnitZ());
		curRot = AngleAxisd(curState(3), Vector3d::UnitX())
			* AngleAxisd(curState(4), Vector3d::UnitY())
			* AngleAxisd(curState(5), Vector3d::UnitZ());
		//std::cout << curRot << std::endl;
		break;
	case QUATERNION:
	default:
		// Under construction
		break;
	}

	MatrixXd A(NUMLSINIT * 2, 3);
	VectorXd B(NUMLSINIT * 2);
	
	double ud1, vd1, ud2, vd2;
	VectorXd preT = preRot*preState.block(0, 0, 3, 1);
	VectorXd curT = curRot*curState.block(0, 0, 3, 1);
	MatrixXd AT, ATA, ATAI;
	MatrixXd NN = MatrixXd::Zero(4, 4);
	Fp3D computed3Ddata;

	fp3DRLS.clear();
	for (int i = 0; i < preFp.size(); ++i){
		ud1 = preFp[i].x - cx;
		vd1 = preFp[i].y - cy;
		B(0) = preT(0)*fx - preT(2)*ud1;
		A(0, 0) = preRot(0, 0)*fx - preRot(2, 0)*ud1;
		A(0, 1) = preRot(0, 1)*fx - preRot(2, 1)*ud1;
		A(0, 2) = preRot(0, 2)*fx - preRot(2, 2)*ud1;
		B(1) = preT(1)*fy - preT(2)*vd1;
		A(1, 0) = preRot(1, 0)*fy - preRot(2, 0)*vd1;
		A(1, 1) = preRot(1, 1)*fy - preRot(2, 1)*vd1;
		A(1, 2) = preRot(1, 2)*fy - preRot(2, 2)*vd1;

		ud2 = curFp[i].x - cx;
		vd2 = curFp[i].y - cy;
		B(2) = curT(0)*fx - curT(2)*ud2;
		A(2, 0) = curRot(0, 0)*fx - curRot(2, 0)*ud2;
		A(2, 1) = curRot(0, 1)*fx - curRot(2, 1)*ud2;
		A(2, 2) = curRot(0, 2)*fx - curRot(2, 2)*ud2;
		B(3) = curT(1)*fy - curT(2)*vd2;
		A(3, 0) = curRot(1, 0)*fy - curRot(2, 0)*vd2;
		A(3, 1) = curRot(1, 1)*fy - curRot(2, 1)*vd2;
		A(3, 2) = curRot(1, 2)*fy - curRot(2, 2)*vd2;

		// Least square estimation: position
		AT   = A.transpose();
		ATA  = AT*A;
		ATAI = ATA.inverse();
		computed3Ddata.pos = ATAI*AT*B;

		// Least square estimation: covariance
		NN(0, 0) = 1;	// Noise of u of pre		// NN: other element must be zero
		NN(1, 1) = 1;	// Noise of v of pre
		NN(2, 2) = 1;	// Noise of u of cur
		NN(3, 3) = 1;	// Noise of v of cur
		computed3Ddata.P = ATAI*AT*NN*A*ATAI;	// P matrix computed by Least square

		fp3DRLS.push_back(computed3Ddata);
		//std::cout << computed3Ddata.pos << std::endl;
		//std::cout << computed3Ddata.P << std::endl;
	}
	std::cout << fp3DRLS.size() << std::endl;
}

void SVTSlam::estimateLS(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd preState, VectorXd curState){
}

// Temporary code until completing to create state estimation
void SVTSlam::setPose(MatrixXd poseData){
	pose = poseData;

	Matrix3d rot;
	rot = poseData.block(0, 0, 3, 3);

	//std::cout << rot << std::endl;

	attitude = rot.eulerAngles(0, 1, 2);
}