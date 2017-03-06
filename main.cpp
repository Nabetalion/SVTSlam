#include <iostream>
#include "SVTSlam.h"
#include "LoadData.h"
#include "Simulate.h"

double computeError(VectorXd truePos, VectorXd estPos){
	double err = 0.0;

	err = sqrt((truePos(0) - estPos(0))*(truePos(0) - estPos(0)) +
		(truePos(1) - estPos(1))*(truePos(1) - estPos(1)) +
		(truePos(2) - estPos(2))*(truePos(2) - estPos(2)));

	return err;
}

int main(int argc, char** argv)
{
	SVTSlam svtSlam;
	LoadData loadData;
	Simulate simulate;

	// Create IMU Data
	//simulate.emulateIMU(KITTI);
	//simulate.simulateForCK();
	//return 0;

	// Window
	cv::namedWindow("CapImage", cv::WINDOW_AUTOSIZE);// Create a window for display.
	cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);// Create a window for display.

	// Define Variables
	cv::Mat capImage;
	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
	static int id = 1;
	double time;

	// Initilize
	loadData.selDataSource(KITTI);
	loadData.loadUniqueData();
	std::cout << "Camera :" << std::endl << loadData.CameraIntrinsic << std::endl;
	svtSlam.setCameraIntrinsic(loadData.CameraIntrinsic);

	while(1){
		std::cout << "Index :" << id << "\t";
		time = loadData.loadTime();
		std::cout <<"Time  :" << time << std::endl;

		// ----------------------------------------------------------
		// Sensor GPS
		// Load Image
		if (loadData.loadInputMode() == KITTI){
			capImage = loadData.loadImage();
		}
		else{	// Camera Mode (Under construction)

		}
		svtSlam.setImage(capImage);

		// Sensor IMU
		ImuData imuData;
		imuData = loadData.loadImu();
		svtSlam.setIMU(imuData);

		// Sensor GPS
		MatrixXd poseMatrix(3, 4);
		poseMatrix = loadData.loadPose();
		VectorXd truePos(3);
		truePos(0) = poseMatrix(0, 3);
		truePos(1) = poseMatrix(1, 3);
		truePos(2) = poseMatrix(2, 3);

		svtSlam.setGPS(truePos);

		svtSlam.setPose(poseMatrix);	// Temporary code until completing to create state estimation

		// ----------------------------------------------------------
		// Sparse Visual Tracking SLAM
		svtSlam.update(time);


		// ----------------------------------------------------------
		// Draw result
		{
			int x = int(truePos(0)) + 300;
			int y = int(truePos(2)) + 100;
			cv::circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 1);
		}
		{
			//int x = int(svtSlam.state(0)) + 300;
			//int y = int(svtSlam.state(2)) + 100;
			int x = int(svtSlam.estimateState.pos(0)) + 300;
			int y = int(svtSlam.estimateState.pos(2)) + 100;
			cv::circle(traj, cv::Point(x, y), 1, CV_RGB(0, 0, 255), 1);
			//std::cout << x << "\t" << y << std::endl;
		}
		double posErr = computeError(truePos, svtSlam.state.block(0,0,3,1));
		//std::cout << "PosErr" << posErr << std::endl;
		char text[100];
		cv::Point textOrg(10, 50);
		int fontFace = cv::FONT_HERSHEY_PLAIN;
		double fontScale = 1;
		int thickness = 1;
		sprintf(text, "Error: %02fm", posErr);
		putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

		// DrawImage
		cv::imshow("CapImage", capImage);
		cv::imshow("Map", traj);
		//if (id>10)
		//	cv::imshow("DenseOptFlow", svtSlam.estimateMap.optfImg);
		cv::waitKey(1);

		id++;
	}

	return 0;
}