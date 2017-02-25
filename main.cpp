#include <iostream>
#include "SVTSlam.h"
#include "LoadData.h"


int main(int argc, char** argv)
{
	SVTSlam svtSlam;
	LoadData loadData;

	// Window
	cv::namedWindow("CapImage", cv::WINDOW_AUTOSIZE);// Create a window for display.
	cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);// Create a window for display.
	cv::namedWindow("LocalMap", cv::WINDOW_AUTOSIZE);// Create a window for display.

	// Define Variables
	cv::Mat capImage;
	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
	cv::Mat localMap = cv::Mat::zeros(600, 600, CV_8UC3);
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
		svtSlam.update();


		// ----------------------------------------------------------
		// Draw result
		int x = int(truePos(0)) + 300;
		int y = int(truePos(2)) + 100;
		cv::circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 1);

		for (int i = 0; i < svtSlam.fp3DRLS.size(); ++i){
			std::cout << svtSlam.fp3DRLS[i].pos(0) - truePos(0) << std::endl;
			x = (svtSlam.fp3DRLS[i].pos(0) - truePos(0)) * 1000 + 300;
			y = (svtSlam.fp3DRLS[i].pos(2) - truePos(2)) * 1000 + 300;
			cv::circle(localMap, cv::Point(x, y), 1, CV_RGB(0, 0, 255), 1);
		}

		//std::cout << truePos << std::endl;
		// DrawImage
		cv::imshow("CapImage", capImage);
		cv::imshow("Map", traj); 
		cv::imshow("LocalMap", localMap); 
		cv::waitKey(1);

		localMap = cv::Mat::zeros(600, 600, CV_8UC3);

		id++;
	}

	return 0;
}