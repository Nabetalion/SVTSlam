#include <iostream>
#include "SVTSlam.h"
#include "LoadData.h"


int main(int argc, char** argv)
{
	SVTSlam svtSlam;
	LoadData loadData;

	// Define Variables
	cv::Mat capImage;
	static int id = 1;
	double time;

	// Initilize
	loadData.selDataSource(KITTI);
	loadData.loadUniqueData();
	std::cout << "Camera :" << std::endl << loadData.CameraIntrinsic << std::endl;

	while(1){
		std::cout << "Index :" << id << "\t";
		time = loadData.loadTime();
		std::cout <<"Time  :" << time << std::endl;

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

		// Sparse Visual Tracking SLAM
		svtSlam.update();

		// DrawImage
		cv::imshow("CapImage", capImage);
		cv::waitKey(1);

		id++;
	}

	return 0;
}