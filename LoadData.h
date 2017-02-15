
#ifndef _LOADDATA_H_
#define _LOADDATA_H_

#include <opencv2\core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2\imgproc.hpp>

#include "Eigen\Dense"

#include "SVTSlam.h"

#define KITTI 1
#define WEBCAM 2
#define EuRoC 3

using namespace Eigen;

class LoadData{
private:
	int inputMode;


	void loadCameraData();
public:
	LoadData();
	~LoadData();

	// Varialbes
	Eigen::MatrixXd CameraIntrinsic;

	// Function
	void selDataSource(int);
	int loadInputMode();
	cv::Mat loadImage();
	ImuData loadImu();
	MatrixXd loadPose();
	double loadTime();
	void loadUniqueData();
};

#endif
