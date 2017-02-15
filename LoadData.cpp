#include <iostream>
#include <fstream>
#include "LoadData.h"

LoadData::LoadData(){
	CameraIntrinsic = Eigen::MatrixXd::Identity(3, 3);
}

LoadData::~LoadData(){

}

void LoadData::selDataSource(int mode){
	inputMode = mode;
}


int LoadData::loadInputMode(){
	return inputMode;
}

void LoadData::loadCameraData(){
	std::string line;
	std::ifstream myfile("E:/dataset/KITTI/sequences_calib/00/cameraIntransic.txt");
	getline(myfile, line);

	std::istringstream in(line);
	Eigen::MatrixXd CameraIntrinsicTrans(3,3);

	double data;
	for (int j = 0; j<9; j++)  {
		in >> data;
		//vecCameraIntrinsic[];
		CameraIntrinsicTrans(j) = data;
	}
	CameraIntrinsic = CameraIntrinsicTrans.transpose();
}

void LoadData::loadUniqueData(){
	loadCameraData();
}

cv::Mat LoadData::loadImage(){
	cv::Mat loadImg;
	static int id=0;

	char filename[100];

	switch (inputMode){
	case KITTI:
		sprintf(filename, "E:/dataset/KITTI/sequences/00/image_2/%06d.png", id);

		loadImg = cv::imread(filename);
		id++;
		break;
	}


	return loadImg;
}
ImuData LoadData::loadImu(){
	ImuData imuData;
	Vector3d imu;
	Vector3d gyro;

	imu(0) = 0;
	imu(1) = 0;
	imu(2) = 0;
	gyro(0) = 0;
	gyro(1) = 0;
	gyro(2) = 0;

	imuData.imu = imu;
	imuData.gyro = gyro;

	return imuData;
}

MatrixXd LoadData::loadPose(){
	std::string line;
	MatrixXd retData(3, 4);
	static bool firstFlag = true;
	static int id = 0;

	switch (inputMode){
		case KITTI:
			static std::ifstream myfile("E:/dataset/KITTI/poses/00.txt");
			if (myfile.is_open())
			{
				getline(myfile, line);
				std::istringstream in(line);

				double data;
				in >> data;	retData(0, 0) = data;
				in >> data;	retData(0, 1) = data;
				in >> data;	retData(0, 2) = data;
				in >> data;	retData(0, 3) = data;
				in >> data;	retData(1, 0) = data;
				in >> data;	retData(1, 1) = data;
				in >> data;	retData(1, 2) = data;
				in >> data;	retData(1, 3) = data;
				in >> data;	retData(2, 0) = data;
				in >> data;	retData(2, 1) = data;
				in >> data;	retData(2, 2) = data;
				in >> data;	retData(2, 3) = data;

				/*
				for (int j = 0; j<12; j++)  {
					in >> data;
					if (j ==  3) state(0) = data;
					if (j ==  7) state(1) = data;
					if (j == 11) state(2) = data;
				}
				*/
			}

			break;
	}

	//std::cout << retData << std::endl;

	return retData;
}

double LoadData::loadTime(){
	double currTime=0.0;

	std::string line;

	switch (inputMode){
		case KITTI:
			static std::ifstream myfile("E:/dataset/KITTI/sequences_calib/00/times.txt");
			if (myfile.is_open())
			{
				getline(myfile, line);
				std::istringstream in(line);

				in >> currTime;
			}

			break;
	}

	return currTime;
}