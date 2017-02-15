#include <iostream>
#include "LoadData.h"

LoadData::LoadData(int mode){
	inputMode = mode;
	id = 1;
}

LoadData::~LoadData(){

}

int LoadData::loadInputMode(){
	return inputMode;
}


cv::Mat LoadData::loadKITTI(){
	cv::Mat loadImg;

	char filename[100];
	sprintf(filename, "E:/dataset/KITTI/sequences/00/image_2/%06d.png", id);

	loadImg = cv::imread(filename);

	id++;

	return loadImg;
}