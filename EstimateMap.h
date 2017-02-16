
#ifndef _ESTIMATEMAP_H_
#define _ESTIMATEMAP_H_

#include <opencv2\core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <Eigen\dense>
#include <iostream>
#include <vector>

using namespace Eigen;

typedef struct Fp3D{
	Vector3d pos;
	Matrix3d P;
}Fp3D;


class EstimateMap{
private:
	std::vector<Fp3D> fp3DRLS;
	std::vector<Fp3D> fp3DEKF;
public:
	EstimateMap();
	~EstimateMap();

	void EstimateLS(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, MatrixXd prePose, MatrixXd curPose);
	void EstimateRLS();	
	void EstimateNpt();	// Estimate probability of N point for trajectory generation

	void erase3DRLS(int);	// to keep the sonsitency with 2d tracking
	void erase3DEKF(int);	// to keep the sonsitency with 2d tracking
};

#endif
