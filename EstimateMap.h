
#ifndef _ESTIMATEMAP_H_
#define _ESTIMATEMAP_H_

#include <opencv2\core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <Eigen\dense>
#include <iostream>
#include <vector>

#define WITH_VIZ
#ifdef WITH_VIZ
#include <opencv2\viz\vizcore.hpp>
#endif

using namespace Eigen;

#define NUMLSINIT 5	// only 2, history file does not correspont with over 3 (inside estimateLS is OK)

enum FpState{
	INIT,
	RLS,
	EKF,
	NONE,
};


enum OriDataMode{
	EULER,
	QUATERNION,
};

typedef struct Fp3D{
	Vector3d pos;
	Matrix3d P;
	int color[3];	// R G B
}Fp3D;

class EstimateMap{
private:
	// Define Mode(Euler / Quaternion)
	int oriDataMode;

	// Camera intransic
	MatrixXd cameraIntrinsic;
	double fx, fy, cx, cy;

	// function
	MatrixXd createRotationMatrix(VectorXd state);

public:
	EstimateMap();
	~EstimateMap();

	// Feature point tracking History
	std::vector<std::vector<cv::Point2f>> *fp2dHist;	// 2D feature point History
	std::vector<int> fpState;

	// Map Data
	std::vector<Fp3D> fp3DRLS;		// 3D feature point during Recursive Least Square
	std::vector<cv::Mat> fpProbRLS;	// Probability matrix of feature point during Recursive Least Square
	std::vector<Fp3D> fp3DEKF;		// 3D feature point during Recursive Least Square
	std::vector<cv::Mat> fpProbEKF;	// Probability matrix of feature point during EKF
	std::vector<Fp3D> fp3DFix;		// 3D feature point during EKF
	std::vector<cv::Mat> fpProbFix;	// Probability matrix of feature point after EKF
	std::vector<VectorXd> posHist;	// Vehicle position History


	// Varialbes
	void EstimateLS2(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp, VectorXd prePos, VectorXd preOri, VectorXd curPos, VectorXd curOri);
	void EstimateLSm(std::vector<std::vector<cv::Point2f>> fp2DInit, std::vector<VectorXd> stateHist);
	void EstimateRLS();
	void EstimateNpt();	// Estimate probability of N point for trajectory generation

	void erase3DRLS(int);	// to keep the sonsitency with 2d tracking
	void erase3DEKF(int);	// to keep the sonsitency with 2d tracking

	void setCameraIntrinsic(MatrixXd receivedData);
	void setfp2dHistPointer(std::vector<std::vector<cv::Point2f>> *manageFpHist);
	void decideFpState();
#ifdef WITH_VIZ
	cv::viz::Viz3d mapWindow;
	void initMapWindow();
	void drawMap();
#endif
};

#endif
