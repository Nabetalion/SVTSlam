#include <iostream>
#include "EstimateMap.h"

EstimateMap::EstimateMap(){
	oriDataMode = MAPORI_EULER;

	R_5pt = Matrix3d::Identity(3,3);
	t_5pt = Vector3d::Zero(3);
}

EstimateMap::~EstimateMap(){

}


MatrixXd EstimateMap::createRotationMatrix(VectorXd state){
	Matrix3d rot;

	// Create Rot Matrix
	switch (oriDataMode){
	case MAPORI_EULER:
		rot = AngleAxisd(state(3), Vector3d::UnitX())
			* AngleAxisd(state(4), Vector3d::UnitY())
			* AngleAxisd(state(5), Vector3d::UnitZ());
		break;
	case MAPORI_QUATERNION:
	default:
		// Under construction
		break;
	}

	return rot;
}

// http://ishidate.my.coocan.jp/opencv310_10/opencv310_10.htm
void EstimateMap::detectFp(cv::Mat img){
	std::vector<cv::Point2f> newFp;
	int numDetectFp = MAXMANAGEDFP - (fpLS.size() + fpRLS.size() + fpEKF.size());


	if (numDetectFp < 1){

	}
	else{
		cv::Mat grayImg;
		cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

#ifdef GOOD_FEATURE
		goodFeaturesToTrack(grayImg, newFp, MAXDETECTFP, 0.0001, 30);
		/// Set the neeed parameters to find the refined corners
		cv::Size winSize = cv::Size(5, 5);
		cv::Size zeroZone = cv::Size(-1, -1);
		cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
		cornerSubPix(grayImg, newFp, winSize, zeroZone, criteria);

		int rndIdx;
		for (int i = 0; i < numDetectFp; i++){
			rndIdx = rand() % newFp.size();
			fpLS.push_back(newFp[rndIdx]);
		}
#endif
	}
	/*
	int fast_threshold = 20;
	bool nonmaxSuppression = true;
	//#if OPENCV24X
	std::vector<cv::KeyPoint> points;
	//#endif
	cv::Mat grayImg;
	cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);
	//cv::FAST(grayImg, points, fast_threshold, nonmaxSuppression);
	cv::AGAST(grayImg, points, fast_threshold, nonmaxSuppression);
	cv::KeyPoint::convert(points, fpLS2);
	*/
}

void EstimateMap::TrackingAndDetectFp(cv::Mat currImg){
	std::vector<cv::Point2f> optFrowResult;

	if (preImg.empty()){		// First time
		preImg = std::move(currImg);
	}
	else{
		if (fpLS.size() + fpRLS.size() == 0){
			// No tracking
		}
		else{
			std::vector<float> err;
			cv::Size winSize = cv::Size(21, 21);
			cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

			cv::calcOpticalFlowPyrLK(preImg, currImg, fpLS, optFrowResult, optflowStatus, err);

			// Reverse pose
			cv::Mat R, t;
			cv::Mat mask;
			cv::Mat essentialMat;
			essentialMat = cv::findEssentialMat(optFrowResult, fpLS, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
			recoverPose(essentialMat, optFrowResult, fpLS, R, t, focal, pp, mask);

			// Convert cv::Mat to Eigen
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++){
					R_5pt(i, j) = R.at<double>(i, j);
				}
			}
			for (int i = 0; i < 3; i++){
				t_5pt(i) = t.at<double>(i);
			}
			//std::cout << "5Pt result" << std::endl;
			//std::cout << R_5pt << std::endl;
			//std::cout << t_5pt << std::endl;

			// Filter: getting rid of points for which the KLT tracking failed or those who have gone outside the frame
			int indexCorrection = 0;
			for (int i = 0; i<optflowStatus.size(); i++)
			{
				cv::Point2f pt = optFrowResult.at(i - indexCorrection);
				if ((optflowStatus.at(i) == 0) || (pt.x<0) || (pt.y<0) || (pt.x>currImg.cols) || (pt.y>currImg.rows))	{
					if ((pt.x<0) || (pt.y<0) || (pt.x>currImg.cols) || (pt.y>currImg.rows))	{
						optflowStatus.at(i) = 0;
					}
					optFrowResult.erase(optFrowResult.begin() + (i - indexCorrection));
					fpLS.erase(fpLS.begin() + (i - indexCorrection));
					fp2dHist.erase(fp2dHist.begin() + (i - indexCorrection));

					indexCorrection++;
				}

			}

		}

		// Post process
		fpPreLS = fpLS;
		fpLS = optFrowResult;


		// Filling up feature points
		detectFp(currImg);

		std::cout << "-----------------------------" << std::endl;

		// Write down hisotory
		for (int i = 0; i < fpLS.size(); i++){
			// Size over is new point
			if (i >= fp2dHist.size()){
				fp2dHist.push_back(std::vector<cv::Point2f>(0));
				fp2dHist.back().push_back(fpLS[i]);
			}
			else{		// Have past history
				fp2dHist[i].push_back(fpLS[i]);
			}
		}
		//std::cout << fpLS.size() << std::endl;
		/*std::cout << "Hist " << fp2dHist.size() << std::endl;
		for (int i = 0; i < fp2dHist.size(); i++){
		std::cout << fp2dHist[i].size() << "\t";
		}
		std::cout << std::endl;
		*/

		preImg = std::move(currImg);
	}
}

void EstimateMap::EstimateLS2(std::vector<cv::Point2f> preFp, std::vector<cv::Point2f> curFp,
							VectorXd prePos, VectorXd preOri, VectorXd curPos, VectorXd curOri){

	MatrixXd preRot = createRotationMatrix(preOri);
	MatrixXd curRot = createRotationMatrix(curOri);

	MatrixXd A(2 * 2, 3);	// from 2 view and 2 measurement, 3 dimension
	VectorXd B(2 * 2);		// from 2 view and 2 measurement

	double ud1, vd1, ud2, vd2;
	VectorXd preT = preRot*prePos;	// previous Translation vector in terms of camera frame
	VectorXd curT = curRot*curPos;	// current Translation vector in terms of camera frame
	MatrixXd AT, ATA, ATAI;
	MatrixXd NN = MatrixXd::Zero(4, 4);
	Fp3D computed3Ddata;

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
		AT = A.transpose();
		ATA = AT*A;
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
void EstimateMap::EstimateLSm(std::vector<std::vector<cv::Point2f>> fp2DInit, std::vector<VectorXd> stateHist){
	MatrixXd rot[NUMLSINIT];	// 0 is current and data is back to state with order i.
	VectorXd pos[NUMLSINIT];	// 0 is current and data is back to state with order i.
	VectorXd tc[NUMLSINIT];	// trans vector in camera frame. 0 is current and data is back to state with order i.
	VectorXd ori;
	int sizeStateHist = stateHist.size();
	for (int i = 0; i < NUMLSINIT; i++){
		ori = stateHist[sizeStateHist - 1 - i].block(3, 0, 3, 1);	// If Euler
		rot[i] = createRotationMatrix(ori);
		pos[i] = stateHist[sizeStateHist - 1 - i].block(0, 0, 3, 1);
		tc[i] = rot[i] * pos[i];
	}

	MatrixXd A(NUMLSINIT * 2, 3);	// from Multiple view and 2 measurement, 3 dimension
	VectorXd B(NUMLSINIT * 2);		// from Multiple view and 2 measurement
	MatrixXd AT, ATA, ATAI;
	MatrixXd NN = MatrixXd::Zero(2 * NUMLSINIT, 2 * NUMLSINIT);
	double ud[NUMLSINIT], vd[NUMLSINIT];
	Fp3D computed3Ddata;

	for (int i = 0; i < fp2DInit.size(); i++){
		if (fp2DInit[i].size() >= NUMLSINIT){


		}
	}

}

void EstimateMap::EstimateRLS(){

}

void EstimateMap::EstimateNpt(){


}

// to keep the sonsitency with 2d tracking
void EstimateMap::erase3DRLS(int eraseId){
	fp3DRLS.erase(fp3DRLS.begin() + eraseId);
}
// to keep the sonsitency with 2d tracking
void EstimateMap::erase3DEKF(int eraseId){
	fp3DEKF.erase(fp3DEKF.begin() + eraseId);
}

void EstimateMap::setCameraIntrinsic(MatrixXd receivedData){
	this->cameraIntrinsic = receivedData;
	fx = cameraIntrinsic(0, 0);
	fy = cameraIntrinsic(1, 1);
	cx = cameraIntrinsic(0, 2);
	cy = cameraIntrinsic(1, 2);

	focal = 718.8560;
	pp = cv::Point2d(607.1928, 185.2157);
}


void EstimateMap::decideFpState(){
	fpState.clear();
	for (int i = 0; i < fp2dHist.size(); i++){
		if (fp2dHist[i].size()<NUMLSINIT){
			fpState.push_back(INIT);
		}
		else if (fp2dHist[i].size()<NUMLSINIT + 1){
			fpState.push_back(RLS);
		}
		else if (fp2dHist[i].size()>=NUMLSINIT + 1){
			fpState.push_back(EKF);
		}
		else{
			fpState.push_back(NONE);
		}
	}
	/*
	for (int i = 0; i < fpState.size(); i++){
		std::cout << fpState[i] << "\t";
	}
	std::cout << std::endl;
	*/
}

void EstimateMap::DrawFp(cv::Mat img){
	int radius = cvRound(5);

	for (int i = 0; i < fpLS.size(); i++){
		cv::circle(img, fpLS[i], radius, cv::Scalar(255, 0, 255));
	}
	//cv::drawKeypoints(img, fpLS2, img, cv::Scalar(0, 0, 255));
}



void EstimateMap::DrawTracking(cv::Mat img){
	for (size_t i = 0; i<fpPreLS.size(); i++){
		if (optflowStatus[i]){
			cv::line(img, fpPreLS[i], fpLS[i], cv::Scalar(0, 0, 255));
		}
	}
}


#ifdef WITH_VIZ
void EstimateMap::initMapWindow(){
	mapWindow = cv::viz::Viz3d("MapResult");
	//mapWindow.spin();
	// Add coordinate axes
	mapWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
}
void EstimateMap::drawMap(){
	//http://docs.opencv.org/2.4/doc/tutorials/viz/table_of_content_viz/table_of_content_viz.html
	mapWindow.spinOnce(1, true);
}
#endif
