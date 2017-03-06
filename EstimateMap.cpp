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
#else
		int fast_threshold = 20;
		bool nonmaxSuppression = true;
		std::vector<cv::KeyPoint> points;
		//cv::FAST(grayImg, points, fast_threshold, nonmaxSuppression);
		cv::AGAST(grayImg, points, fast_threshold, nonmaxSuppression);
		cv::KeyPoint::convert(points, newFp);
#endif
		int rndIdx;
		for (int i = 0; i < numDetectFp; i++){
			rndIdx = rand() % newFp.size();
			fpLS.push_back(newFp[rndIdx]);
		}
	}
}

//オプティカルフローを可視化する。
//縦横のベクトルの強さを色に変換する。
//左：赤、右：緑、上：青、下：黄色
void visualizeFarnebackFlow(
	const cv::Mat& flow,    //オプティカルフロー CV_32FC2
	cv::Mat& visual_flow    //可視化された画像 CV_32FC3
	)
{
	visual_flow = cv::Mat::zeros(flow.rows, flow.cols, CV_32FC3);
	int flow_ch = flow.channels();
	int vis_ch = visual_flow.channels();//3のはず
	for (int y = 0; y < flow.rows; y++) {
		float* psrc = (float*)(flow.data + flow.step * y);
		float* pdst = (float*)(visual_flow.data + visual_flow.step * y);
		for (int x = 0; x < flow.cols; x++) {
			float dx = psrc[0];
			float dy = psrc[1];
			float r = (dx < 0.0) ? abs(dx) : 0;
			float g = (dx > 0.0) ? dx : 0;
			float b = (dy < 0.0) ? abs(dy) : 0;
			r += (dy > 0.0) ? dy : 0;
			g += (dy > 0.0) ? dy : 0;

			pdst[0] = b;
			pdst[1] = g;
			pdst[2] = r;

			psrc += flow_ch;
			pdst += vis_ch;
		}
	}
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

			// Sparse optical flow by LK
			cv::calcOpticalFlowPyrLK(preImg, currImg, fpLS, optFrowResult, optflowStatus, err);

#ifdef DENSE_OPTFLOW
			// Dense optical flow by Farnback
			cv::Mat grayPreImg, grayCurrImg;
			cvtColor(preImg, grayPreImg, cv::COLOR_BGR2GRAY);
			cvtColor(currImg, grayCurrImg, cv::COLOR_BGR2GRAY);
			cv::calcOpticalFlowFarneback(grayPreImg, grayCurrImg, optfImg, 0.8, 10, 5, 3, 7, 1.5, 0);
			fpLS2.clear();
			for (int i = 0; i<fpLS.size(); ++i){
				float fpLSx = fpLS[i].x, fpLSy = fpLS[i].y;
				if (fpLS[i].x < 0){
					fpLSx = 0.0;
				}
				else if (fpLS[i].x>currImg.cols){
					fpLSx = currImg.cols-1.0;
				}
				if (fpLS[i].y < 0){
					fpLSy = 0.0;
				}
				else if (fpLS[i].x>currImg.rows){
					fpLSy = currImg.rows - 1.0;
				}
				//std::cout << "fppos:\t"<< fpLSx << "\t" << fpLSy << std::endl;
				//std::cout << optfImg.size() << std::endl;

				cv::Point2f newPtFB = { fpLS[i].x + optfImg.at<cv::Point2f>(fpLSy, fpLSx).x,
					fpLS[i].y + optfImg.at<cv::Point2f>(fpLSy, fpLSx).y };
				fpLS2.push_back(newPtFB);
			}
#endif


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
#ifdef DENSE_OPTFLOW
					fpLS2.erase(fpLS2.begin() + (i - indexCorrection));
#endif
					indexCorrection++;
				}

			}

		}

		// Post process
		fpPreLS = fpLS;
		fpLS = optFrowResult;


		// Filling up feature points
		detectFp(currImg);

		//std::cout << "-----------------------------" << std::endl;

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
void EstimateMap::EstimateLSm(std::vector<cv::Point2f> fp2DPtHist, std::vector<VectorXd> stateHist){
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

	for (int i = 0; i < fp2DPtHist.size(); i++){
		//if (fp2DInit[i].size() >= NUMLSINIT){
		ud[i] = fp2DPtHist[i].x - cx;
		vd[i] = fp2DPtHist[i].y - cy;
			/*
			B(0) = preT(0)*fx - preT(2)*ud1;
			A(0, 0) = preRot(0, 0)*fx - preRot(2, 0)*ud1;
			A(0, 1) = preRot(0, 1)*fx - preRot(2, 1)*ud1;
			A(0, 2) = preRot(0, 2)*fx - preRot(2, 2)*ud1;
			B(1) = preT(1)*fy - preT(2)*vd1;
			A(1, 0) = preRot(1, 0)*fy - preRot(2, 0)*vd1;
			A(1, 1) = preRot(1, 1)*fy - preRot(2, 1)*vd1;
			A(1, 2) = preRot(1, 2)*fy - preRot(2, 2)*vd1;
			*/
		//}
	}

}

void EstimateMap::estimateFpState(Vector3d pos, Quaterniond quat,double time){
	// Convert quat to DCM
	// http://stackoverflow.com/questions/21761909/eigen-convert-matrix3d-rotation-to-quaternion?rq=1
	// http://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
	// Note: if you update quatanion, you must normalize it
	Matrix3d rot = quat.toRotationMatrix();
	Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
	//std::cout << rot << std::endl;
	//std::cout << euler << std::endl;

	// Update State History
	CamState camStateTemp;
	camStateTemp.pos = pos;
	camStateTemp.quat = quat;
	camStateHist.push_back(camStateTemp);
	capTimeHist.push_back(time);
	if (camStateHist.size() > NUMLSINIT){
		camStateHist.erase(camStateHist.begin());
	}
	if (capTimeHist.size() > NUMLSINIT){
		capTimeHist.erase(capTimeHist.begin());
	}
	//for (int i = 0; i < capTimeHist.size(); i++){
	//	std::cout << capTimeHist[i] << "\t";
	//}
	//std::cout << std::endl;

	// Estimate fp position
	fpMode.clear();
	for (int i = 0; i < fp2dHist.size(); i++){
		if (fp2dHist[i].size()<NUMLSINIT+10000){
			fpMode.push_back(INIT);
			//EstimateLSm(fp2dHist[i], stateHist);
		}
		/*
		else if (fp2dHist[i].size()<NUMLSINIT + 1){
			fpMode.push_back(RLS);
		}
		else if (fp2dHist[i].size() >= NUMLSINIT + 1){
			fpMode.push_back(EKF);
		}
		else{
			fpState.push_back(NONE);
		}
		*/
	}
	/*
	for (int i = 0; i < fpState.size(); i++){
	std::cout << fpState[i] << "\t";
	}
	std::cout << std::endl;
	*/
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



void EstimateMap::DrawFp(cv::Mat img){
	int radius = cvRound(5);

	for (int i = 0; i < fpLS.size(); i++){
		cv::circle(img, fpLS[i], radius, cv::Scalar(255, 0, 255));
	}

}



void EstimateMap::DrawTracking(cv::Mat img){
	for (size_t i = 0; i<fpPreLS.size(); i++){
		if (optflowStatus[i]){
			cv::line(img, fpPreLS[i], fpLS[i], cv::Scalar(0, 0, 255));
		}
	}
#ifdef DENSE_OPTFLOW
	for (size_t i = 0; i<fpPreLS.size(); i++){
		if (optflowStatus[i]){
			cv::line(img, fpPreLS[i], fpLS2[i], cv::Scalar(222, 0, 0));
		}
	}
	cv::Mat visual_flow;
	//visualizeFarnebackFlow(optfImg, visual_flow);
	//std::cout << optfImg.size() << std::endl;
	//if (!visual_flow.empty())
	//	cv::imshow("DenseOptFlow", visual_flow);
#endif
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
