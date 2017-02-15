#include <iostream>
#include "ManageFp.h"

ManageFp::ManageFp(){
	//detector = cv::ORB::create(ORBMAXPOINT);
	//cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
}
ManageFp::~ManageFp(){

}

#define MAXDETECTFP 100

void ManageFp::detectFp(cv::Mat img){
	std::vector<cv::Point2f> newFp;
	int numDetectFp = MAXMANAGEDFP - (fpLS.size() + fpRLS.size() + fpEKF.size());
	
	if (numDetectFp < 1){

	}
	else{
		cv::Mat grayImg;
		cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

		goodFeaturesToTrack(grayImg, newFp, MAXDETECTFP, 0.01, 30);

		/// Set the neeed parameters to find the refined corners
		cv::Size winSize = cv::Size(5, 5);
		cv::Size zeroZone = cv::Size(-1, -1);
		cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
		cornerSubPix(grayImg, newFp, winSize, zeroZone, criteria);

		int rndIdx;
		for (int i = 0; i < numDetectFp; i++){
			rndIdx = rand() % MAXDETECTFP;
			fpLS.push_back(newFp[rndIdx]);
		}
	}
}


void ManageFp::TrackingAndDetectFp(cv::Mat currImg){
	std::vector<cv::Point2f> fpPreFilter;

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

			cv::calcOpticalFlowPyrLK(preImg, currImg, fpLS, fpPreFilter, status, err);

			// Filter: getting rid of points for which the KLT tracking failed or those who have gone outside the frame
			int indexCorrection = 0;
			for (int i = 0; i<status.size(); i++)
			{
				cv::Point2f pt = fpPreFilter.at(i - indexCorrection);
				if ((status.at(i) == 0) || (pt.x<0) || (pt.y<0) || (pt.x>currImg.cols) || (pt.y>currImg.rows))	{
					if ((pt.x<0) || (pt.y<0) || (pt.x>currImg.cols) || (pt.y>currImg.rows))	{
						status.at(i) = 0;
					}
					fpPreFilter.erase(fpPreFilter.begin() + (i - indexCorrection));
					fpLS.erase(fpLS.begin() + (i - indexCorrection));
					indexCorrection++;
				}

			}
		}

		// Post process
		fpPreLS = fpLS;
		fpLS = fpPreFilter;

		detectFp(currImg);

		preImg = std::move(currImg);
	}
}


void ManageFp::DrawFp(cv::Mat img){
	int radius = cvRound(5);

	for (int i = 0; i < fpLS.size(); i++){
		cv::circle(img, fpLS[i], radius, cv::Scalar(255, 0, 255));
	}

}



void ManageFp::DrawTracking(cv::Mat img){
	for (size_t i = 0; i<fpPreLS.size(); i++){
		if (status[i]){
			cv::line(img, fpPreLS[i], fpLS[i], cv::Scalar(0, 0, 255));
		}
	}
	//for (int i = 0; i < featuresOptFlowPos.size(); i++){
	//	cv::circle(img, featuresOptFlowPos[i], 3, cv::Scalar(128, 128, 255));
	//}
}
