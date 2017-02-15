
#ifndef _MANAGEFP_H_
#define _MANAGEFP_H_

#include <opencv2\core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

#define TRACKINGTHRESHHOLD 50

#define MAXMANAGEDFP 200

class ManageFp{
private:
	//cv::Ptr<cv::ORB> detector;
	//std::vector<cv::KeyPoint> keypoints;

	cv::Mat preImg;
	std::vector<cv::Point2f> fpLS,fpRLS,fpEKF;
	std::vector<cv::Point2f> fpPreLS;

	std::vector<uchar> status;
public:
	ManageFp();
	~ManageFp();

	void detectFp(cv::Mat img);
	void TrackingAndDetectFp(cv::Mat);
	void DrawFp(cv::Mat img);
	void DrawTracking(cv::Mat img);
	//cv::Mat img_1, cv::Mat img_2,
	//	std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2,
	//	std::vector<uchar>& status);
};

#endif
