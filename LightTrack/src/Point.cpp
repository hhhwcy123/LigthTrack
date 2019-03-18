#include "Point.h"
#include "impl\Point.hpp"
#include "GlobalMap.h"
#include "Matcher.h"
#include "Camera.h"


namespace SLAM
{


void KeyPoint::undistort(const cv::Mat K, const cv::Mat D)
{
	cv::Mat mat(1, 2, CV_64F);

	mat.at<double>(0, 0) = x;
	mat.at<double>(0, 1) = y;


	mat = mat.reshape(2);
	cv::undistortPoints(mat, mat, K, D, cv::Mat(), K);
	mat = mat.reshape(1);

	_undistortX = mat.at<double>(0);
	_undistortY = mat.at<double>(1);

}

void KeyPoint::calcWeight(const Feature::DotMarker& dotMarker)
{
	if (dotMarker.fBad)
		/*if (markerLeft.definition < 5)
		weight = 0.;
		else*/
		_weight = 1 - pow(dotMarker.ellipseFitError, 1);// markerLeft.definition / 20; //sqrt((markerLeft.definition-5) / 15);
	else
		_weight = 1.0;

	if (dotMarker.fBad)
		/*if (markerRight.definition < 5)
		weight = 0.;
		else*/
		_weight = 1 - pow(dotMarker.ellipseFitError, 1);// markerRight.definition / 20; //sqrt((markerRight.definition - 5) / 15);
	else
		_weight = 1.0;
}


};