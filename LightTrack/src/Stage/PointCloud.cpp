#include "Stage\PointCloud.h"




bool PointCloud::p3DCloudTrans(SLAM::ReadWriter::MatList& x3dList, const cv::Mat& R, const cv::Mat& t)
{
	for (SLAM::ReadWriter::MatList::iterator itr = x3dList.begin(); itr != x3dList.end(); itr++)
	{
		cv::Mat x3DTrans=R*(*itr).t() + t;
		if (!x3DTrans.empty())
			*itr = x3DTrans;
		else
			return false;
	}
	return true;
}

bool PointCloud::normVectorTrans(SLAM::ReadWriter::MatList& normList, const cv::Mat& R, const cv::Mat& t)
{
	for (SLAM::ReadWriter::MatList::iterator itr = normList.begin(); itr != normList.end(); itr++)
	{
		cv::Mat normTrans=R*(*itr).t();
		if (!normTrans.empty())
			*itr = normTrans;
		else
			return false;
	}
	return true;
}