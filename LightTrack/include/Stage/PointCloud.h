#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <opencv2/core/core.hpp>
#include "ReadWriter.h"

#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_TOOL_EXPORTS
#define POINTCLOUD_API  __declspec(dllexport)
#else
#define POINTCLOUD_API  __declspec(dllimport)
#endif
#else
#define POINTCLOUD_API
#endif

class POINTCLOUD_API PointCloud
{
	//typedef std::vector<cv::Point3d> P3DList;
public:
	PointCloud() {}
	~PointCloud() {}

	static bool p3DCloudTrans(SLAM::ReadWriter::MatList& x3dList,const cv::Mat& R,const cv::Mat& t);

	static bool normVectorTrans(SLAM::ReadWriter::MatList& x3dList, const cv::Mat& R, const cv::Mat& t);

};

#endif//POINTCLOUD_H