#ifndef CONVERTER_H
#define CONVERTER_H

#include "Config.h"

#include<opencv2/core/core.hpp>

#include <Eigen/Dense>

#include "types/sba/types_six_dof_expmap.h"
#include  "types/sim3/types_seven_dof_expmap.h"


#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_TOOL_EXPORTS
#define CONVERTER_API  __declspec(dllexport)
#else
#define CONVERTER_API  __declspec(dllimport)
#endif
#else
#define CONVERTER_API
#endif



namespace SLAM
{

class CONVERTER_API Converter
{
public:
	static g2o::SE3Quat toSE3Quat(const cv::Mat& T);

	static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
	static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
	static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
	static cv::Mat toCvMat(const Eigen::Matrix3d &m);
	static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
	static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t);

	static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);
	static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);
	static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);

	static std::vector<double> toQuaternion(const cv::Mat &M);
};

}// namespace SLAM

#endif // CONVERTER_H
