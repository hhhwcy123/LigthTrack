#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <Eigen\Dense>
#include <opencv2/opencv.hpp>
//#include "typedef.h"
#include "HandHeld\GeoBase.h"

namespace HandHeld
{

using namespace Eigen;
struct Pose {
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	Pose() {
		R = Eigen::Matrix3d::Identity();
		T = Eigen::Vector3d::Zero();
	}
	Pose(Eigen::Matrix3d R_, Eigen::Vector3d T_) {
		R = R_;
		T = T_;
	}
	Pose getPos21(Pose& p2) {
		Pose p;
		p.R = p2.R * R.transpose();
		p.T = p2.T - p.R * T;
		return p;
	}
	cv::Mat getPos3X4() {
		cv::Mat pos3X4 = (cv::Mat_<double>(3, 4) <<
			R(0, 0), R(0, 1), R(0, 2), T[0],
			R(1, 0), R(1, 1), R(1, 2), T[1],
			R(2, 0), R(2, 1), R(2, 2), T[2]
			);
		return pos3X4;
	}
	void getPos3X4(cv::Mat& m) {
		m = (cv::Mat_<float>(3, 4) <<
			R(0, 0), R(0, 1), R(0, 2), T[0],
			R(1, 0), R(1, 1), R(1, 2), T[1],
			R(2, 0), R(2, 1), R(2, 2), T[2]
			);
	}
	void getPos3X4(Matrix<double, 3, 4>& T34) {
		T34 <<
			R(0, 0), R(0, 1), R(0, 2), T[0],
			R(1, 0), R(1, 1), R(1, 2), T[1],
			R(2, 0), R(2, 1), R(2, 2), T[2];
	}
};

struct Camera {
	Eigen::Matrix3d K; // intrinstic_matrix
	Eigen::Matrix<double, 5, 1> J; // distortion_coeffs
	Pose pos;
	Vector3d center;

	Camera() {
		pos.R = K = Eigen::Matrix3d::Identity();
		J = Eigen::Matrix<double, 5, 1>::Zero();
		pos.T = Eigen::Vector3d::Zero();
		center = -pos.R.transpose() * pos.T;
	}
	Camera(Eigen::Matrix3d K_, Eigen::Matrix<double, 5, 1> J_, Pose p_) {
		K = K_;
		J = J_;
		pos = p_;
		center = -pos.R.transpose() * pos.T;
	}
	Camera(Eigen::Matrix3d K_, Eigen::Matrix<double, 5, 1> J_, Eigen::Matrix3d R_, Eigen::Vector3d T_) {
		K = K_;
		J = J_;
		pos = Pose(R_, T_);
		center = -pos.R.transpose() * pos.T;
	}
	void init() {
		center = -pos.R.transpose() * pos.T;
	}
	friend std::ostream& operator<< (std::ostream &os, const Camera &cam) {
		os << "\t f: ( " << cam.K(0, 0) << ", " << cam.K(1, 1) << ")\n"
			<< "\t center: ( " << cam.K(0, 2) << ", " << cam.K(1, 2) << ")\n"
			<< "\t distortion: " << cam.J.transpose() << endl
			<< "\t R:\n" << cam.pos.R << endl
			<< "\t T:\n" << cam.pos.T.transpose() << endl
			<< "\t center: " << cam.center.transpose() << endl;
		return os;
	}
};
void transCameraByCam(Camera& c1, const Camera cc);
void readCameraPara(Camera& c1, Camera& c2, string filePath);
void transCamPara(StrCamPos& camPosL, StrCamPos& camPosR, Camera camL, Camera camR);
void Camera2StrCamPos(Camera c, StrCamPos& cam);
}
#endif