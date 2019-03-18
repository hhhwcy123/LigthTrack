#include <opencv2/opencv.hpp>
#include "HandHeld\GeoBase.h"
using namespace std;
using namespace cv;



struct   SingleCamera
{
	cv::Mat m_InParam;			//内参矩阵
	cv::Mat m_distCoeffs;		//畸变系数
	int camTypeIndex;
	SingleCamera()
	{
		m_InParam = Mat::zeros(3, 3, CV_64F);
		m_distCoeffs = Mat::zeros(1, 5, CV_64F);
	}
};
struct stereocamera
{
	SingleCamera  m_pcameraL;		//左相机
	SingleCamera  m_pcameraR;		//右相机
	cv::Mat m_R;		//左右相机之间旋转矩阵
	cv::Mat m_T;		//左右相机之间平移矩阵
	cv::Mat m_RL;		//核线矫正左相机旋转矩阵
	cv::Mat m_PL;		//核线矫正左相机投影矩阵
	cv::Mat m_RR;		//核线矫正右相机旋转矩阵
	cv::Mat m_PR;		//核线矫正右相机投影矩阵
	cv::Mat m_Q;		//核线矫正Q矩阵
};
enum LibBoxCamType
{
	P1_LEFT = 0,
	P1_RIGHT,
	P2_LEFT,
	P2_RIGHT,
	T1,
	T2,
	PROJECTOR_1,
	PROJECTOR_2
};



void GetCamOuterPos(const LibBoxCamType camType, const int platNo, StrPos& camPos);

//按最大容量存储
bool ReadBackInsectNCamera(const char* relateCalibFile);

bool GetCamPara(const LibBoxCamType camType, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

void  CalcFoundationMat(
	cv::Mat& R, cv::Mat& T,
	cv::Mat& camMatL, cv::Mat& camMatR,
	cv::Mat& FoundationMat);

bool GetStereoCamera(
	stereocamera& streoCam,
	LibBoxCamType camLeft,
	LibBoxCamType camRight,
	const int width, const int height);

