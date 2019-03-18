#include <opencv2/opencv.hpp>
#include "HandHeld\GeoBase.h"
using namespace std;
using namespace cv;



struct   SingleCamera
{
	cv::Mat m_InParam;			//�ڲξ���
	cv::Mat m_distCoeffs;		//����ϵ��
	int camTypeIndex;
	SingleCamera()
	{
		m_InParam = Mat::zeros(3, 3, CV_64F);
		m_distCoeffs = Mat::zeros(1, 5, CV_64F);
	}
};
struct stereocamera
{
	SingleCamera  m_pcameraL;		//�����
	SingleCamera  m_pcameraR;		//�����
	cv::Mat m_R;		//�������֮����ת����
	cv::Mat m_T;		//�������֮��ƽ�ƾ���
	cv::Mat m_RL;		//���߽����������ת����
	cv::Mat m_PL;		//���߽��������ͶӰ����
	cv::Mat m_RR;		//���߽����������ת����
	cv::Mat m_PR;		//���߽��������ͶӰ����
	cv::Mat m_Q;		//���߽���Q����
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

//����������洢
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

