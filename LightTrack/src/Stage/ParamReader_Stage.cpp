#include "Stage/ParamReader_STAGE.hpp"
#include <io.h>
#include"HandHeld/GeoBase.h"
#define MAX_FRAME_NUM 50
#define MAX_SENSER_NUM 20



////////////////////////////////////////////////////////////////////////////////////
// 矩阵的若干运算
////////////////////////////////////////////////////////////////////////////////////
// 求矩阵转置，形参m为行，n为列,A转置后存为B 
void Transpose(double *A, double *B, int m, int n)
{
	for (int i = 0; i < n; i++)
		for (int j = 0; j < m; j++)
			B[i*m + j] = A[j*n + i];
}

// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
void Minus(double *A, double *B, double *C, int m, int n)
{
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			C[i*n + j] = A[i*n + j] - B[i*n + j];
		}
	}
}

// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
void Add(double *A, double *B, double *C, int m, int n)
{
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			C[i*n + j] = A[i*n + j] + B[i*n + j];
		}
	}
}

// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
void Multi(double *A, double *B, double *C, int m, int p, int n)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
		{
			double sum = 0;
			for (int k = 0; k < p; k++)
				sum = sum + A[i*p + k] * B[k*n + j];
			C[i*n + j] = sum;
		}
}


//转台量列表
std::vector<StrPos> m_topoPlat;

//相机在转台零号位置的外参
std::vector<StrPos> m_topoCam;

//相机内参对象
std::vector<SingleCamera> m_mcam;
//
//void TransportRT(StrPos pos1, StrPos pos2, StrPos &pos3)
//{
//	double T[3];
//	Multi(pos2.R, pos1.R, pos3.R, 3, 3, 3);
//	Multi(pos2.R, pos1.T, T, 3, 3, 1);
//	Add(T, pos2.T, pos3.T, 3, 1);
//}
//
//void CrossRT2(StrPos pos1, StrPos pos2, StrPos &pos3)
//{
//	double R1T[9], T[3];
//	Transpose(pos1.R, R1T, 3, 3);
//	Multi(pos2.R, R1T, pos3.R, 3, 3, 3);
//	Multi(pos3.R, pos1.T, T, 3, 3, 1);
//	Minus(pos2.T, T, pos3.T, 3, 1);
//}


//读标定文件
bool ReadBackInsectNCamera(const char* relateCalibFile)
{
	if (access(relateCalibFile, 0) == -1)
		return false;

	m_topoPlat.clear();
	m_topoCam.clear();
	m_mcam.clear();
	m_topoPlat.resize(MAX_FRAME_NUM);
	m_topoCam.resize(MAX_SENSER_NUM);
	m_mcam.resize(MAX_SENSER_NUM);

	const int size = 10240;
	char strtmp[size];
	FILE* fp = fopen(relateCalibFile, "rb");

	if (fp == NULL)
	{
		cout << "file:" << relateCalibFile << " is not exist" << endl;
		return false;
	}

	char tag1[size];
	char tag2[size];
	int platFrame;
	int camNum;
	fgets(strtmp, size, fp);//版本号
	fgets(strtmp, size, fp);//帧数，相机数记录
	sscanf(strtmp, "%s%d%s%d", &tag1, &platFrame, &tag2, &camNum);
	//读 转台RT
	if (platFrame > 0)
	{
		fgets(strtmp, size, fp);
		for (int i = 0; i < platFrame; i++)
		{
			int frame;
			fgets(strtmp, size, fp);
			sscanf(strtmp, "%d", &frame);
			StrPos& pos = m_topoPlat[frame];
			double* R = pos.R;
			double* T = pos.T;
			sscanf(strtmp, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
				&frame,
				&R[0], &R[1], &R[2],
				&R[3], &R[4], &R[5],
				&R[6], &R[7], &R[8],
				&T[0], &T[1], &T[2],
				&pos.Eulor[0], &pos.Eulor[1], &pos.Eulor[2]);
		}
	}
	//读 相机RT
	if (camNum > 0)
	{
		fgets(strtmp, size, fp);
		for (int i = 0; i < camNum; i++)
		{
			fgets(strtmp, size, fp);
			int camNo;
			sscanf(strtmp, "%d", &camNo);
			StrPos& pos = m_topoCam[camNo];
			double* R = pos.R;
			double* T = pos.T;
			int no;
			sscanf(strtmp, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
				&no,
				&R[0], &R[1], &R[2],
				&R[3], &R[4], &R[5],
				&R[6], &R[7], &R[8],
				&T[0], &T[1], &T[2],
				&pos.Eulor[0], &pos.Eulor[1], &pos.Eulor[2]);
		}
	}
	//读 相机内参
	fgets(strtmp, size, fp);
	if (camNum > 0)
	{
		for (int i = 0; i < camNum; i++)
		{
			fgets(strtmp, size, fp);
			int camNo;
			sscanf(strtmp, "%d", &camNo);
			SingleCamera& cam = m_mcam[camNo];
			int no;
			sscanf(strtmp, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
				&no,
				&cam.m_InParam.at<double>(0, 0),
				&cam.m_InParam.at<double>(1, 1),
				&cam.m_InParam.at<double>(0, 2),
				&cam.m_InParam.at<double>(1, 2),
				&cam.m_distCoeffs.at<double>(0, 0),
				&cam.m_distCoeffs.at<double>(0, 1),
				&cam.m_distCoeffs.at<double>(0, 4),
				&cam.m_distCoeffs.at<double>(0, 2),
				&cam.m_distCoeffs.at<double>(0, 3));
			cam.m_InParam.at<double>(2, 2) = 1;
		}
	}
	fclose(fp);

	return true;
}

//读外参
void GetCamOuterPos(const LibBoxCamType camType, const int platNo, StrPos& camPos)
{
	StrPos& pos1 = m_topoPlat[platNo];
	StrPos& pos2 = m_topoCam[camType];

	TransportRT(pos1, pos2, camPos);
}



//根据两个相机的RT和两个相机的内参，计算左相机到右相机的基本矩阵
void  CalcFoundationMat(
	cv::Mat& R, cv::Mat& T,
	cv::Mat& camMatL, cv::Mat& camMatR,
	cv::Mat& FoundationMat)
{
	cv::Mat  camMatL_inv = camMatL.inv();

	cv::Mat  camMatR_Pie_inv = camMatR.inv();
	cv::Mat  camMatR_Pie_inv_tranpose(camMatR_Pie_inv.size(), CV_64F);
	cv::transpose(camMatR_Pie_inv, camMatR_Pie_inv_tranpose);

	double t1 = -T.at<double>(0);
	double t2 = -T.at<double>(1);
	double t3 = -T.at<double>(2);

	cv::Mat T_cha = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
	T_cha.at<double>(0, 1) = -t3;
	T_cha.at<double>(0, 2) = t2;
	T_cha.at<double>(1, 0) = t3;
	T_cha.at<double>(1, 2) = -t1;
	T_cha.at<double>(2, 0) = -t2;
	T_cha.at<double>(2, 1) = t1;

	FoundationMat = camMatR_Pie_inv_tranpose*T_cha*R*camMatL_inv;
}

/*********************************
* 函数功能：获取某一个相机的相机内参
* 输入参数：textureCam	     相机号
*		   cameraMatrix		 内矩阵
*		   distCoeffs		 畸变参数
* 函数返回：返回bool		 true-成功
* 备注：
/*********************************/
bool GetCamPara(const LibBoxCamType camType, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
	if (camType >= m_topoCam.size())
		return -1;

	m_mcam[camType].m_InParam.copyTo(cameraMatrix);
	m_mcam[camType].m_distCoeffs.copyTo(distCoeffs);
	return true;

}


//获取两个相机相对rt
bool GetStereoCamera(
	stereocamera& streoCam,
	LibBoxCamType camLeft,
	LibBoxCamType camRight,
	const int width, const int height)
{
	if (camLeft >= m_topoCam.size() || camRight >= m_topoCam.size())
		return false;
	if (camLeft >= m_mcam.size() || camRight >= m_mcam.size())
		return false;

	StrPos& posLeft = m_topoCam[camLeft];
	StrPos& posRight = m_topoCam[camRight];

	StrPos posRelate;//两个相机的相对旋转参数R T
	CrossRT2(posLeft, posRight, posRelate);

	cv::Mat R = Mat::zeros(3, 3, CV_64F);
	cv::Mat T = Mat::zeros(3, 1, CV_64F);
	memcpy(R.data, posRelate.R, sizeof(double) * 3 * 3);
	memcpy(T.data, posRelate.T, sizeof(double) * 3 * 1);

	SingleCamera& camL = m_mcam[camLeft];
	SingleCamera& camR = m_mcam[camRight];

	cv::Size imageSize(width, height);
	cv::Size newimageSize(width, height);
	cv::Rect roi1, roi2;

	//通过相对旋转关系，获取，核线矫正所需的 RL PL, RR, PR, Q矩阵  立体相机参数
	stereoRectify(camL.m_InParam, camL.m_distCoeffs, camR.m_InParam, camR.m_distCoeffs, imageSize,
		R, T, streoCam.m_RL, streoCam.m_RR, streoCam.m_PL, streoCam.m_PR, streoCam.m_Q);

	R.copyTo(streoCam.m_R);
	T.copyTo(streoCam.m_T);

	camL.m_InParam.copyTo(streoCam.m_pcameraL.m_InParam);
	camL.m_distCoeffs.copyTo(streoCam.m_pcameraL.m_distCoeffs);

	camR.m_InParam.copyTo(streoCam.m_pcameraR.m_InParam);
	camR.m_distCoeffs.copyTo(streoCam.m_pcameraR.m_distCoeffs);

	return true;
}