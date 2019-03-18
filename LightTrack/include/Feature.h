#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/opencv.hpp>

#include <cuda_runtime.h>
#include <npp.h>

#include "Config.h"
#include "SubPixel\SubPixel.h"
#include "EdgeDetector\EdgeDrawing.hpp"

#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_ELEM_EXPORTS
#define FEATURE_API  __declspec(dllexport)
#else
#define FEATURE_API  __declspec(dllimport)
#endif
#else
#define FEATURE_API
#endif

namespace SLAM
{
#define  PI (3.1415926535897932346f)  

#define extractFuncsLen 5

class FEATURE_API Feature
{
	friend class KeyPoint;

public:

	static int  MinArea;        //轮廓最小面积
	static int  MaxArea;        //轮廓最大面积
	static double  MinCircularity;  //最小圆度限制
	static double  MaxCircularity;   //最大圆度限制
	static double  MaxConvexity; //最大凸度限制
	static double  MinConvexity;   //最小凸度限制
	static double  MaxEccentricity;//最大惯性率限制
	static double  MaxFitRMSE; //轮廓椭圆拟合RMSE限制
	static double  MaxContourError; //轮廓椭圆拟合局部最大偏差限制
	static double  MinDefinition;//标识点分辨度限制
	static double  DotMarkRadius;//标识点半径默认3mm
	
	enum ExtractMarkType
	{
		SIMPLE,//常规椭圆标志点提取算法
		NPP_CUDA,//使用Cuda-Npp库进行canny检测的常规算法
		CV_CUDA,//基于Opencv-Cuda加速的常规算法		
		THRESHOLD,//图像二值化提取算法
		SUBPIX//椭圆亚像素提取算法
	};

	typedef struct ExtractParams
	{
		bool filterByArea = true;
		double minArea = MinArea; //轮廓最小面积
		double maxArea = MaxArea;//轮廓最大面积

		bool filterByCircularity = false;
		double minCircularity = MinCircularity;//最小圆度限制
		double maxCircularity = MaxCircularity;//最大圆度限制

		bool filterByConvexity = false;
		double maxConvexity = MaxConvexity;//最大凸度限制
		double minConvexity = MinConvexity;//最小凸度限制
		bool filterByEccentricity = false;
		double maxEccentricity = MaxEccentricity;//最大离心率限制
	};

	struct EllipseParam
	{
		EllipseParam() {}
		EllipseParam(const double& A, const double& B, const double& C, const double& D, const double& E, const double& F) :
			A(A), B(B), C(C), D(D), E(E), F(F) {}
		double A;
		double B;
		double C;
		double D;
		double E;
		double F;
	};

	struct MarkerPose
	{
		MarkerPose() {}
		MarkerPose(const cv::Mat& center, const cv::Mat& norm) :center3d(center), normVector(norm) {}
		MarkerPose& operator =(const MarkerPose& pose)
		{
			center3d = pose.center3d;
			normVector = pose.normVector;
			return *this;
		}
		cv::Mat center3d;
		cv::Mat normVector;
	};

	typedef struct tagMarkerMask
	{
		cv::Mat mask;
		cv::Point oriPos;
	}MarkerMask;

	struct DotMarker
	{
		DotMarker() :
			x(0),
			y(0),
			undistortX(0),
			undistortY(0),
			areaSize(0),
			eccentricity(0),
			definition(0),
			ellipseFitError(0)
		{}
		DotMarker(cv::RotatedRect& rect) :
			x(rect.center.x),
			y(rect.center.y),
			undistortX(rect.center.x),
			undistortY(rect.center.y),
			areaSize(0),
			eccentricity(0),
			definition(0),
			ellipseFitError(0),
			rect(rect)
		{}
		
		DotMarker(const double& x,
			const double& y,
			const double& areaSize,
			const double &eccentricity) :
			x(x),
			y(y),
			undistortX(x),
			undistortY(y),
			areaSize(areaSize),
			eccentricity(eccentricity),
			definition(0),
			ellipseFitError(0)
		{}

		DotMarker operator -(const DotMarker& marker) //重载 - 
		{
			x = x - marker.x;
			y = y - marker.y;
			return DotMarker(x, y, 0, 0);
		}
		bool operator ==(const DotMarker& marker) const//重载 == 
		{
			return (x == marker.x) && (y == marker.y);
		}

		bool operator <(const DotMarker& marker) const //重载 < 
		{
			return cv::norm(cv::Point(x, y))<cv::norm(cv::Point(marker.x, marker.y));
		}

		void undistort(cv::Mat& K, cv::Mat& D, cv::Mat& P)
		{
			cv::Mat posMat(1, 2, CV_64F);

			posMat.at<double>(0) = x;
			posMat.at<double>(1) = y;


			posMat = posMat.reshape(2);
			cv::undistortPoints(posMat, posMat, K, D, cv::Mat(), P);
			posMat = posMat.reshape(1);

			undistortX = posMat.at<double>(0);
			undistortY = posMat.at<double>(1);
		}

		double length() { return sqrt(x*x + y*y); }

		double x;
		double y;
		double undistortX;
		double undistortY;
		double areaSize;//面积
		double eccentricity;//惯性率
		double definition;//清晰度
		double ellipseFitError;//椭圆拟合误差
		std::list<MarkerPose> poseList;//空间位姿（中心+法向量）
		MarkerPose bestPose;
		cv::RotatedRect rect;//椭圆外包矩形
		std::vector<cv::Point> contours;//标志点轮廓
		bool fBad = false;
};

	struct Comparator
	{
		bool operator()(DotMarker& marker1, DotMarker& marker2) const { return marker1.x*marker1.x + marker1.y*marker1.y < marker2.x*marker2.x + marker2.y*marker2.y; }
	};
	typedef std::vector<DotMarker> MarkerList;
	typedef std::pair<DotMarker, DotMarker> MarkerPair;
	typedef std::vector<MarkerPair> MarkerPairList;
	typedef std::map<DotMarker, DotMarker> MarkerMap;
	typedef std::multimap<DotMarker, DotMarker> MarkerMultiMap;

	typedef bool(*ExtractMarkFunc)(const cv::Mat&, MarkerList&);

public:
	Feature() {};
	~Feature() {};


	static bool cannyNpp(const cv::Mat& srcImg, cv::Mat& dstImg,
						 const double threshold1, const double threshold2,
						 NppiMaskSize kernel_size = NPP_MASK_SIZE_3_X_3);

	static bool extractDotMarkersNpp(const cv::Mat& srcImg, MarkerList& markerList);

	static bool extractDotMarkersCuda(const cv::Mat& srcImg, MarkerList& markerList);

	static bool extractDotMarkersSimple(const cv::Mat& srcImg, MarkerList& markerList);

	static bool extractDotMarkersFast(const cv::Mat& srcImg, MarkerList& markerList);

	static bool extractDotMarkersSubPixel(const cv::Mat& srcImg, MarkerList& markerList);	

	static void fitEllipse2f(const std::vector<cv::Point>& contours, cv::Mat& params);

	static cv::RotatedRect cvfitEllipse(const cv::InputArray& points);

	static void getEllipseParam(const cv::RotatedRect& ellipsemege, EllipseParam& param);

	static std::list<Feature::MarkerPose> calcMarkerPose(const DotMarker& marker, const cv::Mat& K);

	static double calcVecAngle(const cv::Mat& norm1, const cv::Mat& norm2) { return acos(norm1.dot(norm2) / (cv::norm(norm1)*cv::norm(norm2))); }

	template<class T>
	static bool calcEllipseFitError(const cv::RotatedRect& rectEllipse, const std::vector<T>& contour, double& fitError, const double& th);
	
	template<class T>
	static double calcEllipseFitError(const std::vector<T>& contours, const EllipseParam& params);

	static double calcMarkerDefinition(const cv::Mat& srcImg, const DotMarker& marker);

	static MarkerMask getMarkerMask(const cv::RotatedRect& rectEllipse);

	static const ExtractMarkFunc& extractMarkFunc(const unsigned& i) { assert(i >= 0 && i < extractFuncsLen); return _extractMarkFuncs[i]; }

	static const ExtractMarkFunc _extractMarkFuncs[extractFuncsLen];
};

}


#endif//FEATURE_H