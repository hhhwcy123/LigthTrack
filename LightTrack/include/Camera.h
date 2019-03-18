#ifndef CAMERA_H
#define CAMERA_H
#include <iostream>

#include <opencv2/opencv.hpp>

#include "ReadWriter.h"
#include "Config.h"

#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_ELEM_EXPORTS
#define CAMERA_API  __declspec(dllexport)
#else
#define CAMERA_API  __declspec(dllimport)
#endif
#else
#define CAMERA_API
#endif


namespace SLAM
{

//class  CAMERA_API Camera
class CAMERA_API Camera
{
public:
	Camera():
		_imgWidth(0),
		_imgHeight(0),
		_isRGB(true),
		_baseLen(0)
	{};

	Camera& operator =(const Camera& cam)
	{
		_Kl = cam._Kl.clone();
		_Kr = cam._Kr.clone();
		_Dl = cam._Dl.clone();
		_Dr = cam._Dr.clone();
		_baseLen = cam._baseLen;
		_Rl = cam._Rl.clone();
		_Rr = cam._Rr.clone();
		_Pl = cam._Pl.clone();
		_Pr = cam._Pr.clone();
		_R = cam._R.clone();
		_t = cam._t.clone();
		_E = cam._E.clone();
		_F = cam._F.clone();
		_Q = cam._Q.clone();
		_mapXLeft = cam._mapXLeft.clone();
		_mapYLeft = cam._mapYLeft.clone();
		_mapXRight = cam._mapXRight.clone();
		_mapYRight = cam._mapYRight.clone();

		_imgWidth = cam._imgWidth;
		_imgHeight = cam._imgHeight;

		_isRGB = cam._isRGB;
		return *this;
	}

	~Camera() {}

	bool calibrateStereo(const std::string& chessImgDirLeft, const std::string& chessImgDirRight,
						 cv::Size boardSize, float squareSize, 	
						 std::string outputDir = "Parameters/",
						 bool displayCorners = false, bool showRectified = true);


	bool loadStereoExtrinsicsFile(const std::string& filePath);

	bool loadStereoIntrinsicsFile(const std::string& filePath);

	 cv::Mat& Kl()  { return _Kl; }
	 cv::Mat& Kr()  { return _Kr; }
	 cv::Mat& Dl()  { return _Dl; }
	 cv::Mat& Dr()  { return _Dr; }
	 double& baseLen()  { return _baseLen; }

	 cv::Mat& Rl()  { return _Rl; }
	 cv::Mat& Rr()  { return _Rr; }
	 cv::Mat& Pl()  { return _Pl; }
	 cv::Mat& Pr()  { return _Pr; }
	 cv::Mat& R() { return _R; }
	 cv::Mat& t() { return _t; }
	 cv::Mat& E() { return _E; }
	 cv::Mat& F() { return _F; }
	 cv::Mat& Q() { return _Q; }
	 cv::Mat& mapXl() { return _mapXLeft; }
	 cv::Mat& mapYl() { return _mapYLeft; }
	 cv::Mat& mapXr() { return _mapXRight; }
	 cv::Mat& mapYr() { return _mapYRight; }

	 int& width()  { return _imgWidth; }
	 int& height()  { return _imgHeight; }

	 bool& isRGB()  { return _isRGB; }

private:
	
	cv::Mat					_Kl;
	cv::Mat					_Kr;
	cv::Mat					_Dl;
	cv::Mat					_Dr;
	double					_baseLen;
							
	cv::Mat					_Rl;
	cv::Mat					_Rr;
	cv::Mat					_Pl;
	cv::Mat					_Pr;
	cv::Mat					_R;
	cv::Mat					_t;
	cv::Mat					_E;
	cv::Mat					_F;
	cv::Mat					_Q;
	cv::Mat                 _mapXLeft;
	cv::Mat					_mapYLeft;
	cv::Mat					_mapXRight;
	cv::Mat					_mapYRight;

	int						_imgWidth;
	int						_imgHeight;

	bool					_isRGB;
};







}

#endif // CAMERA_H