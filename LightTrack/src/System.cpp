#include "System.h"
#include "Converter.h"

#include "HandHeld\GeoBase.h"
#include "HandHeld\ParamReader_Hand.h"
#include "Stage\ParamReader_STAGE.hpp"

namespace SLAM
{

System* System::_instance = nullptr;
System::Garbo System::_garbo;


void usleep(__int64 usec)
{
	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
}


void System::loadHandHeldCamera(std::string calibFilePath )
{
	HandHeld::Camera camL, camR;
	StrCamPos camPosL,camPosR;
	readCameraPara(camL, camR, calibFilePath);
	transCamPara(camPosL, camPosR, camL, camR);
	

	_camera.Kl() = (cv::Mat_<double>(3, 3) << camPosL.f[0], 0, camPosL.C[0],
										      0, camPosL.f[1], camPosL.C[1],
										      0, 0, 1);
	_camera.Kr() = (cv::Mat_<double>(3, 3) << camPosR.f[0], 0, camPosR.C[0],
											  0, camPosR.f[1], camPosR.C[1],
											  0, 0, 1);

	_camera.Dl() = (cv::Mat_<double>(1, 5) << camPosL.k[0], camPosL.k[1], camPosL.p[0], camPosL.p[1], camPosL.k[2]);
	_camera.Dr() = (cv::Mat_<double>(1, 5) << camPosR.k[0], camPosR.k[1], camPosR.p[0], camPosR.p[1], camPosR.k[2]);
	_camera.width() = 1280;
	_camera.height() = 1024;
	_camera.R() = cv::Mat(3, 3, CV_64F, camPosR.pos.R).clone();
	_camera.t() = cv::Mat(3, 1, CV_64F, camPosR.pos.T).clone();

	CalcFoundationMat(_camera.R(), _camera.t(), _camera.Kl(), _camera.Kr(), _camera.F());

	cv::Rect validRoi[2];
	const int& width = _camera.width();
	const int& height = _camera.height();
	cv::Size imageSize = cv::Size(width, height);
	stereoRectify(_camera.Kl(), _camera.Dl(), _camera.Kr(), _camera.Dr(),
				  imageSize, _camera.R(), _camera.t(), _camera.Rl(), _camera.Rr(), _camera.Pl(), _camera.Pr(), _camera.Q(),
				  0, 1, imageSize, &validRoi[0], &validRoi[1]);

	cv::initUndistortRectifyMap(_camera.Kl(), _camera.Dl(), cv::Mat(),  _camera.Kl(),
								cv::Size(_camera.width(), _camera.height()), CV_32F,
								_camera.mapXl(), _camera.mapYl());
	cv::initUndistortRectifyMap(_camera.Kr(), _camera.Dr(),  cv::Mat(), _camera.Kr(),
								cv::Size(_camera.width(), _camera.height()), CV_32F, 
								_camera.mapXr(), _camera.mapYr());
}



void System::loadStageCamera(std::string calibFilePath)
{
	ReadBackInsectNCamera(calibFilePath.c_str());


	LibBoxCamType  L_CAM = P1_LEFT;
	LibBoxCamType  R_CAM = P1_RIGHT;

	//L_CAM内参
	cv::Mat  L_cameraMatrix;
	cv::Mat  L_distCoeffs;
	GetCamPara(L_CAM, L_cameraMatrix, L_distCoeffs);


	//R_CAM内参
	cv::Mat  R_cameraMatrix;
	cv::Mat  R_distCoeffs;
	GetCamPara(R_CAM, R_cameraMatrix, R_distCoeffs);

	stereocamera stereoCam;
	const int width = 2048;
	const int height = 2448;
	GetStereoCamera(stereoCam, L_CAM, R_CAM, width, height);


	cv::Mat F;
	CalcFoundationMat(stereoCam.m_R, stereoCam.m_T, L_cameraMatrix, R_cameraMatrix, F);

	_camera.Kl() = L_cameraMatrix;
	_camera.Kr() = R_cameraMatrix;
	_camera.Dl() = L_distCoeffs;
	_camera.Dr() = R_distCoeffs;
	_camera.width() = width;
	_camera.height() = height;
	_camera.R() = stereoCam.m_R;
	_camera.t() = stereoCam.m_T;
	_camera.Pl() = stereoCam.m_PL;
	_camera.Pr() = stereoCam.m_PR;
	_camera.Rl() = stereoCam.m_RL;
	_camera.Rr() = stereoCam.m_RR;
	_camera.F() = F;

	cv::initUndistortRectifyMap(_camera.Kl(), _camera.Dl(), cv::Mat(), _camera.Kl(), cv::Size(_camera.width(), _camera.height()), CV_32F, _camera.mapXl(), _camera.mapYl());
	cv::initUndistortRectifyMap(_camera.Kr(), _camera.Dr(), cv::Mat(), _camera.Kr(), cv::Size(_camera.width(), _camera.height()), CV_32F, _camera.mapXr(), _camera.mapYr());
}



bool System::calibrate()
{
	if (_calibImgDir.back() != '\\')_calibImgDir += '\\';

	std::string calibImgLeftDir, calibImgRightDir;
	calibImgLeftDir = _calibImgDir + "L";
	calibImgRightDir = _calibImgDir + "R";

	int w = 13, h = 10, squareSize = 15;
	cv::Size boardSize(w, h);
	if (!_camera.calibrateStereo(calibImgLeftDir, calibImgRightDir, boardSize, squareSize, _calibImgDir, true))
		return false;

	return true;
}



bool System::loadCalibParams()
{
	if (_calibDataDir.back() != '\\')_calibDataDir += '\\';
	std::string calibExPath = _calibDataDir + "extrinsics.yml";
	std::string calibInPath = _calibDataDir + "intrinsics.yml";
	if (!_camera.loadStereoExtrinsicsFile(calibExPath))
	{
		std::cout << "Failed loading extrinsics file from: " << calibExPath << std::endl;
		return false;
	}
	if (!_camera.loadStereoIntrinsicsFile(calibInPath))
	{
		std::cout << "Failed loading intrinsics file from: " << calibInPath << std::endl;
		return false;
	}

	return true;
}



bool System::initiate()
{	
	_mapDrawer.setGlobalMap(&_globalMap);
	_mapDrawer.setViewer(&_viewer);
	_mapDrawer.setLoopCloser(&_loopCloser);

	_viewer.setSystem(this);
	_viewer.setDrawer(&_mapDrawer);
	_viewer.setTracker(&_tracker);

	_tracker.setViewer(&_viewer);
	_tracker.setMap(&_globalMap);
	_tracker.setMapDrawer(&_mapDrawer);
	_tracker.setSystem(this);
	_tracker.setLocalMapper(&_localMapper);

	_localMapper.setSystem(this);
	_localMapper.setGlobalMap(&_globalMap);
	_localMapper.setLoopCloser(&_loopCloser);
	_localMapper.setTracker(&_tracker);

	_loopCloser.setGlobalMap(&_globalMap);
	_loopCloser.setSystem(this);
	_loopCloser.setLocalMapper(&_localMapper);
	

	if (isOutputPos())
	{
		if (!openPosFile())
		{
			std::cout << "Failed opening output pos file!" << std::endl;
		}
	}
	if (isOutputP3d())
	{
		if (!openP3dFile())
		{
			std::cout << "Failed opening output 3d points List file!" << std::endl;
		}
	}
		
	///////////////////建立帧位姿3D模型显示线程(可选)/////////////////////
	if (isDisplay3DModel())
	{
		if (!_viewer.startThread())
		{
			std::cout << "Failed creating viewing thread" << std::endl;
			return false;
		}
		std::cout << "Created viewing thread!" << std::endl;
	}
	
	///////////////////建立局部优化线程(可选)/////////////////////
	if (isLocalBundleAdjustment())
	{
		if (!_localMapper.startThread())
		{
			std::cout << "Failed creating local bundle adjustment thread" << std::endl;
			return false;
		}
		std::cout << "Created local bundle adjustment thread!" << std::endl;
	}

	/////////////////建立回环检测子线程(可选)////////////////////
	if (isLoopClosing())
	{
		if (!_loopCloser.startThread())
		{
			std::cout << "Failed creating Loop Closing thread" << std::endl;
			return false;
		}
		std::cout << "Created loop closing thread!" << std::endl;	
	}
	return true;
}




void System::undistortImage(const cv::Mat& imgLeft, const cv::Mat& imgRight, cv::Mat& imgRectLeft,cv::Mat& imgRectRight)
{
	//cv::undistort(imgLeft, outImgLeft, _camera.Kl(), _camera.Dl());
	//cv::undistort(imgRight, outImgRight, _camera.Kl(), _camera.Dl());

	if (_camera.mapXl().empty() || _camera.mapXr().empty()||_camera.mapYl().empty() || _camera.mapYr().empty())
	{
		std::cout << "Wrong: Empty camera undistort mapping matrix!" << std::endl;
		return;
	}
	/*std::thread* pThread1 = new std::thread(&cv::remap, imgLeft, std::ref(imgRectLeft), _camera.mapXL(), _camera.mapYL(), cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar());
	std::thread* pThread2 = new std::thread(&cv::remap, imgRight, std::ref(imgRectRight), _camera.mapXR(), _camera.mapYR(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	pThread1->join();
	pThread2->join();*/

	cv::remap(imgLeft, imgRectLeft, _camera.mapXl(), _camera.mapYl(), cv::INTER_LINEAR);
	cv::remap(imgRight, imgRectRight, _camera.mapXr(), _camera.mapYr(), cv::INTER_LINEAR);
}



SLAM::Frame* System::createImgFrame(const cv::Mat& imgLeft, const cv::Mat& imgRight, bool isUndistored)
{
	assert(imgLeft.rows == imgRight.rows&&imgLeft.cols == imgRight.cols);
	SLAM::Frame* pFrame = new SLAM::Frame(0, imgLeft.cols, imgRight.rows);
	if (pFrame)
	{
		pFrame->setSystem(this);
		pFrame->setCamera(&_camera);
		pFrame->setGlobalMap(&_globalMap);

		pFrame->setUndistorted(isUndistored);

		pFrame->compute3Dpoints(imgLeft, imgRight,
								pFrame->calcMatchedFunc(Frame::NORM),
								Feature::extractMarkFunc(Feature::SIMPLE));

		return pFrame;
	}
	return nullptr;
}



SLAM::Frame* System::createFrame(const cv::Mat& imgLeft, const cv::Mat& imgRight, bool isUndistored)
{
	const int& frameID = GlobalMap::frameCount()++;
	assert(imgLeft.rows == imgRight.rows&&imgLeft.cols == imgRight.cols);

	SLAM::Frame* pFrame = new SLAM::Frame(frameID, imgLeft.cols, imgLeft.rows);

	if (pFrame)
	{
		pFrame->setSystem(this);
		pFrame->setCamera(&_camera);
		pFrame->setGlobalMap(&_globalMap);

		pFrame->setUndistorted(isUndistored);

		if (!pFrame->compute3Dpoints(imgLeft, imgRight, 
									 pFrame->calcMatchedFunc(getCalcMatchedType()), 
									 Feature::extractMarkFunc(getExtractMarkType())))
			return nullptr;

		
		if (_bOutputStereoImg)
		{
			std::string outDir = getStereoImgOutDir();//outDir可以从System对象调取
			if (outDir.back() != '\\') outDir += '\\';
			if (access(outDir.c_str(), 0) == -1)
			{
				if (_mkdir(outDir.c_str()))
				{
					std::cout << "Cannot create mathced output dir!" << std::endl;
				}
			}
			if (!pFrame->createMatchedImg())
			{
				fprintf(stderr, "Frame %d failed creating matched image\n", pFrame->getID());
			}
			ReadWriter::saveMatchedMarkImg(outDir, pFrame);
		}
		return pFrame;
	}

	return nullptr;
}


SLAM::Frame* System::createFrame(const unsigned int& index,const cv::Mat& imgLeft, const cv::Mat& imgRight, bool isUndistored)
{
	assert(imgLeft.rows == imgRight.rows&&imgLeft.cols == imgRight.cols);

	SLAM::Frame* pFrame = new SLAM::Frame(index, imgLeft.cols, imgLeft.rows);
	if (pFrame)
	{
		pFrame->setSystem(this);
		pFrame->setCamera(&_camera);
		pFrame->setGlobalMap(&_globalMap);

		pFrame->setUndistorted(isUndistored);

		if (!pFrame->compute3Dpoints(imgLeft, imgRight,
									 pFrame->calcMatchedFunc(getCalcMatchedType()),
									 Feature::extractMarkFunc(getExtractMarkType())))
			return nullptr;


		if (_bOutputStereoImg)
		{
			std::string outDir = getStereoImgOutDir();//outDir可以从System对象调取
			if (outDir.back() != '\\') outDir += '\\';
			if (access(outDir.c_str(), 0) == -1)
			{
				if (_mkdir(outDir.c_str()))
				{
					std::cout << "Cannot create mathced output dir!" << std::endl;
				}
			}
			if (!pFrame->createMatchedImg())
			{
				fprintf(stderr, "Frame %d failed creating matched image\n", pFrame->getID());
			}
			ReadWriter::saveMatchedMarkImg(outDir, pFrame);
		}
		return pFrame;
	}
	return nullptr;
}


cv::Mat System::trackFrame(const unsigned int& index, const cv::Mat& imgLeft, const cv::Mat& imgRight)
{
	if (SLAM::Frame* newFrame = createFrame(index, imgLeft, imgRight))
	{
	
		if (getTracker().tracking(*newFrame))
			return newFrame->getPose();
		
	}	
	return cv::Mat();
}



cv::Mat System::trackFrame(Frame& frame)
{
	if (getTracker().tracking(frame))
		return frame.getPose();
	return cv::Mat();
}


void System::calcFrameMatchedRMSE(GlobalMap* pMap, int* matchedRMSEHist, const int& len, const double& maxVal, Statistic& matchedRMSEStats)
{
	double sum = 0;
	double bin = (double)maxVal / (double)len;
	double min = DBL_MAX;
	double max = DBL_MIN;
	FrameList& frameList = pMap->getFrameList();
	for (auto fItr = frameList.begin(); fItr != frameList.end(); fItr++)
	{
		if ((*fItr)->isBad())
			continue;
		double matchedRMSE = (*fItr)->calcMatchedRMSE();
		double i = matchedRMSE / bin;
		matchedRMSEHist[(int)i]++;
		sum += matchedRMSE;
		if (matchedRMSE > max)
			max = matchedRMSE;
		if (matchedRMSE < min)
			min = matchedRMSE;
	}

	matchedRMSEStats.minVal = min;
	matchedRMSEStats.maxVal = max;
	matchedRMSEStats.meanVal = sum / (double)(frameList.size());
}



void System::calcKFMatchedRMSE(GlobalMap* pMap, int* matchedRMSEHist, const int& len, const double& maxVal, Statistic& matchedRMSEStats)
{
	double sum = 0;
	double bin = (double)maxVal / (double)len;
	double min = DBL_MAX;
	double max = DBL_MIN;
	KeyFrameList& kFList = pMap->getKeyFrameList();
	for (auto kFItr = kFList.begin(); kFItr != kFList.end(); kFItr++)
	{
		if ((*kFItr)->isBad())
			continue;
		double matchedRMSE = (*kFItr)->calcMatchedRMSE();
		double i = matchedRMSE / bin;
		matchedRMSEHist[(int)i]++;
		sum += matchedRMSE;
		if (matchedRMSE > max)
			max = matchedRMSE;
		if (matchedRMSE < min)
			min = matchedRMSE;
	}

	matchedRMSEStats.minVal = min;
	matchedRMSEStats.maxVal = max;
	matchedRMSEStats.meanVal = sum / (double)(kFList.size());
}


void System::calcMarkEccentHisto(Frame* frame, int* EccHist, const int& len, const double& maxVal, Statistic& eccStats)
{
	//*EccHist = new int[len];
	//memset(EccHist, 0, len*sizeof(int));
	double sum=0;
	double bin = (double)maxVal / (double)len;
	double min = DBL_MAX;
	double max = DBL_MIN;
	for (KeyPointList::iterator itr = frame->_keyPointsLeft.begin(); itr != frame->_keyPointsLeft.end(); itr++)
	{
		double i = (*itr)->_eccentricity / bin;
		EccHist[(int)i]++;
		sum += (*itr)->_eccentricity;
		if ((*itr)->_eccentricity > max)
			max = (*itr)->_eccentricity;
		if((*itr)->_eccentricity < min)
			min= (*itr)->_eccentricity;
	}
	for (KeyPointList::iterator itr = frame->_keyPointsRight.begin(); itr != frame->_keyPointsRight.end(); itr++)
	{
		double i = (*itr)->_eccentricity / bin;
		EccHist[(int)i]++;
		sum+= (*itr)->_eccentricity;
		if ((*itr)->_eccentricity > max)
			max = (*itr)->_eccentricity;
		if ((*itr)->_eccentricity < min)
			min = (*itr)->_eccentricity;
	}
	eccStats.minVal = min;
	eccStats.maxVal = max;
	eccStats.meanVal = sum / (double)(frame->_keyPointsLeft.size() + frame->_keyPointsRight.size());
}


void System::calcMarkAreaHisto(Frame* frame, int* areaHist, const int& len, const double& maxVal, Statistic& areaStats)
{
	double sum = 0;
	double bin = (double)maxVal / (double)len;
	double min = DBL_MAX;
	double max = DBL_MIN;
	for (KeyPointList::iterator itr = frame->_keyPointsLeft.begin(); itr != frame->_keyPointsLeft.end(); itr++)
	{
		double i = (*itr)->_areaSize / bin;
		areaHist[(int)i]++;
		sum += (*itr)->_areaSize;
		if ((*itr)->_areaSize > max)
			max = (*itr)->_areaSize;
		if ((*itr)->_areaSize < min)
			min = (*itr)->_areaSize;
	}
	for (KeyPointList::iterator itr = frame->_keyPointsRight.begin(); itr != frame->_keyPointsRight.end(); itr++)
	{
		double i = (*itr)->_areaSize / bin;
		areaHist[(int)i]++;
		sum += (*itr)->_areaSize;
		if ((*itr)->_areaSize > max)
			max = (*itr)->_areaSize;
		if ((*itr)->_areaSize < min)
			min = (*itr)->_areaSize;
	}
	areaStats.minVal = min;
	areaStats.maxVal = max;
	areaStats.meanVal = sum / (double)(frame->_keyPointsLeft.size() + frame->_keyPointsRight.size());
	/*
	//*EccHist = new int[len];
	//memset(EccHist, 0, len*sizeof(int));
	double sum = 0;

	double min = DBL_MAX;
	double max = DBL_MIN;
	for (Frame::KeyPointList::iterator itr = frame->_keyPointsLeft.begin(); itr != frame->_keyPointsLeft.end(); itr++)
	{
		sum += (*itr)->_areaSize;
		if ((*itr)->_areaSize > max)
			max = (*itr)->_areaSize;
		if ((*itr)->_areaSize < min)
			min = (*itr)->_areaSize;
	}
	for (Frame::KeyPointList::iterator itr = frame->_keyPointsRight.begin(); itr != frame->_keyPointsRight.end(); itr++)
	{
		sum += (*itr)->_areaSize;
		if ((*itr)->_areaSize > max)
			max = (*itr)->_areaSize;
		if ((*itr)->_areaSize < min)
			min = (*itr)->_areaSize;
	}
	areaStats.minVal = min;
	areaStats.maxVal = max;
	areaStats.meanVal = sum / (double)(frame->_keyPointsLeft.size() + frame->_keyPointsRight.size());

	double bin = (double)max / (double)len;
	for (Frame::KeyPointList::iterator itr = frame->_keyPointsLeft.begin(); itr != frame->_keyPointsLeft.end(); itr++)
	{
		double i = (*itr)->_areaSize / bin;
		areaHist[(int)i]++;
	}
	for (Frame::KeyPointList::iterator itr = frame->_keyPointsRight.begin(); itr != frame->_keyPointsRight.end(); itr++)
	{
		double i = (*itr)->_areaSize / bin;
		areaHist[(int)i]++;
	}*/
}


void System::calcPlaneFittingHisto(int* fitRMSEHist, const int& len, const double& maxVal, Statistic& fitRMSEStats)
{
	double mean = 0.;
	double RMSE = 0.;
	double sum = 0;
	double bin = (double)maxVal / (double)len;
	double min = DBL_MAX;
	double max = DBL_MIN;
	GlobalMap::MatList& laserPts = _globalMap.getLaserPtCloud();
	for (auto lItr = laserPts.begin(); lItr != laserPts.end(); lItr++)
	{
		calcPlaneFitError(*lItr, mean, RMSE);
	
		double i = RMSE / bin;
		fitRMSEHist[(int)i]++;
		sum += RMSE;
		if (RMSE > max)
			max = RMSE;
		if (RMSE < min)
			min = RMSE;
	}

	fitRMSEStats.minVal = min;
	fitRMSEStats.maxVal = max;
	fitRMSEStats.meanVal = sum / (double)(laserPts.size());
}


void System::drawStatsHisto(cv::Mat& outImg,int* Histo, const int& len, const double& maxX, Statistic* pStats)
{
	outImg = cv::Mat::zeros(cv::Size(512, 512), CV_8UC3);
	int* maxPtr = std::max_element(Histo, Histo + len);

	int width = 512 / len;
	int histoSum = 0;
	for (int i = 0; i < len; i++)
		histoSum += Histo[i];

	double step = maxX / len;
	for (int i = 0; i < len; i++)
	{
		double ratio = (double)Histo[i] / (double)histoSum;
		int val = 512 * (double)Histo[i] / (double)(*maxPtr);
		std::ostringstream osStep, osVal;
		osStep << step*i;
		osVal << std::setprecision(2) << ratio;
		cv::rectangle(outImg, cv::Rect(cv::Point(width*i, outImg.rows - 1 - val), cv::Point(width * i + width - 1, outImg.rows - 1)), cv::Scalar(255, 0, 0), CV_FILLED);
		cv::putText(outImg, osStep.str(), cv::Point(width*i, outImg.rows - 1 - 5), cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
		if (ratio > 0.05)
		{
			cv::putText(outImg, osVal.str(), cv::Point(width*i + (width - 1) / 5, outImg.rows - 1 - val / 2), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
		}
		//cv::line(histoImg, cv::Point(i, histoImg.rows - 1), cv::Point(i, histoImg.rows - 1 - val), cv::Scalar(255, 0, 0));
	}
	std::ostringstream osStep, osVal;
	osStep << step*len;
	cv::putText(outImg, osStep.str(), cv::Point(width*len, outImg.rows - 1 - 5), cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
	
	if (pStats)
	{
		std::ostringstream osMean, osMin, osMax;
		osMean << "mean: " << pStats->meanVal;
		cv::putText(outImg, osMean.str(), cv::Point(1, 10), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
		osMin << "min: " << pStats->minVal;
		cv::putText(outImg, osMin.str(), cv::Point(1, 22), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
		osMax << "max: " << pStats->maxVal;
		cv::putText(outImg, osMax.str(), cv::Point(1, 34), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
	}
	
	
}



bool System::slamRun()
{

	std::ofstream ofile;
	if (openPosFile())
	{
		std::cout << "Failed opening output pos file!" << std::endl;
	}
	//cv::Mat M1l, M2l, M1r, M2r;
	//cv::initUndistortRectifyMap(_camera.Kl(), _camera.Dl(), _camera.Rl(), _camera.Pl().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1l, M2l);
	//cv::initUndistortRectifyMap(_camera.Kr(), _camera.Dr(), _camera.Rr(), _camera.Pr().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1r, M2r);


	cv::Mat imgLeft, imgRight, imgLeftRect, imgRightRect;
	SLAM::ReadWriter::StrList leftImgPathList;
	SLAM::ReadWriter::StrList rightImgPathList;
	if (!SLAM::ReadWriter::loadImageList(_imgInputDir, leftImgPathList, rightImgPathList))
	{
		std::cout << "Failed Loading image list" << std::endl;
		return EXIT_FAILURE;
	}
	unsigned int imgCount = leftImgPathList.size();


	for (int ni = 0; ni < imgCount; ni++)
	{
		imgLeft = cv::imread(leftImgPathList[ni], CV_LOAD_IMAGE_UNCHANGED);
		imgRight = cv::imread(rightImgPathList[ni], CV_LOAD_IMAGE_UNCHANGED);

		if (imgLeft.empty())
		{
			std::cerr << std::endl << "Failed to load image at: " << std::string(leftImgPathList[ni]) << std::endl;
			return 1;
		}

		if (imgRight.empty())
		{
			std::cerr << std::endl << "Failed to load image at: " << std::string(rightImgPathList[ni]) << std::endl;
			return 1;
		}

		//cv::remap(imgLeft, imgLeftRect, M1l, M2l, cv::INTER_LINEAR);
		//cv::remap(imgRight, imgRightRect, M1r, M2r, cv::INTER_LINEAR);
	

		//if (SLAM::Frame* newFrame=createFrame(imgLeftRect, imgRightRect))
		if (SLAM::Frame* newFrame = createFrame(imgLeft, imgRight))
		{	
			if (_tracker.tracking(*newFrame))
			{
				/////////////////自定义区/////////////////
				SLAM::Frame::FrameCompSet localFrames;
				KeyFrame* newKF = dynamic_cast<KeyFrame*>(newFrame);
				if (newKF)
				{
					std::ofstream ofs = std::ofstream(_workSpace + "posListBA.txt", std::ios::trunc | std::ios::in);
					localFrames = newKF->getLocalFrames();
					SLAM::FrameList localFrameList = SLAM::FrameList(localFrames.begin(), localFrames.end());
					if (!outputRt(localFrameList, ofs))
					{
						std::cout << "Failed outputing position file" << std::endl;
					}
				}
				else
				{
					newFrame->outputPos(ofile);
				}		
				///////////////////////////////////////////

				fprintf(stdout, "Tracking frame %d succeeded!\n", newFrame->getID());
			}
		}
		else
		{
			std::cout << "Failed creating frame!" << std::endl<<std::endl;
		}
	}
}



bool System::openPosFile()
{ 
	if (_posOutDir.back() != '\\')_posOutDir += '\\';
	std::string outputPath = _posOutDir + _posFileName;

	_posFile =std::ofstream(outputPath, std::ios::trunc | std::ios::in);
	if (!_posFile.is_open())
	{
		std::cout << "Failed opening the pos file to write frame position: " << outputPath << std::endl;
		return false;
	}
}



bool System::openP3dFile()
{
	std:string path = getWorkSpace() + "p3dList.txt";
	_p3dFile = std::ofstream(path, std::ios::trunc | std::ios::in);
	if (!_p3dFile.is_open())
	{
		std::cout << "Failed opening the pos file to write frame position: " << path << std::endl;
		return false;
	}
	return true;
}



bool System::outputRt(const FrameList& frameList, std::ofstream& ofs)
{
	if (!ofs.is_open())
		return false;

	if (frameList.empty())
		return true;

	//SLAM::FrameList::iterator itr = frameList.begin();
	//ofs << (*itr)->getID() << std::endl;

	for (SLAM::FrameList::const_iterator itr = frameList.begin(); itr != frameList.end(); itr++)
	{
		(*itr)->outputPos(ofs);
	}
	std::cout << "Outputting frames' position succeeded!" << std::endl;

	return true;
}



bool System::outputMpt(const MptList& mptList, std::ofstream& ofs)
{
	if (!ofs.is_open())
		return false;

	if (mptList.empty())
		return true;

	//SLAM::FrameList::iterator itr = frameList.begin();
	//ofs << (*itr)->getID() << std::endl;
	for (SLAM::MptList::const_iterator itr = mptList.begin(); itr != mptList.end(); itr++)
	{
		(*itr)->outputCoord(ofs);
	}
	ofs << "Outputting map points' world coord succeeded!"<<std::endl;

	return true;
}



bool System::outputMptDistList(const MptDistMap& mptDistMap, std::ofstream& ofs)
{
	if (!ofs.is_open())
		return false;

	if (mptDistMap.empty())
		return true;

	//SLAM::FrameList::iterator itr = frameList.begin();
	//ofs << (*itr)->getID() << std::endl;

	for (SLAM::MptDistMap::const_iterator itr = mptDistMap.begin(); itr != mptDistMap.end(); itr++)
	{
		ofs << std::setprecision(8) << itr->first << ": " << itr->second.first->getID() << " " << itr->second.second->getID() << std::endl;
	}
	std::cout << "Outputting map points' distance list succeeded!" << std::endl;

	return true;
}



bool System::saveGlobalMap(std::string filePath)
{
	if (!ReadWriter::saveMapFile(filePath, _globalMap))
	{
		std::cout << "Failed save map to file:" << filePath << std::endl;
		return false;
	}
	std::cout << "Saved map to file: " << filePath << std::endl;
	return true;
}



bool System::loadGlobalMap(std::string filePath)
{
	if (!ReadWriter::loadMapFile(filePath, *this))
	{
		std::cout << "Failed load map from file: " << filePath << std::endl;
		return false;
	}
	std::cout << "Loaded map from file: " << filePath << std::endl;
	return true;
}



void System::shutDown()
{
	if (!_localMapper.isFinished())
	{
		_localMapper.requestFinish();
		while (!_localMapper.isFinished())
			usleep(5000);
	}
	if (!_viewer.isFinished())
	{
		_viewer.requestFinish();
		while (!_viewer.isFinished())
			usleep(5000);
	}
	if (!_loopCloser.isFinished())
	{
		_loopCloser.requestFinish();
		while (!_loopCloser.isFinished())
			usleep(5000);
	}
	//pangolin::BindToContext("DotMarker Tracking: Map Viewer");
}





bool System::outPutPlaneFittingError(const std::string& cloudFilePath, const std::string& filePath)
{
	ifstream ifs(cloudFilePath);
	double x, y, z;
	std::vector<cv::Point3d> p3dList;
	while (!ifs.eof())
	{
		ifs >> x >> y >> z;
		cv::Point3d p3d(x, y, z);
		p3dList.emplace_back(p3d);
		//cout << points.back() << endl;
	}

	std::ofstream ofs = std::ofstream(filePath, std::ios::trunc | std::ios::in);
	if (!ofs.is_open())
		return false;

	cv::Mat mptCloud(cv::Size(3, p3dList.size()), CV_64F);
	int r = 0;
	for (std::vector<cv::Point3d>::iterator itr = p3dList.begin(); itr != p3dList.end(); itr++, r++)
	{
		mptCloud.at<double>(r, 0) = itr->x;
		mptCloud.at<double>(r, 1) = itr->y;
		mptCloud.at<double>(r, 2) = itr->z;
	}
	cv::Vec4d planeParams;
	planeFitting(mptCloud, planeParams);
	double mean; 
	double RMSE;
	calcPlaneFitError( mptCloud, mean, RMSE);
	ofs << "Mean of error:" << mean << endl;
	ofs << "RMSE of error:" << RMSE << endl;
}



void System::planeFitting(const cv::Mat& pointCloud, cv::Vec4d& planeParam)
{
	cv::Mat copy = pointCloud.clone();

	cv::Scalar mean1, mean2, mean3;
	cv::Mat datacol1, datacol2, datacol3;
	datacol1 = copy.colRange(0, 1).clone();
	datacol2 = copy.colRange(1, 2).clone();
	datacol3 = copy.colRange(2, 3).clone();
	mean1 = mean(datacol1);
	mean2 = mean(datacol2);
	mean3 = mean(datacol3);

	for (int i = 0; i < copy.rows; i++)
	{
		copy.at<double>(i, 0) = copy.at<double>(i, 0) - mean1[0];
		copy.at<double>(i, 1) = copy.at<double>(i, 1) - mean2[0];
		copy.at<double>(i, 2) = copy.at<double>(i, 2) - mean3[0];
	}
	//cout << "\ninput - mean:\n" << pointCloud << endl;
	cv::Mat U, W, V;
	cv::SVDecomp(copy, W, U, V);//W特征值，U左奇异值矩阵，V右奇异值矩阵
	double a, b, c, d;
	//以下具体求解见SVD拟合平面的matlab程序//注意！！！opencv的SVDecomp函数所得U和V与MATLAB中svd所得U与V互为转置！！！！因此此处应取矩阵V的第三行作为法向量！！！！！！
	a = V.at<double>(2, 0);
	b = V.at<double>(2, 1);
	c = V.at<double>(2, 2);
	d = -(a*mean1[0] + b*mean2[0] + c*mean3[0]);
	//激光平面在相机坐标系下方程为x+b/ay+c/az+d/a=0
	if (a != 0)
	{
		planeParam.val[0] = 1;
		planeParam.val[1] = b / a;
		planeParam.val[2] = c / a;
		planeParam.val[3] = d / a;
	}
	else
	{
		planeParam.val[0] = a;
		planeParam.val[1] = b;
		planeParam.val[2] = c;
		planeParam.val[3] = d;
	}

	//cout << "SVD fit plane:" << planeParam <<"\n"<< endl;
}



void System::calcPlaneFitError(const cv::Mat cloud, double& mean, double& RMSE)
{
	cv::Vec4d planeParam;
	planeFitting(cloud, planeParam);
	double a = planeParam[0];
	double b = planeParam[1];
	double c = planeParam[2];
	double d = planeParam[3];
	//由于SVD分解奇异值矩阵为正交矩阵，因此a,b,c正交，deno必为1
	double deno = 1;
	deno = sqrt(a*a + b*b + c*c);
	double err2 = 0;
	double err = 0;
	double totalErr = 0.;
	double totalErr2 = 0.;
	for (int i = 0; i < cloud.rows; i++)
	{
		double x = cloud.at<double>(i, 0);
		double y = cloud.at<double>(i, 1);
		double z = cloud.at<double>(i, 2);
		double nomi = fabs(a*x + b*y + c*z + d);
		err = nomi / deno;
		err2 = err*err;
		totalErr += err;
		totalErr2 += err2;
	}
	mean = totalErr / cloud.rows;
	RMSE = sqrt(totalErr2 / cloud.rows);
	//cv::meanStdDev(error, mean, stddev);
	//std::cout << "Mean of error:" << mean << endl;
	//std::cout << "Stddev of error:" << stddev << endl;	
}



bool System::calcFramesRefShiftRMSE(const FrameList& frameList, Frame& refFrame,double& RMSE)
{
	if (frameList.empty())
		return false;
	double shiftTotal = .0;
	double shiftVal = .0;
	for (FrameList::const_iterator fItr = frameList.begin(); fItr != frameList.end(); fItr++)
	{
		if ((*fItr) == &refFrame)
			continue;
		cv::Mat shiftMat = (*fItr)->getCameraCenter()-refFrame.getCameraCenter();
		shiftTotal += shiftMat.dot(shiftMat);
	}
	RMSE = sqrt(shiftTotal / (double)frameList.size());
}


bool System::calcFramesShiftRMSE(const FrameList& frameList, double& RMSE)
{
	if (frameList.empty())
		return false;
	cv::Mat meanPos=cv::Mat::zeros(3,1,CV_64F);
	for (FrameList::const_iterator fItr = frameList.begin(); fItr != frameList.end(); fItr++)
	{
		meanPos += (*fItr)->getCameraCenter();
	}
	meanPos /= frameList.size();

	double shiftTotal = .0;
	for (FrameList::const_iterator fItr = frameList.begin(); fItr != frameList.end(); fItr++)
	{
		cv::Mat shiftMat = (*fItr)->getCameraCenter() - meanPos;
		shiftTotal += shiftMat.dot(shiftMat);
	}
	RMSE = sqrt(shiftTotal / (double)frameList.size());
	return true;
}


bool System::calcFramesRotateRMSE(const FrameList& frameList, double& RMSE)
{
	if (frameList.empty())
		return false;
	Eigen::Vector3d meanAngles(0.,0.,0.);
	std::vector<Eigen::Vector3d> anglesList;
	for (FrameList::const_iterator fItr = frameList.begin(); fItr != frameList.end(); fItr++)
	{
		cv::Mat& Rwc = (*fItr)->getRotationInverse();
		Eigen::Matrix3d rotationMatrix = Converter::toMatrix3d(Rwc);
		Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0).transpose();
		eulerAngles = eulerAngles / PI * 180.;
		for (int i = 0; i < eulerAngles.size(); i++)
		{
			if (eulerAngles[i] < 0.) eulerAngles[i] = -eulerAngles[i];
			eulerAngles[i] = 180. - eulerAngles[i]>90. ? eulerAngles[i] : 180. - eulerAngles[i];
		}
		anglesList.emplace_back(eulerAngles);
		meanAngles += eulerAngles;
	}
	meanAngles /= frameList.size();

	double angleTotal = 0.;
	double angleVal = 0.;

	for (std::vector<Eigen::Vector3d>::const_iterator aItr = anglesList.begin(); aItr != anglesList.end(); aItr++)
	{
		Eigen::Vector3d  angleMat = *aItr - meanAngles;
		angleTotal += angleMat.dot(angleMat);
	}

	RMSE = sqrt(angleTotal / (double)frameList.size());
}


void System::clearBadFrames()
{
	Frame::FrameCompSet frameSet = _globalMap.getBadFrames();
	for (auto fBadItr = frameSet.begin(); fBadItr != frameSet.end(); fBadItr++)
	{
		Frame* pFrameBad = *fBadItr;
		assert(pFrameBad->isBad());

		if (pFrameBad->isNotToErase())
			continue;

		std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//避免主线程改变 _refKF
		if (pFrameBad == _tracker._refFrame)
			continue;//跳过主线程参考帧

		KeyFrame* pKFBad = dynamic_cast<KeyFrame*>(pFrameBad);

		if (pKFBad)
		{
			if (pKFBad == _tracker._refKF)
				continue;//跳过主线程参考关键帧

			auto& _keyFrameQueue = _localMapper.getKFQueue();
			auto itr1 = std::find(_keyFrameQueue.begin(), _keyFrameQueue.end(), pKFBad);
			if (itr1 != _keyFrameQueue.end())
				_keyFrameQueue.erase(itr1);//删除局部BA队列剩余bad关键帧，避免内存清空后被取出

			auto& loopKFQueue = _loopCloser.getKFQueue();
			auto itr2 = std::find(loopKFQueue.begin(), loopKFQueue.end(), pKFBad);
			if (itr2 != loopKFQueue.end())
				loopKFQueue.erase(itr2);//删除回环检测队列剩余bad关键帧，避免内存清空后被取出
		}

		//_globalMap->getBadFrames().erase(pFrameBad);
		_globalMap.eraseFrame(pFrameBad);
	}
}


}