#ifndef SYSTEM_H
#define SYSTEM_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <map>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include "GlobalMap.h"
#include "Tracker.h"
#include "Point.h"
#include "impl\Point.hpp"
#include "Drawer.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "ReadWriter.h"
#include "Camera.h"
#include "LoopClosing.h"
#include "Optimizer.h"


#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_TRACK_EXPORTS
#define SYSTEM_API  __declspec(dllexport)
#else
#define SYSTEM_API  __declspec(dllimport)
#endif
#else
#define SYSTEM_API
#endif


namespace SLAM{


void  __declspec(dllexport) usleep(__int64 usec);


class SYSTEM_API System
	//class  System
{
	friend class Tracker;
public:
	struct Statistic
	{
		double meanVal = 0;
		double minVal = 0;
		double maxVal = 0;
		double meanSquare = 0;
		double variance = 0;
		double medianVal = 0;
	};


private:
	System() :
		_workSpace(""),
		_calibImgDir("F:\\Data\\CALIBRATE"),
		_calibDataDir("F:\\Data\\CALIBRATE"),
		_imgInputDir("F:\\Data\\TEST"),//ÐèÐÞ¸Ä
		_posOutDir("D:\\"),
		_posFileName("pos.txt"),
		_p3dOutPath("D:\\p3dList.txt"),
		_stereoImgOutDir("F:\\Data\\TEST\\output2D\\"),
		_TrackImgOutDir("F:\\Data\\TEST\\outputTrack\\"),
		_bOutputP3dList(false),
		_bOutputPos(false),
		_bOutputTrackImg(false),
		_bOutputStereoImg(false),
		_bOptimize3D(true),
		_bLocalBundleAdjustment(true),
		_bDisplay3DModel(true),
		_bDispOptimizeStats(false),
		_bLoopClosing(true),
		_bStopped(false),
		_calcMatchedType(Frame::NORM),
		_extractMarkType(Feature::SIMPLE),
		_orientType(Matcher_<>::UNIT_QUAT),
		_calcSim3Type(Matcher_<>::SUCCESSIVE),
		_matchedKeysType(Matcher_<>::UNPROJ)
	{}
	~System() {}

	class Garbo
	{
	public:
		
		~Garbo()
		{
			if(System::_instance)
			{
				_instance->release();
			}
		};
	};

	 
public:
	static System* instance()
	{
		if (!_instance)
			_instance = new System;
		return _instance;
	}

	const System& system() { return *this; }

	void release()
	{
		_globalMap.release();
		delete _instance;
		_instance = nullptr;
	}

	std::mutex& laserMutex() { return GlobalMap::laserMutex(); }

	void setMaxEdgeMSE(const double& val) { Optimizer::MAX_EDGE_RMSE = val; }
	void setMaxEdgeError(const double& val) { Optimizer::MAX_EDGE_ERROR = val; }
	void setMaxBAEdgeError(const double& val) { Optimizer::MAX_BA_EDGE_ERROR = val; }
	
	void setStopped(const bool& flag) { _bStopped = flag; }
	bool isStopped() { return _bStopped; }

	void setDispErrorStats(const bool& flag) { _bDispOptimizeStats = flag; }
	const bool& isDispErrorStats() { return _bDispOptimizeStats; }

	void setOutputP3dFlag(const bool& flag) { _bOutputP3dList = flag; }
	const bool& isOutputP3d() { return _bOutputP3dList; }

	void setOutputPosFlag(const bool& flag) { _bOutputPos = flag; }
	const bool& isOutputPos() { return _bOutputPos; }

	void setOptimize3DFlag(const bool& flag) { _bOptimize3D = flag; }
	const bool& isOptimize3D() { return _bOptimize3D; }

	void setLocalBundleAdjustmentFlag(const bool& flag) { _bLocalBundleAdjustment = flag; }
	const bool& isLocalBundleAdjustment() { return _bLocalBundleAdjustment; }

	void setLoopClosingFlag(const bool& flag) { _bLoopClosing = flag; }
	const bool& isLoopClosing() { return _bLoopClosing; }

	void setDisplay3DModelFlag(const bool& flag) { _bDisplay3DModel = flag; }
	const bool& isDisplay3DModel() { return _bDisplay3DModel; }

	void setOutputStereoImgFlag(const bool& flag) { _bOutputStereoImg = flag; }
	const bool& isOutputStereoImg() { return _bOutputStereoImg; }

	void setOutputTrackImgFlag(const bool& flag) { _bOutputTrackImg = flag; }
	const bool& isOutputTrackImg() { return _bOutputTrackImg; }

	const std::string& getWorkSpace() { return _workSpace; }
	void setWorkSpace(const std::string dir) { _workSpace = dir; }

	const std::string& getStereoImgOutDir() { return _stereoImgOutDir; }
	void setStereoImgOutDir(const std::string dir) { _stereoImgOutDir = dir; }

	const std::string& getTrackImgOutDir() { return _TrackImgOutDir; }
	void setTrackImgOutDir(const std::string dir) { _TrackImgOutDir = dir; }

	void setCalibImgDir(const std::string dir) { _calibImgDir = dir; }
	std::string& getCalibImgDir() { return _calibImgDir; }

	void setCalibDataDir(const std::string dir) { _calibDataDir = dir; }
	std::string& getCalibDataDir() { return _calibDataDir; }

	void setImgInputDir(const std::string dir) { _imgInputDir = dir; }
	std::string& getImgInputDir() { return _imgInputDir; }

	void setPosOutDir(const std::string dir) { _posOutDir = dir; }
	std::string& getPosOutDir() { return _posOutDir; }

	void setP3dOutPath(const std::string dir) { _p3dOutPath = dir; }
	std::string& getP3dOutPath() { return _p3dOutPath; }

	void setPosFileName(const std::string dir) { _posFileName = dir; }
	std::string& getPosFileName() { return _posFileName; }

	void setExtractMarkType(const Feature::ExtractMarkType type) { _extractMarkType = type; }
	const Feature::ExtractMarkType& getExtractMarkType() {return _extractMarkType;}

	void setCalcMatchedType(const Frame::CalcMatchedType type) { _calcMatchedType = type; }
	const Frame::CalcMatchedType& getCalcMatchedType() { return _calcMatchedType; }

	void setOrientType(const Matcher_<>::OrientFuncType type) { _orientType = type; }
	const Matcher_<>::OrientFuncType& getOrientType() {return _orientType;}

	void setCalcSim3Type(const Matcher_<>::CalcSim3FuncType type) { _calcSim3Type = type; }
	const Matcher_<>::CalcSim3FuncType& getCalcSim3Type() { return _calcSim3Type; }

	void setMatchedKeysType(const Matcher_<>::MatchedKeysFuncsType type) { _matchedKeysType = type; }
	const Matcher_<>::MatchedKeysFuncsType& getMatchedKeysType() { return _matchedKeysType; }
		
	Drawer& getMapDrawer() { return _mapDrawer; }

	Viewer& getViewer() { return _viewer; }

	Tracker& getTracker() { return _tracker; }

	GlobalMap& getGlobalMap() { return _globalMap; }

	Camera& getCamera() { return _camera; }

	LocalMapping& getlocalMapper() { return _localMapper; }

	LoopClosing& getLoopCloser() { return _loopCloser; }

	FrameList& getGlobalFrameList() { return _globalMap.getFrameList(); }
	void addFrame(Frame* frame) { _globalMap.addFrame(frame); }

	std::ofstream& p3dOfstream() { return _p3dFile; }

	void addLaserPtCloud(const cv::Mat& ptCloud) { _globalMap.addLaserPtCloud(ptCloud); }

	bool calibrate();

	bool loadCalibParams();

	bool initiate();

	void undistortImage(const cv::Mat& imgLeft, const cv::Mat& imgRight, cv::Mat& imgRectLeft, cv::Mat& imgRectRight);

	Frame* createImgFrame(const cv::Mat& imgLeft, const cv::Mat& imgRight, bool isUndistored = false);

	Frame* createFrame(const cv::Mat& imgLeft, const cv::Mat& imgRight, bool isUndistored = false);

	Frame* createFrame(const unsigned int& index, const cv::Mat& imgLeft, const cv::Mat& imgRight, bool isUndistored = false);

	cv::Mat trackFrame(const unsigned int& index, const cv::Mat& imgLeft, const cv::Mat& imgRight);

	cv::Mat trackFrame(Frame& frame);

	bool slamRun();

	bool openP3dFile();

	bool openPosFile();

	bool outputRt(const FrameList& frameList, std::ofstream& ofs);

	bool outputMpt(const MptList& mptList, std::ofstream& ofs);

	bool outputMptDistList(const MptDistMap& mptDistMap, std::ofstream& ofs);

	void loadStageCamera(std::string calibFilePath);

	void loadHandHeldCamera(std::string calibFilePath);

	void shutDown();

	bool saveGlobalMap(std::string filePath);

	bool loadGlobalMap(std::string filePath);

	void calcFrameMatchedRMSE(GlobalMap* pMap, int* matchedRMSEHist, const int& len, const double& max, Statistic& matchedRMSEStats);

	void calcKFMatchedRMSE(GlobalMap* pMap, int* matchedRMSEHist, const int& len, const double& max, Statistic& matchedRMSEStats);

	void calcMarkEccentHisto(Frame* frame, int* EccHist, const int& len, const double& max, Statistic& eccStats);

	void calcMarkAreaHisto(Frame* frame, int* areaHist, const int& len, const double& max, Statistic& areaStats);

	void calcPlaneFittingHisto(int* fitRMSEHist, const int& len, const double& maxVal, Statistic& fitRMSEStats);

	void drawStatsHisto(cv::Mat& outImg, int* Histo, const int& len, const double& maxX, Statistic* pStats = nullptr);

	void planeFitting(const cv::Mat& input, cv::Vec4d &planeParam);

	void calcPlaneFitError(const cv::Mat cloud, double& mean, double& RMSE);

	bool outPutPlaneFittingError(const std::string& cloudFilePath, const std::string& filePath);

	bool calcFramesRefShiftRMSE(const FrameList& frameList, Frame& refFrame, double& RMSE);

	bool calcFramesShiftRMSE(const FrameList& frameList, double& RMSE);

	bool calcFramesRotateRMSE(const FrameList& frameList, double& RMSE);

	void clearBadFrames();



private:


	Drawer									_mapDrawer;
	Viewer									_viewer;
	Tracker									_tracker;
	GlobalMap								_globalMap;
	Camera									_camera;
	LocalMapping							_localMapper;
	LoopClosing								_loopCloser;

	std::string								_workSpace;
	std::string								_calibImgDir;
	std::string								_calibDataDir;
	std::string								_imgInputDir;
	std::string								_posOutDir;
	std::string								_posFileName;
	std::string								_p3dOutPath;
	std::string								_stereoImgOutDir;
	std::string								_TrackImgOutDir;

	bool									_bDisplay3DModel;
	bool									_bOutputTrackImg;
	bool									_bOutputStereoImg;
	bool									_bOptimize3D;
	bool									_bLocalBundleAdjustment;
	bool									_bLoopClosing;
	bool									_bOutputPos;
	bool									_bOutputP3dList;
	bool									_bDispOptimizeStats;
	bool									_bStopped;

	std::ofstream							_posFile;
	std::ofstream							_p3dFile;

	Frame::CalcMatchedType					_calcMatchedType;
	Feature::ExtractMarkType				_extractMarkType;
	Matcher_<>::OrientFuncType				_orientType;
	Matcher_<>::CalcSim3FuncType			_calcSim3Type;
	Matcher_<>::MatchedKeysFuncsType		_matchedKeysType;

	static System*							_instance;
	static Garbo				   			_garbo;
};


}



#endif // SYSTEM_H