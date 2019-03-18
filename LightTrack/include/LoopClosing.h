#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <list>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include "KeyFrame.h"


#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_TRACK_EXPORTS
#define LOOPCLOSING_API  __declspec(dllexport)
#else
#define LOOPCLOSING_API  __declspec(dllimport)
#endif
#else
#define LOOPCLOSING_API
#endif


namespace SLAM
{

class System;
class GlobalMap;
class LocalMapping;


class LOOPCLOSING_API LoopClosing
{
	friend class Drawer;

public:
	typedef std::list<KeyFrame*> KFQueue;
	typedef std::map<KeyFrame*, cv::Mat, std::less<KeyFrame*>/*,
		    Eigen::aligned_allocator<std::pair<const KeyFrame*, cv::Mat> >*/ > KeyFrameAndPose;

	LoopClosing() :
		_globalMap(nullptr),
		_system(nullptr),
		_localMapper(nullptr),
		_bFinished(false),
		_bFinishRequested(false),
		//_lastLoopKFId(0),
		_currentKF(nullptr),
		_loopKF(nullptr),
		_backKF(nullptr),
		_lastKF(nullptr),
		_loopClosingThread(nullptr)
	{}

	~LoopClosing(){  }

	static bool pKFComparator(const KeyFrame* pKF1, const KeyFrame* pKF2) { return *pKF1 < *pKF2; }


	void setBackKF(KeyFrame* pKF) { _backKF = pKF; }
	void setLoopKF(KeyFrame* pKF) { _loopKF = pKF; }
	void setLastKF(KeyFrame* pKF) { _lastKF = pKF; }

	void setGlobalMap(GlobalMap* map) { _globalMap = map; }
	GlobalMap* getGlobalMap() { return _globalMap; }

	void setSystem(System* const  system) { _system = system; }
	System* getSystem() { return _system; }

	void setLocalMapper(LocalMapping* localMapper) { _localMapper = localMapper; }
	LocalMapping* getLocalMapper() { return _localMapper; }

	std::thread* getLoopClosingThreadPtr() { return _loopClosingThread; }

	bool startThread()
	{
		_loopClosingThread = new std::thread(&LoopClosing::run, this);
		if (!_loopClosingThread)	
			return false;
		return true;
	}
	

	KFQueue& getKFQueue() 
	{
		std::unique_lock<std::mutex> lock(_mutexLoopQueue);
		return _keyFrameQueue; 
	}
	
	void addKeyFrame(KeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(_mutexLoopQueue);
		_keyFrameQueue.emplace_back(pKF);
	}

	int keyframesInQueue()
	{
		std::unique_lock<std::mutex> lock(_mutexLoopQueue);
		return _keyFrameQueue.size();
	}



	bool checkNewKeyFrames()
	{
		std::unique_lock<std::mutex> lock(_mutexLoopQueue);
		return(!_keyFrameQueue.empty());
	}

	void runGlobalBundleAdjustment();

	void runGlobalPoseCorrect();

	void run();

	bool detectLoop();

	bool findLoopForKF(KeyFrame* pKF);

	bool loopCorrect();

	void requestFinish()
	{
		std::unique_lock<std::mutex> lock(_mutexFinish);
		_bFinishRequested = true;
	}

	bool checkFinish()
	{
		std::unique_lock<std::mutex> lock(_mutexFinish);
		return _bFinishRequested;
	}

	void setFinish()
	{
		std::unique_lock<std::mutex> lock(_mutexFinish);
		_bFinished = true;
	}

	bool isFinished()
	{
		std::unique_lock<std::mutex> lock(_mutexFinish);
		return _bFinished;
	}

	

private:
	GlobalMap*              _globalMap;
	System*                 _system;
	LocalMapping*			_localMapper;

	KFQueue					_keyFrameQueue;

	//int					_lastLoopKFId;
	KeyFrame*               _currentKF;
	KeyFrame*               _loopKF;
	KeyFrame*               _backKF;
	KeyFrame*				_lastKF;
	KeyFrame::KFCompSet     _backKFsSet;
	KeyFrameList	        _condidateLoopKFs;
	cv::Mat					_TcwBack;//匹配回环帧->当前帧的变换矩阵
	
	std::mutex              _mutexLoopQueue;
	std::mutex  			_mutexFinish;
	std::mutex              _mutexGBA;
	bool					_bFinishRequested;
	bool					_bFinished;

	std::thread*			_loopClosingThread;
};	


}


#endif 