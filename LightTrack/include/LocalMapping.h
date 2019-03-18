#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H
#include <list>
#include <mutex>

#include "KeyFrame.h"
#include "LoopClosing.h"
#include "GlobalMap.h"


namespace SLAM
{

class System;

class LocalMapping
{
	friend class Tracker;
	friend class GlobalMap;
	friend class LoopClosing;
	friend class Optimizer;

public:
	typedef std::list<KeyFrame*> KFQueue;

public:
	LocalMapping():
		_globalMap(nullptr),
		_system(nullptr),
		_tracker(nullptr),
		_bAcceptKeyFrames(true),
		_bFinished(true),
		_bStopped(false),
		_bFinishRequested(false),
		_bStopRequested(false),
		_bAbortBA(false),
		_localMappingThread(nullptr)
	{};
	~LocalMapping() {};

	void setTracker(Tracker* tracker) { _tracker = tracker; }
	Tracker* getTracker() { return _tracker; }

	void setGlobalMap(GlobalMap* map) { _globalMap = map; }
	GlobalMap* getGlobalMap() { return _globalMap; }

	void setSystem(System* const  system) { _system = system; }
	System* getSystem() { return _system; }

	void setLoopCloser(LoopClosing* loopCloser) { _loopCloser = loopCloser; }
	LoopClosing* getLoopCloser() { return _loopCloser; }

	std::thread* getLocalMappingThreadPtr() { return _localMappingThread; }

	bool startThread()
	{
		_localMappingThread = new std::thread(&LocalMapping::run, this);
		if (!_localMappingThread)
			return false;
		return true;
	}

	void abortBA() { _bAbortBA = true; }

	bool isAcceptKeyFrames()
	{
		std::unique_lock<std::mutex> lock(_mutexAccept);
		return _bAcceptKeyFrames;
	}

	void setAcceptKeyFrames(bool flag)
	{
		std::unique_lock<std::mutex> lock(_mutexAccept);
		_bAcceptKeyFrames = flag;
	}

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
	void LocalMapping::requestStop()
	{
		std::unique_lock<std::mutex> lock(_mutexStop);
		_bStopRequested = true;
		std::unique_lock<std::mutex> lock2(_mutexNewKFs);
		_bAbortBA = true;
	}
	bool LocalMapping::stop()
	{
		std::unique_lock<std::mutex> lock(_mutexStop);
		if (_bStopRequested )
		{
			_bStopped = true;
			std::cout << "Local Mapping STOP" << std::endl;
			return true;
		}

		return false;
	}
	bool isStopped()
	{
		std::unique_lock<std::mutex> lock(_mutexStop);
		return _bStopped;
	}

	bool stopRequested()
	{
		std::unique_lock<std::mutex> lock(_mutexStop);
		return _bStopRequested;
	}

	void setFinish()
	{
		std::unique_lock<std::mutex> lock(_mutexFinish);
		_bFinished = true;
		_bStopped = true;
	}

	bool isFinished()
	{
		std::unique_lock<std::mutex> lock(_mutexFinish);
		return _bFinished;
	}

	void globalMptCulling();


	void keyFrameCulling(KeyFrame* keyFrame);


	KFQueue& getKFQueue() 
	{ 	
		std::unique_lock<std::mutex> lock(_mutexNewKFs);
		return _keyFrameQueue;
	}
	void addKeyFrame(KeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(_mutexNewKFs);
		_keyFrameQueue.emplace_back(pKF);
	}
	int keyframesInQueue() 
	{
		std::unique_lock<std::mutex> lock(_mutexNewKFs);
		return _keyFrameQueue.size();
	}


	bool checkNewKeyFrames()
	{
		std::unique_lock<std::mutex> lock(_mutexNewKFs);

		return(!_keyFrameQueue.empty());
	}

	KeyFrame*  proccessNewKeyFrame();

	void run();

	void release();

private:
	 
	GlobalMap*              _globalMap;
	System*                 _system;
	LoopClosing*            _loopCloser;
	Tracker*                _tracker;

	KFQueue					_keyFrameQueue;


	std::mutex				_mutexCout;
	std::mutex				_mutexNewKFs;
	std::mutex				_mutexAccept;
	std::mutex	            _mutexFinish;
	std::mutex	            _mutexStop;

	bool					_bAcceptKeyFrames;
	bool					_bFinished;
	bool					_bStopped;
	bool				    _bFinishRequested;
	bool					_bStopRequested;
	bool					_bAbortBA;

	std::thread*			_localMappingThread;
};




}


#endif 