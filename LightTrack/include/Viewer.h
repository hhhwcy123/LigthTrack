#ifndef VIEWER_H
#define VIEWER_H


#include "Drawer.h"
#include "Tracker.h"


#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_TOOL_EXPORTS
#define VIEWER_API  __declspec(dllexport)
#else
#define VIEWER_API  __declspec(dllimport)
#endif
#else
#define VIEWER_API
#endif

using namespace pangolin;

namespace SLAM
{
#define UV_MINDIFF 50

class System;
class GlobalMap;

class HandlerPick3D :public pangolin::Handler3D
{
public:
	typedef std::vector<MapPoint3d*> MptList;
	typedef std::pair<MapPoint3d*, MapPoint3d*> MptPair;
	typedef std::vector<MptPair> MptPairList;

	enum State
	{
		Indle,
		Selecting
	};
public:

	MptPairList& getMptPairSel() { return _mptSelPairList; }

	HandlerPick3D(OpenGlRenderState& cam_state, GlobalMap* globalMap) :Handler3D(cam_state), _globalMap(globalMap), _state(Indle){};

	bool pick3DPoint(View& display, int x, int y);

	void Mouse(View&display , MouseButton button, int x, int y, bool pressed, int button_state);

	void XYZProjectUV(cv::Mat& projMat, cv::Mat& modelViewMat, cv::Mat& view, const cv::Point3d& p3D, cv::Point2d& p2D);

//	void MouseMotion(View&, int x, int y, int button_state);

private:
	GlobalMap*					 _globalMap;
	MptPairList                  _mptSelPairList;
	MptPair						 _mptSelPair;
	State						 _state;
	static bool					 _isFistP3d;

	
};
class VIEWER_API Viewer
{

public:
	Viewer() :
		_drawer(nullptr),
		_viewpointX(0),
		_viewpointY(-0.7),
		_viewpointZ(-150),
		_viewpointF(1000),
		_isFinishRequested(false),
		_isStopRequested(false),
		_isStopped(true),
		_system(nullptr),
		_viewThread(nullptr)
	{}
	~Viewer() 
	{}

	void setSystem(System* const  system) { _system = system; }
	System* getSystem() { return _system; }

	void setDrawer(Drawer* const  drawer) { _drawer = drawer; }
	Drawer* getDrawer() { return _drawer; }

	void setTracker(Tracker* const tracker) { _tracker = tracker; }
	Tracker* getTracker() { return _tracker; }

	void setHandler(pangolin::Handler* const  handler) {_handler = handler;}
	pangolin::Handler* getHandler() { return _handler; }

	std::thread* getViewThreadPtr() { return _viewThread; }

	bool startThread()
	{
		_viewThread = new std::thread(&Viewer::run, this);
		if (!_viewThread)			
			return false;
		return true;
	}

	void run();

	void requestFinish()
	{
		_isFinishRequested = true;
	}

	bool checkFinish()
	{
		return _isFinishRequested;
	}

	void setFinish(){ _isFinished = true;}

	bool isFinished(){return _isFinished;}

	void requestStop()
	{
		if (!_isStopped)
			_isStopRequested = true;
	}

	bool isStopped(){return _isStopped;}

	bool stop()
	{
		if (_isFinishRequested)
			return false;
		else if (_isStopRequested)
		{
			_isStopped = true;
			_isStopRequested = false;
			return true;
		}

		return false;

	}

	void release()
	{
		_isStopped = false;
		_isStopRequested = false;
	}


private:

	System*					_system;
	Drawer*					_drawer;
	Tracker*				_tracker;
	double					_viewpointX;
	double					_viewpointY;
	double					_viewpointZ;
	double					_viewpointF;

	bool					_isFinishRequested;
	bool					_isFinished;

	bool					_isStopped;
	bool					_isStopRequested;

	std::thread*			_viewThread;

	pangolin::Handler*	    _handler;

};

}


#endif//VIEWER_H
