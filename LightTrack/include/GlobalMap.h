#ifndef MAP_H
#define MAP_H


#include <set>
#include <map>
#include <mutex>

#include "Point.h"
#include "impl\Point.hpp"
#include "KeyFrame.h"
#include "ReadWriter.h"


#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_ELEM_EXPORTS
#define MAP_API  __declspec(dllexport)
#else
#define MAP_API  __declspec(dllimport)
#endif
#else
#define MAP_API
#endif


namespace SLAM
{


class MAP_API GlobalMap
{

public:

	typedef std::vector<cv::Mat> MatList;
	
	friend class LocalMapping;
	friend class Tracker;
	MatcherTemplate
	friend class Matcher_;
	friend class Frame;
	friend class Optimizer;
	friend class Drawer;
	friend class KeyFrame;
	friend class ReaderWriter;
	friend class HandlerPick3D;
	friend class LoopClosing;

public:
	GlobalMap() { /*_hEvent = CreateEvent(NULL, false, true, NULL);*/}
	~GlobalMap();

	void release(); 
	

	//HANDLE mutableEvent() { return _hEvent; }
	static std::mutex& mptMutex(){ return _mptMutex; }
	static std::mutex& frameMutex(){ return _frameMutex; }
	static std::mutex& laserMutex(){ return _laserMutex; }
	/*
	void insertFrame( Frame* const pF)
	{ 
		WaitForSingleObject(_hEvent, INFINITE); 
		_frameSet[pF->getID()] = pF; 
		_frameCount++; 
		SetEvent(_hEvent);
	}
	FrameMapSet& getFrameSet() { return _frameSet; }*/

	static void setMptCount(const unsigned& val) { _mapPointCount = val; }
	static unsigned int& mptCount() { return _mapPointCount; }

	static void setFrameCount(const unsigned& val) { _frameCount = val; }
	static unsigned int& frameCount() { return _frameCount; }

	static void setKeyFrameCount(const unsigned& val) { _keyFrameCount = val; }
	static unsigned int& keyFrameCount() { return _keyFrameCount; }

	MptSet& getBadMpts() { return _badMptSet; }
	void insertBadMpt(MapPoint3d* mpt) { _badMptSet.insert(mpt); }

	Frame::FrameCompSet& getBadFrames() { return _badFrameSet; }
	void insertBadFrame(Frame* pF) { _badFrameSet.insert(pF); }

	FrameList& getFrameList() {return _frameList;}
	void addFrame(Frame* const pF)
	{
		std::unique_lock<std::mutex> lockFrame(_frameMutex);
		//WaitForSingleObject(_hEvent, INFINITE);
		_frameList.emplace_back(pF);
		//SetEvent(_hEvent);
	}


	KeyFrameList& getKeyFrameList() {return _keyFrameList; }
	void addKeyFrame(KeyFrame* const pKF)
	{
		std::unique_lock<std::mutex> lockFrame(_frameMutex);
		//WaitForSingleObject(_hEvent, INFINITE);
		_keyFrameList.emplace_back(pKF);
		_keyFrameCount++;
		//SetEvent(_hEvent);
	}

	MptList& getMapPointList() { return _mapPointList; }
	void addMapPoint(MapPoint3d* const mapPoint) 
	{
		std::unique_lock<std::mutex> lockMpt(_mptMutex);
		//WaitForSingleObject(_hEvent, INFINITE);
		_mapPointList.emplace_back(mapPoint);
		//SetEvent(_hEvent);
	}

	MptDistMap& getMptDistMap() {return _mptDistMap; }
	
	void eraseMapPoint(MapPoint3d* mapPoint);

	bool eraseMapPoint(const int& id);

	bool eraseFrame(Frame* pF);

	bool eraseKeyFrame(KeyFrame* pKF);

	void setReferenceMapPoints(const MptList& mapPointList) 
	{
		std::unique_lock<std::mutex> lockMpt(_mptMutex);
		//WaitForSingleObject(_hEvent, INFINITE);
		_refMapPointList = mapPointList; 
		//SetEvent(_hEvent);
	}
	MptList& getReferenceMapPoints() {	return _refMapPointList; }

	void calcMptDistList(MapPoint3d* mpt, MptDistMap& mptDistMap);

	void addMptDistList(MapPoint3d* mpt,double th=0);

	void updateMptDistList(MapPoint3d* mpt,double th);

	void deleteMptDist(MapPoint3d* mpt, double th);

	void addLaserPtCloud(const cv::Mat ptCloud) 
	{
		std::unique_lock<std::mutex> laserLock(_laserMutex);
		_laserPtClouds.emplace_back(ptCloud);
	}

	MatList& getLaserPtCloud() { return _laserPtClouds; }

	bool searchAndFuse(MapPoint3d* mpt, std::set<MapPoint3d*>& mptSet);

private:

	MptList				        _mapPointList;
	FrameList				    _frameList;
	KeyFrameList			    _keyFrameList;
	Frame::FrameCompSet         _badFrameSet;
	MptSet						_badMptSet;
	MptList			            _refMapPointList;
	MptDistMap                  _mptDistMap;
	MatList					    _laserPtClouds;

	static unsigned				_frameCount;
	static unsigned				_keyFrameCount;
	static unsigned             _mapPointCount;

	//HANDLE						_hEvent;
	static std::mutex			_frameMutex;
	static std::mutex			_mptMutex;
	static std::mutex           _laserMutex;



};


} //namespace SLAM

#endif // MAP_H
