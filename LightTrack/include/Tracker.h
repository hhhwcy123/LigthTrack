#ifndef TRACKER_H
#define TRACKER_H

#include <io.h>
#include <direct.h>
#include <map>
#include <set>
#include <mutex>

#include "TypeDef.h"
#include "Matcher.h"
#include "ThreadPool.h"

#include "Config.h"


#include <opencv2/opencv.hpp>

#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_TRACK_EXPORTS
#define TRACKER_API  __declspec(dllexport)
#else
#define TRACKER_API  __declspec(dllimport)
#endif
#else
#define TRACKER_API
#endif


namespace SLAM
{


class GlobalMap;
class Frame;
class KeyFrame;
class KeyPoint;
template<typename Tp>
class MapPoint_;
class Drawer;
class Viewer;
class System;
class LocalMapping;

#define PI (3.1415926535897932346f)  

struct Comparator
{
	bool operator()(cv::Point3d* pt1, cv::Point3d* pt2) const { return cv::norm(*pt1) < cv::norm(*pt2); }
};



class TRACKER_API Tracker
{

	friend class System;
	friend class ReadWriter;
	friend class Viewer;
	friend class LocalMapping;
public:
	static const int FRAME_THRES;
	static const int OBSERVE_THRES;
	static const int KEYFRAME_INTERVAL;
	static const int LOCAL_MAX_MPTS;
	static const double P3D_NEAR_MAX;
	static const double P3D_NEAR_MIN;
	static const double KEY_MATCH_THRES;
	static const double MPT_MATCH_THRES;
	static const double ROTATION_THRES;
	static const double TRANSPOSE_THRES;
	static const double MATCHED_RATIO;
	static const double P3D_DIST_MAX;

	enum RelocState
	{
		Reloc_Failed,
		Reloc_Ref,
		Reloc_Map
	};

public:

	Tracker() :
		_globalMap(nullptr),
		_mapDrawer(nullptr),
		_system(nullptr),
		_viewer(nullptr),
		_refFrame(nullptr),
		_currentFrame(nullptr),
		_refKF(nullptr)
	{}
	~Tracker(){}

	static std::mutex& trackMutex() { return _trackMutex; }

	GlobalMap* getMap() { return _globalMap; }
	void setMap(GlobalMap* const map) { _globalMap = map; }

	Drawer* getMapDrawer() { return _mapDrawer; }
	void setMapDrawer(Drawer* const drawer) { _mapDrawer = drawer; }

	Viewer* getViewer() { return _viewer; }
	void setViewer(Viewer* const viewer) { _viewer = viewer; }

	System* getSystem() { return _system; }
	void setSystem(System* system) { _system = system; }

	LocalMapping* getLocalMapper() { return _localMapper; }
	void setLocalMapper(LocalMapping* localMapper) { _localMapper = localMapper; }

	ThreadPool& getRelocThreadPool() { return _relocThreadPool; }

	void setRefFrame(Frame* pFrame) { _refFrame = pFrame; }

	void setRefKF(KeyFrame* pKF) { _refKF = pKF; }

	P3DPairList& getMatchedP3DList() { return _matchedP3DList; }

	bool preTrack(Frame& frame);

	bool tracking(Frame& frame);

	bool initTracker(Frame& frame);

	bool findMatchMptsByFrame(Frame& refFrame, Frame& curFrame, P3DMptPairList& p3DMptMatchedList, Matcher_<cv::Point3_, MapPoint_>::CalcSim3Func calcSim3Func, Matcher_<>::OrientFunc orientationFunc, const double p3dMathcedRatio);

	bool findMatchMptsByGlobalMap(GlobalMap* globalMap, Frame& curFrame, P3DMptPairList& p3DMptMatchedList, Matcher_<cv::Point3_, MapPoint_>::CalcSim3Func calcSim3Func, Matcher_<>::OrientFunc orientationFunc, const double p3dMathcedRatio = 0.);

	bool trackRefFrame(Frame& prevFrame, Frame& curFrame, Matcher_<cv::Point3_, MapPoint_>::CalcSim3Func func, Matcher_<>::OrientFunc orientationFunc, const double p3dMathcedRatio=0.);

	RelocState relocalizing(Frame& curFrame);


	static bool relocByRefKF(KeyFrame& refKF, Frame& curFrame, P3DPairList& matchedP3DList,
							 Matcher_<cv::Point3_, MapPoint_>::CalcSim3Func func, 
							 Matcher_<>::OrientFunc orientationFunc, 
							 System* system,GlobalMap* globalMap, const double p3dMathcedRatio = 0.);
							 
	bool trackMap(Frame& currentFrame, Matcher_<cv::Point3_, MapPoint_, double>::CalcSim3Func func, Matcher_<>::OrientFunc orientationFunc, const double p3dMathcedRatio = 0.);

	bool TrackWithMotionModel(Frame& prevFrame, Frame& currentFrame);

	void updateLocalMpts(MptList& localMptList);

	int checkUnMatchedKeys(Frame& frame, P3DMptPairList& p3DMptMatchedList, Matcher_<cv::Point3_, MapPoint_>::MatchedKeysFunc fun);

	int expandGlobalMap(KeyFrame* const pKF);

	bool isP3DValid(cv::Point3d* p3d, P3DKeyPairMap& p3dMap,double thMin,double thMax);

	bool needNewKeyFrame(const Frame& frame);

	KeyFrame* createKeyFrame(Frame& frame);

	bool createTrackImg(Frame& frameCur, Frame& framePrev);

	bool writeTrackMatchedImg(Frame& frameCur,  Frame& framePrev, const P3DPairList& matchedP3DList);

	bool writeTrackNewMptImg(Frame& frameCur,  Frame& framePrev);

	int checkMapUntilFrame(const Frame& currentFrame,const unsigned& frameThres, const unsigned& obsThres);

	void mapPointCulling(const Frame& Frame);
	
	bool checkRt(cv::Mat& Rcw, cv::Mat& tcw, Frame* currentFrame, Frame* prevFrame, GlobalMap* globalMap);
private:
	
	Frame*                  _refFrame;
	Frame*				    _currentFrame;
	KeyFrame*               _refKF;
	static KeyFrame*		_relocRefKF;
	P3DPairList				_matchedP3DList;
	cv::Mat					_canvasImg;
	ThreadPool				_relocThreadPool;

	GlobalMap*				_globalMap;
	Drawer*					_mapDrawer;
	Viewer*					_viewer;
	System*					_system;
	LocalMapping*			_localMapper;

	static std::mutex       _trackMutex;
};



}


#endif // TRACKER_H