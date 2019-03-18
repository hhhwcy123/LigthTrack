#ifndef FRAME_H
#define FRAME_H

#include "Tracker.h"
#include "Point.h"
#include "impl\Point.hpp"

//#include "Camera.h"

#include <vector>
#include <direct.h>
#include <mutex>
#include <set>

#include <opencv2/opencv.hpp>

#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_ELEM_EXPORTS
#define FRAME_API  __declspec(dllexport)
#else
#define FRAME_API  __declspec(dllimport)
#endif
#else
#define FRAME_API
#endif



namespace SLAM
{

#define calcMatchedFuncsLen 5

class GlobalMap;
class System;
class Camera;

class FRAME_API Frame
{
	static const double MaxEpilineDist;
	static const double MinEnergyValue;
	static const double MarkerAngleThres;
	static const double MarkerCenterThres;

	friend class KeyPoint;
	friend class Optimizer;
	friend class System;
	MatcherTemplate
	friend class Matcher_;

public:
	struct FrameComparator
	{
		bool operator()(Frame* pF1, Frame* pF2) const { return pF1->_id < pF2->_id; }
	};
	typedef std::set<Frame*, FrameComparator> FrameCompSet;

	enum CalcMatchedType
	{
		NORM,//单目圆法向量位姿约束
		EPIPOLAR,//对极几何约束
		ENERGY//双目能量匹配
	};

	typedef bool (*CalcMatchedFunc)(Frame*, Feature::MarkerList&, Feature::MarkerList&, Feature::MarkerMultiMap&);
	
	const Frame::CalcMatchedFunc& calcMatchedFunc(const unsigned& i) { assert(i >= 0 && i < calcMatchedFuncsLen); return _calcMatchedFuncs[i]; }

	template<class T>
	struct DerefFrameFunctor
	{
		bool operator()(const T& lhs, const T& rhs)
		{
			//if (!lhs || !rhs)
			//	return lhs < rhs;
			if (lhs.second == rhs.second)
				return lhs.first->getID() < rhs.first->getID();
			else
				return lhs.second < rhs.second;
		}
	};
	bool operator < (const Frame& F)
	{
		if (_id != F._id)
			return _id < F._id;
		else
			return false;
	}

	bool operator < (const Frame& F) const
	{
		if (_id != F._id)
			return _id < F._id;
		else
			return false;
	}

	//计算sin<AC,BC>
	static double crossProductAngle2D(const Feature::DotMarker& A, const Feature::DotMarker& B, const Feature::DotMarker& C)
	{
		cv::Point2d AC = cv::Point2d(A.x - C.x, A.y - C.y);
		cv::Point2d BC = cv::Point2d(B.x - C.x, B.y - C.y);
		return (AC.x*BC.y - AC.y*BC.x)/(cv::norm(AC)*cv::norm(BC));
	}

public:
	Frame() {}

	Frame(const int& id, int width = INT_MAX, int height = INT_MAX, double minDistance = 0, double maxDistance = FLT_MAX) :
		_id(id), 
		_bUndistorted(false),
		_minDistance(minDistance),
		_maxDistance(maxDistance),
		_minX(0),
		_maxX(width),
		_minY(0),
		_maxY(height),
		_globalMap(nullptr),
		_localForKF(-1),
		_fixedForKF(-1),
		_system(nullptr),
		_maxPrevConn(nullptr),
		_isFirstConnection(true),
		_refKF(nullptr),
		_isRelocByMap(false),
		_isRelocByRefKF(false),
		_fBad(false),
		_notToErase(false)
	{}


	virtual ~Frame();

public:
	bool operator == (Frame& f) const { return f._id == _id; }

	bool operator < (Frame& f) const { return f._id < _id; }

	const int getID() const{ return _id; }
	void setID(const int& id) { _id = id; }

	System* getSystem() { return _system; }
	void setSystem(System* system) { _system = system; }

	Camera* getCamera() { return _camera; }
	void setCamera(Camera* camera) { _camera = camera; }

	GlobalMap* getGlobalMap() { return _globalMap; }
	void setGlobalMap(GlobalMap* pMap) { _globalMap = pMap; }

	int& minX() { return _minX; }
	void setMinX(const int& x) { _minX = x; }	

	int& maxX() { return _maxX; }
	void setMaxX(const int& x) { _maxX = x; }

	int& minY() { return _minY; }
	void setMinY(const int& y) { _minY = y; }

	int& maxY() { return _maxY; }
	void setMaxY(const int& y) { _maxY = y; }

	//cv::Mat& Kl() { return _camera->Kl(); };
	//cv::Mat& Kr() { return _camera->Kr(); };
	//cv::Mat& Dl() { return _camera->Dl(); };
	//cv::Mat& Dr() { return _camera->Dr(); };
	//double& baseLen() { return _camera->baseLen(); }
	//cv::Mat& Rl() { return _camera->Rl(); };
	//cv::Mat& Rr() { return _camera->Rr(); };
	//cv::Mat& Pl() { return _camera->Pl(); };
	//cv::Mat& Pr() { return _camera->Pr(); };
	//cv::Mat& R() { return _camera->R(); };
	//cv::Mat& t() { return _camera->t(); };
	//cv::Mat& E() { return _camera->E(); };
	//cv::Mat& F() { return _camera->F(); };
	//cv::Mat& Q() { return _camera->Q(); };

	void setUndistorted(const bool& flag) { _bUndistorted = flag; }
	bool isUndistorted() { return _bUndistorted; }

	bool isBad() { return _fBad; }
	void setBad(const bool& flag) { _fBad = flag; }

	void setNotToErase(const bool& flag) { _notToErase = flag; }
	const bool& isNotToErase() { return _notToErase; }

	const KeyFrame* getRefKF() { return _refKF; }
	void setRefKF(KeyFrame* const refKF) { _refKF = refKF; }


	void addKeyPointsLeft( KeyPoint* const point) 
	{
		_keyPointsLeft.emplace_back(point); 
		if (!point->getFrame())
			point->setFrame(this);	
	}

	void addKeyPointsRight(KeyPoint* const point) 
	{
		_keyPointsRight.emplace_back(point);
		if (!point->getFrame())
			point->setFrame(this);
	}
	
	P3DKeyPairMap& getP3DMap() { return _p3DMap; }

	void insert3DPoints(cv::Point3d* const point3D,KeyPoint* point2DLeft,KeyPoint* point2DRight)
	{
		std::unique_lock<std::mutex> lock(_matchedMutex);
		_p3DMap[point3D]=std::make_pair( point2DLeft, point2DRight);
		if (!point2DLeft->_matchedP3D)point2DLeft->setMatchedP3D(point3D);
		if (!point2DRight->_matchedP3D)point2DRight->setMatchedP3D(point3D);
	}

	bool getKeyPairFromP3D(cv::Point3d* p3d, KeyPointPair& keyPointPair)
	{
		P3DKeyPairMap::iterator itr = _p3DMap.find(p3d);
		if (itr != _p3DMap.end())
		{
			keyPointPair= itr->second;
			return true;
		}
		return false;
	}

	bool getKeyPairFromMpt(MapPoint3d* mpt, KeyPointPair& keyPointPair)
	{
		MptKeyPairMap::iterator itr = _matchedMptMap.find(mpt);
		if (itr != _matchedMptMap.end())
		{
			keyPointPair = itr->second;
			assert(keyPointPair.first);
			assert(keyPointPair.second);
			return true;
		}
		return false;
	}
	bool getP3dFromKey(KeyPoint* keyLeft, cv::Point3d*& p3D)
	{
		if (keyLeft->_matchedP3D)
		{
			p3D = keyLeft->_matchedP3D;
			return true;
		}
		return false;
	}

	bool getMptFromKey(KeyPoint* keyLeft, MapPoint3d*& mapPoint)
	{
		if (keyLeft->_matchedMpt)
		{
			mapPoint = keyLeft->_matchedMpt;
			return true;
		}	
		return false;
	}

	cv::Point3d* getP3DFromMpt(MapPoint3d* mapPoint)
	{
		KeyPointPair keyPair;
		if (!getKeyPairFromMpt(mapPoint, keyPair))
			return nullptr;
		else
			return keyPair.first->_matchedP3D;	
	}



	MapPoint3d* getMptFromeP3D(cv::Point3d* p3d)
	{
		KeyPointPair kptPair;
		if (!getKeyPairFromP3D(p3d, kptPair))
			return nullptr;
		
		return kptPair.first->_matchedMpt;
	}

	MptKeyPairMap& getMatchedMptMap()
	{
		std::unique_lock<std::mutex> lock(_matchedMutex);
		return _matchedMptMap;
	}

	MptSet& getOutLiers() { return _outLiers; }

	void insertMatchedMPt(MapPoint3d* const mapPoint, KeyPoint* const keyPointL, KeyPoint* const keyPointR);

	void insertOutLier(MapPoint3d* mpt);
	
	void eraseOutlier(MapPoint3d* pMP);
	
	void clearOutLiers();

	cv::Mat Rcw() { return _Rcw.clone(); }
	cv::Mat Rcw() const { return _Rcw.clone(); }

	cv::Mat tcw() { return _tcw.clone(); }
	cv::Mat tcw() const { return _tcw.clone(); }

	cv::Mat Rwc() { return _Rwc.clone(); }
	cv::Mat Rwc() const { return _Rwc.clone(); }

	cv::Mat twc() { return _Ow.clone(); }
	cv::Mat twc() const { return _Ow.clone(); }

	void setPose(cv::Mat Tcw);
	void setPose(cv::Mat Rcw, cv::Mat tcw);

	cv::Mat getPose() { return _Tcw.clone(); }
	cv::Mat getPose() const { return _Tcw.clone(); }

	cv::Mat getPoseInverse() { return _Twc.clone(); }
	cv::Mat getPoseInverse() const { return _Twc.clone(); }

	void setRotation(const cv::Mat& R) { _Rwc = R; }

	cv::Mat getRotation(){return _Tcw.rowRange(0, 3).colRange(0, 3).clone();}
	cv::Mat getRotation() const { return _Tcw.rowRange(0, 3).colRange(0, 3).clone(); }

	void setTranslation(const cv::Mat& t) { _tcw = t; }

	cv::Mat getTranslation(){return _Tcw.rowRange(0, 3).col(3).clone();}
	cv::Mat getTranslation() const { return _Tcw.rowRange(0, 3).col(3).clone(); }

	void updatePoseMatrices();

	cv::Mat getCameraCenter() { return _Ow.clone(); }

	cv::Mat getRotationInverse() { return _Rwc.clone(); }

	bool isInFrustum(KeyPoint *pMP, double viewingCosLimit);

	cv::Point3d Frame::localToWorld(cv::Point3d* point3D);

	bool searchMptMatchedByFrame(const Frame& prevFrame, const P3DPairList& matchedP3DList, P3DMptPairList& p3DMptMatchedList);

	void searchMptMatchedByGlobalMap(P3DMptPairList& p3dMptPairList);

	void updateNormMatched();

	void undistortKeyPoint();

	void calcMotionModel(const Frame& prevFrame);

	cv::Mat calcRelativeVelocity(Frame& prevFrame);

	KeyPoint* getKeyPointRight(KeyPoint* const KeyPointLeft)
	{
		cv::Point3d* matched3DPoint = KeyPointLeft->_matchedP3D;
		return _p3DMap.find(matched3DPoint)->second.second;
	}

	KeyPoint* getKeyPointLeft(KeyPoint* const KeyPointRight)
	{
		cv::Point3d* matched3DPoint = KeyPointRight->_matchedP3D;
		return _p3DMap.find(matched3DPoint)->second.second;
	}

	bool eraseMptMatchedWithP3D(MapPoint3d* pMP);

	void eraseMptMatchedWithP3D();

	bool eraseMptMatched(MapPoint3d* pMP);

	double calcMptMatchedError(MapPoint3d* mpt);

	double calcMptMatchedError(cv::Point3d* p3d, MapPoint3d* mpt);

	void clearMatchedMpts();

	KeyFrame* getKFInGlobalMap();

	bool outputPos(std::ofstream& ofs);

	static double calcEnergyValue(const Feature::DotMarker& marker1, const Feature::DotMarker& marker2,
								  Feature::MarkerList& markList1, Feature::MarkerList& markList2,
								  cv::Mat F, double radius, int cameraID);

	static Feature::MarkerList seekNeighborMarks(Feature::DotMarker marker, Feature::MarkerList markList1, double radius);

	static bool calcStereoMatchesByNorms(Frame* pFrame, Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight,  Feature::MarkerMultiMap& markerMathcedMap);
	
	static bool calcStereoMatchesByEpipolar(Frame* pFrame, Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight, Feature::MarkerMultiMap& markerMathcedMap);

	static bool calcStereoMatchesByEnergy(Frame* pFrame, Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight, Feature::MarkerMultiMap& markerMathcedMap);

	static bool getBestMarkerPose(Camera* camera, Feature::DotMarker& dotMarker1, Feature::DotMarker& dotMarker2);
	
	void unprojectStereo(const Feature::MarkerPair& MarkerPair, cv::Point3d* p3d);

	bool compute3Dpoints(const cv::Mat& imgLeft, const cv::Mat& imgRight, CalcMatchedFunc func1, Feature::ExtractMarkFunc func2);

	bool compute3Dpoints(Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight, CalcMatchedFunc func);
	
	void createFrameMarkImg(cv::Mat& img, const Feature::MarkerList& markerList);

	cv::Mat& createMatchedImg(bool isTackedOK);

	bool createMatchedImg();

	void releaseImg();

	void updateConnections();

	void addConnection(Frame* F, const int &weight) { _orderedConnecteds.insert(std::make_pair(F,weight)); }

	int getCovisibleCount(Frame* pF);

	FrameList getBestCovisibles(const int &N);

	void eraseConnection(Frame* pF);

	FrameList getCovisibles();

	void addMaxNextConn(Frame* pF) { _maxNextConns.insert(pF); }

	double calcMatchedRMSE();

	bool checkMatchedExtend();



public:

	unsigned									_id;

	bool										_bUndistorted;
	//外参以左相机为准
	cv::Mat										_Rcw;
	cv::Mat										_tcw;
	cv::Mat										_Rwc;
	cv::Mat										_Ow;
	cv::Mat										_Tcw;
	cv::Mat			  							_Twc;

	cv::Mat										_markImgLeft;
	cv::Mat										_markImgRight;
	cv::Mat										_stereoImg;

	P3DKeyPairMap								_p3DMap;
	KeyPointList								_keyPointsLeft;
	KeyPointList								_keyPointsRight;
	MptKeyPairMap								_matchedMptMap;
	MptSet										_outLiers;
	 
	KeyFrame*									_refKF;//参考关键帧

	double										_minDistance;
	double										_maxDistance;
	int											_minX;
	int											_maxX;
	int											_minY;
	int											_maxY;

	cv::Mat										_refMotion;//跟踪时相对上一帧的运动矩阵

	///////////////////局部BA优化的时候用,判断该帧是否是当前帧的局部共点帧////////////////////////
	int											_localForKF;//局部共点帧（共点数量大于一定阈值）
	int											_fixedForKF;//共点但不属于局部共点帧（共点数量小于阈值）
	FrameMap									_orderedConnecteds;
	Frame*										_maxPrevConn;
	FrameSet									_maxNextConns;
	bool										_isFirstConnection;
	bool										_isRelocByRefKF;
	bool										_isRelocByMap;
	bool										_fBad;
	bool										_notToErase;

	Camera*										_camera;
	GlobalMap*									_globalMap;
	System*									    _system;

	static const Frame::CalcMatchedFunc			_calcMatchedFuncs[calcMatchedFuncsLen];

	static std::mutex							_matchedMutex;
};

}// namespace SLAM

#endif // FRAME_H
