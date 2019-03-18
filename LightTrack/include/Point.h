#ifndef KEYPOINT_H
#define KEYPOINT_H
#include <list>
#include <map>
#include <set>
#include <mutex>
#include <type_traits>

#include "Config.h"
#include "Feature.h"
#include "TypeDef.h"

#include<opencv2/core/core.hpp>

#ifdef SLAM_SHARED_LIBS
#ifdef SLAM_ELEM_EXPORTS
#define POINT_API  __declspec(dllexport)
#else
#define POINT_API  __declspec(dllimport)
#endif
#else
#define POINT_API
#endif



namespace SLAM
{
class Frame;
class KeyFrame;
template<typename Tp>
class MapPoint_;
class GlobalMap;


class POINT_API KeyPoint : public cv::Point_<double>
{
	friend class Tracker;
	friend class Frame;
	friend class Optimizer;
	//template<template<typename Tp> typename T1, template<typename Tp> typename T2, typename Tp>
	//friend class Matcher_;
	friend class System;
public:
	KeyPoint(const int id = 0) :
		_id(id),
		_undistortX(0),
		_undistortY(0),
		_frame(nullptr),
		_matchedP3D(nullptr),
		_matchedMpt(nullptr),
		_eccentricity(0),
		_areaSize(0),
		_weight(1.0),
		cv::Point2d(0, 0)
	{}
	KeyPoint(const double& x, const double& y, const int id = 0) :
		_id(id),
		_undistortX(0),
		_undistortY(0),
		_frame(nullptr),
		_matchedP3D(nullptr),
		_matchedMpt(nullptr),
		_eccentricity(0),
		_areaSize(0),
		_weight(1.0),
		cv::Point2d(x, y)
	{}
	KeyPoint(const Feature::DotMarker& dotMarker, const int id = 0) :
		_id(id),
		_undistortX(dotMarker.undistortX),
		_undistortY(dotMarker.undistortY),
		_frame(nullptr),
		_matchedP3D(nullptr),
		_matchedMpt(nullptr),
		_eccentricity(dotMarker.eccentricity),
		_areaSize(dotMarker.areaSize),
		_weight(1.0),
		_rect(dotMarker.rect),
		cv::Point2d(dotMarker.x, dotMarker.y)
	{}

	KeyPoint& operator =(const KeyPoint& kpt)	
	{
		cv::Point2d(kpt.x, kpt.y);
		_id = (kpt._id);
		_undistortX = kpt._undistortX;
		_undistortY = kpt._undistortY;
		_eccentricity = kpt._eccentricity;
		_areaSize = kpt._areaSize;
		_bestNorm = kpt._bestNorm.clone();
		_rect = kpt._rect;
		_weight = kpt._weight;		
		_matchedP3D = kpt._matchedP3D;
		_matchedMpt = kpt._matchedMpt;
		_frame = kpt._frame;
		return *this;
	}

	~KeyPoint() {}


public:
	bool operator <(KeyPoint* point)
	{
		if (_id != point->_id)
			return _id < point->_id;
		else
			return false;
	}
	bool operator ==(KeyPoint* point)
	{
		if (_id == point->_id)
			return true;
		else
			return false;
	}
	cv::Point2d operator -(KeyPoint& point)
	{
		return cv::Point2d(_undistortX - point._undistortX, _undistortY - point._undistortY);
	}

	const unsigned int& getID() const { return _id; }
	void setID(const unsigned& val) { _id = val; }

	const double& getUndistortX() { return _undistortX; }
	void setUndistorX(const double& val) { _undistortX = val; }

	const double& getUndistortY() { return _undistortY; }
	void setUndistorY(const double& val) { _undistortY = val; }

	const double& getAreaSize() { return _areaSize; }
	void setAreaSize(const double& val) { _areaSize = val; }

	const double& getEccentricity() { return _eccentricity; }
	void setEccentricity(const double& val) { _eccentricity = val; }

	void setFrame(Frame* const frame) { _frame = frame; }
	Frame* getFrame() { return _frame; }

	void setMatchedP3D(cv::Point3d* Point3D) { _matchedP3D = Point3D; }
	cv::Point3d* getMatchedP3D() { return _matchedP3D; }

	void setMatchedMpt(MapPoint3d* mapPoint) { _matchedMpt = mapPoint; }
	MapPoint3d* getMatchedMpt() { return _matchedMpt; }

	void setRect(const cv::RotatedRect& rect) { _rect = rect; }
	const cv::RotatedRect& getRect() { return _rect; }

	void setWeight(double val) { _weight = val; }
	const double& getWeight() { return _weight; }

	void setBestNorm(const cv::Mat& norm) { _bestNorm = norm; }
	const cv::Mat& getBestNorm() { return _bestNorm; }

	void undistort(const cv::Mat K,const cv::Mat D);

	double distWithKeyPoint(KeyPoint* keyPoint) { return cv::norm(*this - *keyPoint); }
	
	void calcWeight(const Feature::DotMarker& dotMarker);


private:
	unsigned int				    _id;
	//double						_x;
	//double						_y;
	double							_undistortX;
	double							_undistortY;
	double							_eccentricity;//统计用
	double							_areaSize;//统计用
	std::list<cv::Mat>				_normsList;//法向量
	cv::Mat							_bestNorm;
	cv::RotatedRect                 _rect;
	double							_weight;
	cv::Point3d*					_matchedP3D;//相机坐标系3D点（非世界坐标系）
	MapPoint3d*				        _matchedMpt;//匹配地图点
	Frame*							_frame;

};








template<typename T = double>
class MapPoint_:public cv::Point3_<T>
{
	static_assert(std::is_same<int, T>::value ||
				  std::is_same<float, T>::value ||
				  std::is_same<double, T>::value
				  , "Template argument T must be a base type in class MapPoint_!");
	friend class Tracker;
	friend class Frame;
	friend class Optimizer;
	//template<template<typename Tp> typename T1, template<typename Tp> typename T2, typename Tp>
	//friend class Matcher_;
	friend class GlobalMap;
	friend class ReaderWriter;
	friend class LoopClosing;
public:

	typedef std::pair<MapPoint_<T>*, MapPoint_<T>*> MptPair;
	typedef std::map<double, MptPair> MptDistMap;



public:
	MapPoint_(cv::Point3_<T>& point3D);

	MapPoint_(double x = 0.0, double y = 0.0, double z = 0.0);

	~MapPoint_() {}

	bool operator <(MapPoint_<T>* point)
	{
		if (_id != point->_id)
			return _id < point->_id;
		else
			return false;
	}
	bool operator ==(MapPoint_<T>* point)
	{
		if (_id == point->_id)
			return true;
		else
			return false;
	}
	bool operator ==(const int& id)
	{
		if (_id == id)
			return true;
		else
			return false;
	}


	const unsigned int getID() { return _id; }
	void setID(const int& id) { _id = id; }

	void setGlobalMap(GlobalMap* pMap) { _globalMap = pMap; }
	GlobalMap* getGlobalMap() {return _globalMap;}

	cv::Mat getWorldPos() { return cv::Mat(*dynamic_cast<cv::Point3_<T>*>(this)); }

	void setWorldPos(cv::Mat posMat) 
	{
		x = posMat.at<T>(0, 0); 
		y = posMat.at<T>(1, 0);
		z = posMat.at<T>(2, 0);
	}

	//double& getX() { return _x; }

	//double& getY() { return _y; }

	//double& getZ() { return _z; }


	void setMatchedKeyPointL(KeyPoint* keyPoint){ keyPoint->setMatchedMpt(this); }

	void setMatchedKeyPointR(KeyPoint* keyPoint) { keyPoint->setMatchedMpt(this); }

	cv::Point3_<T>& getOriCoord() { return _coordOri; }

	cv::Point3_<T>& getLastCoord() { return _coordLast; }

	void setBad(bool flag) { _fBad = flag; }
	bool isBad() { return _fBad; }

	void setRefKF(KeyFrame* pKF) { _refKF = pKF; }
	KeyFrame* getReferenceKeyFrame() { return _refKF; }

	void setNormOri(const cv::Mat& norm) { _normOri = norm.clone(); }
	cv::Mat& getNormOri() { return _normOri; }

	void setNormLast(const cv::Mat& norm) { _normLast = norm.clone(); }
	cv::Mat& getNormLast() { return _normLast; }

	MptDistMap& getConnSiteMap() { return _connSiteMap; }

	const int obs() { return _observations.size(); }

	const int obsKF() { return _observationKFs.size(); }

	int& outliersCount() { return _outLierCount; }

	FrameSet& getObservations() { return _observations; }

	KFSet& getObservationKFs() { return _observationKFs; }

	void addKFObservation(KeyFrame* kF);
	
	void addObservation(Frame* frame);

	void eraseObservation(Frame* frame);

	void eraseKFObservation(KeyFrame* pKF);

	void addConnSite(MptPair& site,double len = 0);
	
	bool outputCoord(std::ofstream& ofs);

	

private:
	
	unsigned int					_id;
	//double						_x;
	//double						_y;
	//double						_z;

	FrameSet						_observations;
	KFSet							_observationKFs;
	int								_outLierCount;//非线性优化中的离散次数	
	cv::Mat                         _normOri;
	cv::Mat							_normLast;
	KeyFrame*						_refKF;
	cv::Point3_<T>                  _coordOri;
	cv::Point3_<T>					_coordLast;
	cv::Mat                         _posGBA;

	//bool							_fOutLier;//在g2o中确定，离群点标志位
	bool                            _fBad;//丢弃标志，观察帧数过小会置位
	int                             _localForKF;

	MptDistMap                      _connSiteMap;

	GlobalMap*                      _globalMap;


	static std::mutex               _obsMutex;
	
};





} //namespace SLAM

#endif // KEYPOINT_H
