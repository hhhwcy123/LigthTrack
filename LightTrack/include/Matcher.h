#ifndef MATCHER_H
#define MATCHER_H

#include <set>
#include <map>
#include <type_traits>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "Point.h"
#include "impl\Point.hpp"

namespace SLAM
{

class GlobalMap;
//template<typename Tp>
//class MapPoint_;
class KeyPoint;
class Frame;

#define MatcherTemplate template<template<typename Tp> typename T1, template<typename Tp> typename T2, typename Tp>

#define OrientFuncsLen 1
#define CalcSim3FuncsLen 2
#define MatchedKeysFuncsLen 2


template<template<typename Tp> typename T1 = cv::Point3_, template<typename Tp> typename T2 = T1, typename Tp = double>
class Matcher_
{
	static_assert(std::is_base_of<cv::Point3d, T1<Tp>>::value ||
				  std::is_base_of<cv::Point3f, T1<Tp>>::value ||
				  std::is_base_of<cv::Point3i, T1<Tp>>::value
				  , "Template argument T1<Tp> must be a cv::Point3_<_Tp> type in class Matcher_!");
	static_assert(std::is_base_of<cv::Point3d, T2<Tp>>::value ||
				  std::is_base_of<cv::Point3f, T2<Tp>>::value ||
				  std::is_base_of<cv::Point3i, T2<Tp>>::value
				  , "Template argument T2 must be a cv::Point3_<_Tp> type in class Matcher_!");

	friend class Tracker;

public:

	static const double SITE_DIFF_THRES;
	//static const int HISTO_LENGTH;
	static const double TRIANGLEBIAS_THRES;
	static const double P3D_MATCH_THRES;
	static const double MAX_PTBIAS_ERROR;


	struct Comparator
	{
		bool operator()(T1<Tp>* pt1, T2<Tp>* pt2) const { return cv::norm(*pt1) < cv::norm(*pt2); }
	};
	typedef std::vector<T1<Tp>*> T1List;
	typedef std::pair<T1<Tp>*, T1<Tp>*> T1Pair;
	typedef std::vector<T1Pair> T1PairList;
	typedef std::map<double, T1Pair> T1DistMap;
	typedef std::map<T1<Tp>*, std::pair<KeyPoint*, KeyPoint*>> T1KeyPairMap;

	//typedef std::map<KeyPoint*, KeyPoint*, Comparator> MatchedKeysList;
	typedef std::vector<T2<Tp>*> T2List;
	typedef std::pair<T2<Tp>*, T2<Tp>*> T2Pair;
	typedef std::vector<T2Pair> T2PairList;
	typedef std::map<double, T2Pair> T2DistMap;
	typedef std::map<T2<Tp>*, std::pair<KeyPoint*, KeyPoint*>> T2KeyPairMap;

	typedef std::pair<T1<Tp>*, T2<Tp>*> T1T2Pair;
	typedef std::vector<T1T2Pair> T1T2PairList;
	typedef std::multimap<double, T1T2Pair> T1T2DistMap;
	typedef std::set<T1T2Pair> T1T2PairSet;
	typedef std::map<T1<Tp>*, T2<Tp>*> T1T2MatchedMap;


	//typedef std::map<T1<Tp>*, T1<Tp>*, Comparator> T1MatchedMap;
	//typedef std::map<T1<Tp>*, T2*, Comparator > T1T2MatchedMap;

	typedef std::pair<T1T2Pair, T1T2Pair> TrianglePair;
	typedef std::vector<T1T2PairList> TriangleList;
	typedef std::vector<TrianglePair> TrianglePairList;
	typedef std::multimap<TrianglePair, T1T2PairList> TrianglePairMap;//类型multimap


	typedef void (*OrientFunc)(cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, double&, bool);
	typedef bool (*CalcSim3Func)(T1T2PairList&, T1T2PairList&, cv::Mat&, cv::Mat&, OrientFunc, const bool&);
	typedef int (*MatchedKeysFunc)(const T2List&, Frame&, T1T2PairList&, const double);

	enum OrientFuncType 
	{ 
		UNIT_QUAT //单位四元数
	};	
	enum CalcSim3FuncType 
	{ 
		SUCCESSIVE, //迭代删除
		RANSAC//随机抽样
	};
	enum MatchedKeysFuncsType 
	{
		PROJ,//3d-2d投影法
		UNPROJ//2d-3d反投影法
	};

	static const OrientFunc& orientFunc(const unsigned& i) { assert(i >= 0 && i < OrientFuncsLen); return _orientFuncs[i]; }
	static const CalcSim3Func& calcSim3Func(const unsigned& i) { assert(i >= 0 && i < CalcSim3FuncsLen); return _calcSim3Funcs[i]; }
	static const MatchedKeysFunc& matchedKeysFunc(const unsigned& i) { assert(i >= 0 && i < MatchedKeysFuncsLen); return _matchedKeysFuncs[i]; }

public:
	Matcher_() {}
	~Matcher_() {}

private:

	static void computeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
	{
		cv::reduce(P, C, 1, CV_REDUCE_SUM);
		C = C / P.cols;

		for (int i = 0; i<P.cols; i++)
			Pr.col(i) = P.col(i) - C;		
	}

	static int RandomInt(int min, int max)
	{
		int d = max - min + 1;
		return int(((double)rand() / ((double)RAND_MAX + 1.0)) * d) + min;
	}

	//bool correctMatchMptsList(MatchedKeysList& mapPointList);
	static double crossProductAngle3D(const T1<Tp>& A, const T1<Tp>& B, const T1<Tp>& C)
	{
		T1<Tp> AC = T1<Tp>(A.x - C.x, A.y - C.y, A.z - C.z);
		T1<Tp> BC = T1<Tp>(B.x - C.x, B.y - C.y, B.z - C.z);
		T1<Tp> CD = AC.cross(BC);
		return cv::norm(CD) / (cv::norm(AC)*cv::norm(BC));
	}


	static T1<Tp>* findVertex(const T1Pair& p3DPair1, const T1Pair& p3DPair2);

	//bool correctMatchMptsList(MatchedKeysList& mapPointList);
	static bool calcP3DDistList(const T1List& p3DList, T1DistMap& distList, double th);

	static bool calcP3DDistList(const Frame& frame, T1DistMap& distMap);

	static bool evaluateRt(const T1List& p3dListCur, const T2List& p3dListPrev,
						   std::set<T1<Tp>*>& curP3DTriSet, std::set<T2<Tp>*>& prevP3DTriSet,
						   cv::Mat& R, cv::Mat& t,
						   T1T2PairList& matchedP3DList, const double matchedRatio = .0);

	static bool evaluateRt(const T1List& p3dListCur, const T2List& p3dListPrev,
						   cv::Mat& R, cv::Mat& t,
						   T1T2PairList& matchedP3DList, const double matchedRatio= .0);

	static bool evaluateRt(TrianglePairMap& triPairMap, 
						   cv::Mat& R, cv::Mat& t, 
						   T1T2MatchedMap& matchedP3DList, const double matchedRatio = .0);


	static bool findMatchedP3DSite(T2PairList& condidateObjSiteList, T2DistMap& objSiteLenMap,
								   const T1Pair& srcSite, double th);

	static bool findMatchedP3DSite(T2Pair& objSite, T2DistMap& objSiteLenMap,
								   const T1Pair& srcSite, double th);

	
	static bool findMatchedTriVertexByFrame(T1<Tp>* p3DCur1, T1<Tp>* p3DCur2, T1<Tp>* p3DCur3,
											T2Pair& p3dSitePrev, T2DistMap& siteLenMapPrev,
											T1T2PairList& p3DPairList, double th);

	static bool findMatchedTriVertexByGlobalMap(T1<Tp>* p3DCur1, T1<Tp>* p3DCur2, T1<Tp>* p3DCur3,
												T2Pair& mptSite,  
												T1T2MatchedMap& p3DPairList, double th);
	

	static bool checkCollinear(cv::Mat& p3dMat);

	static bool checkCollinear(std::vector<T1<Tp>>& p3dList);

	static bool matchP3DByDist(Frame& prevFrame, Frame& curFrame, T1T2MatchedMap& matchedP3DList, 
							   cv::Mat& R, cv::Mat& t, 
							   OrientFunc orientFunc);

	static bool generateTriList(T1DistMap& distListCur, T2DistMap& distListPrev, TrianglePairList& trianglePairList);

	static bool findCondidateMatchedList(const TrianglePairList& trianglePairList, TrianglePairMap& triPairMap);

	static bool checkTriangulate(const T1PairList& p3DPairList);

	static bool evaluateMatch(TrianglePairMap& triPairMap, cv::Mat& RBest, cv::Mat& tBest, 
						      T1T2MatchedMap& bestMatchedP3DList, OrientFunc orientFunc);


public:
	static bool matchP3DByFrame(Frame& prevFrame, Frame& curFrame,
								T1T2PairList& matchedP3DList,
								cv::Mat& R, cv::Mat& t,
								const bool& bDispErrInfo, OrientFunc func,
								const double maxP3dDist, const double matchedRatio = 0.);

	static bool matchP3DByGlobalMap(GlobalMap* globalMap, Frame& curFrame,
									T1T2PairList& mptP3dPairList,
									cv::Mat& R, cv::Mat& t,
									OrientFunc func, const bool& bDispErrInfo, const double matchedRatio = 0.);

	static void absoluteOrientation(cv::Mat &P1, cv::Mat &P2, cv::Mat& R12, cv::Mat& t12, double& scale, bool bScale);

	static bool calcSim3Successive(T1T2PairList& matchedP3DList, T1T2PairList& outLiers, 
								   cv::Mat& R, cv::Mat& t, 
								   OrientFunc func, const bool& bDispErrorInfo);

	static bool calcSim3RANSAC(T1T2PairList& matchedP3DList, T1T2PairList& outLiers, 
							   cv::Mat& R, cv::Mat& t,
							   OrientFunc func, const bool& bDispErrorInfo);
	
	static int searchMptMatchedByFrame(Frame& refFrame, Frame& curFrame, T1PairList& p3DMatchedList, T1T2PairList& p3DMptMatchedList);
	
	//static int findMatchKeysByProj(Frame &prevFrame, Frame &currentFrame, T1T2PairList& p3DMptMatchedList, const double th2D);

	static int findMatchKeysByProj(const T2List& mapPointList, Frame &curFrame, 
								   T1T2PairList& p3DMptMatchedList, 
								   const double th2D);

	static int findMatchKeysByUnProj(const T2List& mapPointList, Frame &curFrame, 
									 T1T2PairList& p3DMptMatchedList, 
									 const double th3D);

	
	static std::mutex					_matchedMutex;


	static const OrientFunc				_orientFuncs[OrientFuncsLen];
	static const CalcSim3Func			_calcSim3Funcs[CalcSim3FuncsLen];
	static const MatchedKeysFunc		_matchedKeysFuncs[MatchedKeysFuncsLen];

};





}
#endif //MATCHER_H
