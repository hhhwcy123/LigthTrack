#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "Frame.h"

namespace SLAM
{

class GlobalMap;

class FRAME_API KeyFrame :public Frame
{
	friend class System;
	friend class Optimizer;
	friend class GlobalMap;
	friend class LoopClosing;
	friend class Tracker;
	friend class LocalMapping;
	friend class Drawer;
	template<typename Tp>
	friend class MapPoint_;

public:
	struct KFComparator
	{
		bool operator()(KeyFrame* pKF1, KeyFrame* pKF2) const { return pKF1->_id < pKF2->_id; }
	};

	template<class T>
	struct DerefKFWightFunctor
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

	typedef std::map<KeyFrame*, int, KFComparator> KFWeight;
	typedef std::set<KeyFrame*, KFComparator> KFCompSet;
	
	
public:
	KeyFrame();
	KeyFrame(const Frame& frame);

	~KeyFrame() {}


	friend bool operator == (const Frame& F, const KeyFrame& kF);

	bool operator < (const KeyFrame& kF)
	{
		if (_id != kF._id)
			return _id < kF._id;
		else
			return false;
	}

	bool operator < (const KeyFrame& kF) const
	{
		if (_id != kF._id)
			return _id < kF._id;
		else
			return false;
	}

	const unsigned getID() const { return _id; }
	void setID(const unsigned int& id) { _id = id; }

	void addLoopKF(KeyFrame* pKF) { _loopKFs.insert(pKF); }
	KFSet& getLoopKFs() { return _loopKFs; }

	Frame::FrameCompSet& getLocalFrames() { return _localFrameSet; }
	void addLocalFrame(Frame* frame) { _localFrameSet.insert(frame); }
	void eraseLocalFrame(Frame* frame) {_localFrameSet.erase(frame);}

	MptList& getNewMptList() { return _newMptList; }
	void addNewMapPoint(MapPoint3d* mapPoint){_newMptList.emplace_back(mapPoint);}

	Frame* getRefInMap();

	void setMaxPrevKF(KeyFrame* pKF) { _maxPrevConnKF = pKF; }
	KeyFrame* getMaxPrevKF() { return _maxPrevConnKF; }

	void addMaxNextKF(KeyFrame* pKF) { _maxNextConnKFs.insert(pKF); }
	KFSet& getMaxNextConnKFs() { return _maxNextConnKFs; }

	void eraseMaxNextKF(KeyFrame* pKF) { _maxNextConnKFs.erase(pKF); }

	bool hasNextMaxKF(KeyFrame* pKF) {return _maxNextConnKFs.count(pKF); }

	void changeMaxPrevKF(KeyFrame* pKF)
	{
		_maxPrevConnKF = pKF;
		if (pKF)
			pKF->addMaxNextKF(this);
	}

	
	void insertMatchedMPt(MapPoint3d* const mapPoint, KeyPoint* const keyPointL, KeyPoint* const keyPointR)
	{
		//std::unique_lock<std::mutex> lock(_matchedMptMutex);
		Frame::insertMatchedMPt(mapPoint, keyPointL, keyPointR);		
	}

	void updateBALocalForKF();

	void updateAndCorrectLocalPoses(Matcher_<cv::Point3_, MapPoint_>::CalcSim3Func calcSim3Func, 
									Matcher_<>::OrientFunc orientationFunc);

	void updateLocalPoses();

	void updateChildrenPoses(cv::Mat correctPose);

	void updateKFConnections();

	void addKFConnection(KeyFrame* kF, const int &weight){_orderedConnectedKFs[kF]=weight;}

	void eraseKFConnection(KeyFrame* pKF);

	KeyFrameList getBestCovisibleKeyFrames(const int &N);

	KeyFrameList getCovisibleKFs();

	int getKFCovisibleCount(KeyFrame* pKF);

	void cutAllChildrenKFs();
	
	void getAllChildrenKFs(KFSet& childrenKFs);

	KFSet& getChildrenKFs() { return _childrenKFs; }
	void insertChildrenKF(KeyFrame* pKF) { _childrenKFs.insert(pKF); }

	void setParentKF(KeyFrame* pKF) { _parentKF = pKF; }
	KeyFrame* getParentKF() { return _parentKF; }

	void eraseChild(KeyFrame* pKF);

	bool hasChild(KeyFrame* pKF);
private:

	unsigned							_id;

	KFWeight					        _orderedConnectedKFs;
	KeyFrame*							_maxPrevConnKF;
	KFSet								_maxNextConnKFs;

	KeyFrame*							_parentKF;//�ռ���ǰ���ڽӹؼ�֡
	KFSet						        _childrenKFs;//�ռ��Ϻ����ڽӹؼ�֡���ϣ�֡-ȫ��ƥ���������з�֧��

	KFSet                               _loopKFs;//�ػ�ƥ��֡

	bool								_isFirstKFConnection;

	MptList						        _newMptList;
	FrameCompSet						_localFrameSet;

	///////////////////�ֲ�BA�Ż���ʱ��ʹ��,�жϸ�֡�Ƿ��ǵ�ǰ֡�ľֲ�����֡////////////////////////
	int									_localForKF;//�ֲ�����֡��������������һ����ֵ��
	int									_fixedForKF;//���㵫�����ھֲ�����֡����������С����ֵ��

	cv::Mat							    _TcwGBA;

	static std::mutex					_connMutex;
};






}


#endif // !KEYFRAME_H