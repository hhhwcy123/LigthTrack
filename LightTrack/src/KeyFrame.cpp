#include "KeyFrame.h"
#include "GlobalMap.h"
#include "Matcher.h"
#include "impl\Matcher.hpp"


namespace SLAM
{

std::mutex KeyFrame::_connMutex;

KeyFrame::KeyFrame() :
	_id(GlobalMap::keyFrameCount()),
	_maxPrevConnKF(nullptr),
	_localForKF(-1),
	_fixedForKF(-1),
	_isFirstKFConnection(true),
	_parentKF(nullptr)
{}

KeyFrame::KeyFrame(const Frame& frame) :
	Frame(frame),
	_id(GlobalMap::keyFrameCount()),
	_maxPrevConnKF(nullptr),
	_localForKF(-1),
	_fixedForKF(-1),
	_isFirstKFConnection(true),
	_parentKF(nullptr)
{
	//位姿深拷贝
	_Rcw = frame._Rcw.clone();
	_tcw = frame._tcw.clone();
	_Rwc = frame._Rwc.clone();
	_Ow = frame._Ow.clone();
	_Tcw = frame._Tcw.clone();
	_Twc = frame._Twc.clone();
}

bool  operator == (const Frame& F,  KeyFrame& kF)
{
	KeyFrame& mutableKF = const_cast<KeyFrame&>(kF);
	Frame& mutableF = dynamic_cast<Frame&>(mutableKF);
	return mutableF._id == F._id;
}




Frame* KeyFrame::getRefInMap()
{
	FrameList& frameList = _globalMap->getFrameList();
	for (FrameList::iterator itr = frameList.begin(); itr != frameList.end(); itr++)
	{
		if ((*itr)->getID() == dynamic_cast<Frame*>(this)->getID())
		{
			return *itr;
		}
	}
	/*
	FrameList::iterator fItr = std::find(frameList.begin(), frameList.end(), *frame);
	if (fItr != frameList.end())
	{
		return *fItr;	
	}*/

	//std::cout << "Can't find the corresponding frame for keyFrame:" << this->getID() << std::endl;
	return nullptr;
	

}




void KeyFrame::updateKFConnections()
{
	std::unique_lock<std::mutex> lockConn(_connMutex);
	std::unique_lock<std::mutex> lockMatched(_matchedMutex);

	KFWeight KFCounter;

	for (MptKeyPairMap::iterator itr = _matchedMptMap.begin(); itr != _matchedMptMap.end(); itr++)
	{//遍历当前关键帧的匹配地图点
		MapPoint3d* mapPoint = itr->first;//mapPoint:当前关键帧的匹配地图点

		if (mapPoint->isBad())
			continue;
		KFSet& observationKFs = mapPoint->getObservationKFs();//获取观察到该地图点pMP的关键帧集合

		for (KFSet::iterator kFItr = observationKFs.begin(); kFItr != observationKFs.end(); kFItr++)
		{
			if ((*kFItr)->getID() == _id)
				continue;// 跳过当前关键帧
			KFCounter[*kFItr]++;//增加相应帧观察到该mapPoint的观察次数(与当前关键帧的共点数)
		}
	}

	if (KFCounter.empty())
		return;

	int nmax = 0;
	KeyFrame* kFMax = nullptr;
	int th = 0;


	for (KFWeight::iterator mit = KFCounter.begin(); mit != KFCounter.end(); mit++)
	{
		if (mit->second>=nmax)
		{
			nmax = mit->second;
			kFMax = mit->first;
		}
		if (mit->second >= th)
		{
			_orderedConnectedKFs.insert(std::make_pair(mit->first,mit->second));
			(mit->first)->addKFConnection(this, mit->second);
		}
	}

	if (_orderedConnectedKFs.empty())
	{
		_orderedConnectedKFs.insert(std::make_pair(kFMax,nmax));
		kFMax->addKFConnection(this, nmax);
	}

	if (_isFirstKFConnection && _id != 0)
	{
		_maxPrevConnKF = kFMax;
		_maxPrevConnKF->addMaxNextKF(this);
		_isFirstKFConnection = false;
	}

}




void KeyFrame::eraseKFConnection(KeyFrame* pKF)
{
	auto itr = _orderedConnectedKFs.find(pKF);
	if(itr!= _orderedConnectedKFs.end())
		_orderedConnectedKFs.erase(itr);
}




KeyFrameList KeyFrame::getBestCovisibleKeyFrames(const int &N)
{
	KeyFrameList bestCovisibleKFs;

	std::vector<std::pair< KeyFrame*,int>> covisibleList(_orderedConnectedKFs.begin(), _orderedConnectedKFs.end());
	sort(covisibleList.begin(), covisibleList.end(), DerefKFWightFunctor<std::pair<KeyFrame*,int>>());//按照观察帧数从小到大，若观察帧数相同，则按id号从小到大

	int count = 0;
	for (auto itr = covisibleList.rbegin(); itr != covisibleList.rend(); itr++, count++)
	{//按照观察帧数从大到小遍历前N各共点帧，当观察帧数相同，按ID号从大到小遍历
		if (count >= N)
			break;
		bestCovisibleKFs.emplace_back(itr->first);
	}
			
	return bestCovisibleKFs;
}



KeyFrameList KeyFrame::getCovisibleKFs() 
{ 
	KeyFrameList covisibleKFs;

	std::vector<std::pair<KeyFrame*, int>> covisibleList(_orderedConnectedKFs.begin(), _orderedConnectedKFs.end());
	sort(covisibleList.begin(), covisibleList.end(), DerefKFWightFunctor<std::pair<KeyFrame*, int>>());//按照观察帧数从小到大，若观察帧数相同，则按id号从小到大

	for (auto itr = covisibleList.rbegin(); itr != covisibleList.rend(); itr++)
	{//按照观察帧数从大到小遍历共点帧，当观察帧数相同，按ID号从大到小遍历

		covisibleKFs.emplace_back(itr->first);
	}
	return covisibleKFs;
}


int KeyFrame::getKFCovisibleCount(KeyFrame* pKF)
{
	auto itr = _orderedConnectedKFs.find(pKF);
	if (itr != _orderedConnectedKFs.end())
		return itr->second;
	else
		return 0;
}




void KeyFrame::updateBALocalForKF()
{
	FrameList& frameList = _globalMap->getFrameList();//包含了主线程已估位姿的前帧或当前帧
	KeyFrameList& kFList = _globalMap->getKeyFrameList();

	KeyFrame* prevKF = nullptr;
	if (_isRelocByRefKF || _isRelocByMap)
	{//帧-全局关键帧匹配,前向邻接_parentKF不是时间序列上的最近邻关键帧，需定位时间序列最近邻关键帧
		for (auto itrKF = kFList.rbegin(); itrKF != kFList.rend(); itrKF++)
		{
			if ((*itrKF)->_id < _id)
			{//存在当前关键帧未加入kFList的情况，因此判断序号大小定位邻接前向关键帧
				prevKF = *(itrKF);
				break;
			}
		}
	}
	else
		prevKF = _parentKF;//帧-帧匹配，时间序列的最近邻关键帧即为前向邻接_parentKF
	if (!prevKF)//跳过头关键帧
		return;

	Frame* prevFrame = dynamic_cast<KeyFrame*>(prevKF);
	FrameList::iterator itrPrevF = std::find(frameList.begin(), frameList.end(), prevFrame);
	//FrameList::iterator itrPrevF = frameList.begin() + (kFPrev->getRef()->getID() - (*frameList.begin())->getID());//计算当前关键帧的前一关键帧在关联帧容器中的位置（只有无帧删除时此式正确）
	if (itrPrevF == frameList.end())
		return;

	for (FrameList::iterator itrCurF = itrPrevF + 1; itrCurF != frameList.end(); itrCurF++)
	{
		if ((*itrCurF)->_refKF)
			continue;//跳过前一关键帧的局部帧
		if(dynamic_cast<KeyFrame*>(*itrCurF))
			continue;//跳过当前关键帧和下一关键帧的对应帧
		if (((*itrCurF)->_isRelocByMap || (*itrCurF)->_isRelocByRefKF) && (*itrCurF)->getID() > prevFrame->getID())
			continue;//跳过该关键帧之后的帧-全局地图或帧-全局关键帧匹配的帧

		if ((_isRelocByMap || _isRelocByRefKF) && (*itrCurF)->getID() < prevFrame->getID())
		{//若当前关键帧由帧-全局匹配得到，则所有该帧之前的帧以前一关键帧为BA参考
			prevKF->_localFrameSet.insert(*itrCurF);
			(*itrCurF)->_refKF = prevKF;
			(*itrCurF)->calcMotionModel(*prevKF);
			continue;
		}

		(*itrCurF)->_refKF = this;
		(*itrCurF)->calcMotionModel(*this);

		_localFrameSet.insert(*itrCurF);
	}

}





void KeyFrame::updateLocalPoses()
{
	for (FrameSet::iterator fItr = _localFrameSet.begin(); fItr != _localFrameSet.end(); fItr++)
	{
		Frame* pFrame = *fItr;
		if (pFrame->_refMotion.empty())
		{//保证关键帧从队列取出后（位姿未矫正）主线程最新帧邻接相对位置已更新
			std::cout << "No ref motion for Frame: " << pFrame->getID() << std::endl;
			continue;
		}

		pFrame->setPose(pFrame->_refMotion*_Tcw);
	}

}
void KeyFrame::updateAndCorrectLocalPoses(Matcher_<cv::Point3_, MapPoint_>::CalcSim3Func calcSim3Func, Matcher_<>::OrientFunc orientationFunc)
{
	if (calcMatchedRMSE() > Matcher_<>::MAX_PTBIAS_ERROR)
	{
		cv::Mat R, t;
		P3DMptPairList outLiers;
		P3DMptPairList p3DMptMatchedList;
		for (auto mItr = _matchedMptMap.begin(); mItr != _matchedMptMap.end(); mItr++)
		{
			MapPoint3d* pMP = mItr->first;
			if (pMP->isBad())
				continue;
			cv::Point3d* p3D = nullptr;
			getP3dFromKey(mItr->second.first, p3D);
			p3DMptMatchedList.emplace_back(std::make_pair(p3D, pMP));
		}
		if (calcSim3Func(p3DMptMatchedList, outLiers, R, t, orientationFunc, false))
		{
			for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
				insertOutLier(itr->second);
	
			setPose(R, t);
		}
		else
		{
			setBad(true);
			_globalMap->insertBadFrame(this);
		}
	}

	for (FrameSet::iterator fItr = _localFrameSet.begin(); fItr != _localFrameSet.end(); fItr++)
	{
		Frame* pFrame = *fItr;

		pFrame->setPose(pFrame->_refMotion*getPose());	
		if (pFrame->calcMatchedRMSE() > Matcher_<>::MAX_PTBIAS_ERROR)
		{
			cv::Mat R, t;
			P3DMptPairList outLiers;
			P3DMptPairList p3DMptMatchedList;
			for (auto mItr = pFrame->_matchedMptMap.begin(); mItr != pFrame->_matchedMptMap.end(); mItr++)
			{
				MapPoint3d* pMP = mItr->first;
				if (pMP->isBad())
					continue;
		
				cv::Point3d* p3D = mItr->second.first->getMatchedP3D();
				p3DMptMatchedList.emplace_back(std::make_pair(p3D, pMP));
			}
			if (calcSim3Func(p3DMptMatchedList, outLiers, R, t, orientationFunc, false))
			{
				for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
					pFrame->insertOutLier(itr->second);
						
				if (!pFrame->isBad())
				{
					pFrame->setPose(R, t);
					pFrame->calcMotionModel(*this);
				}			
			}
			else
			{
				pFrame->setBad(true);
				_globalMap->insertBadFrame(pFrame);
			}
		}

	}

}




/*********************************
递归修正孩子节点位姿
correctPose：当前关键帧矫正后的位姿
**********************************/
void KeyFrame::updateChildrenPoses(cv::Mat correctPose)
{
	if (_childrenKFs.empty())
		return;
	for (auto itr = _childrenKFs.begin(); itr != _childrenKFs.end(); itr++)
	{
		KeyFrame* childKF = *itr;
		cv::Mat correctChildPose = childKF->getPose()*getPoseInverse()*correctPose;
		childKF->updateChildrenPoses(correctChildPose);//递归修正
		childKF->setPose(correctChildPose);
		childKF->updateLocalPoses();
	}
	
}



void KeyFrame::cutAllChildrenKFs()
{
	for (auto itr = _childrenKFs.begin(); itr != _childrenKFs.end();)
	{
		(*itr)->cutAllChildrenKFs();
		itr =_childrenKFs.erase(itr);
	}
}


void KeyFrame::getAllChildrenKFs(KFSet& childrenKFs)
{
	for (auto itr = _childrenKFs.begin(); itr != _childrenKFs.end(); itr++)
	{
		(*itr)->getAllChildrenKFs(childrenKFs);
		childrenKFs.insert((*itr));
	}
}

void KeyFrame::eraseChild(KeyFrame* pKF)
{
	auto itr= _childrenKFs.find(pKF);
	if (itr != _childrenKFs.end())
		_childrenKFs.erase(itr);
}



bool KeyFrame::hasChild(KeyFrame* pKF)
{
	if (_childrenKFs.empty())
		return false;
	auto itr = _childrenKFs.find(pKF);
	if (itr != _childrenKFs.end())
		return true;
	else
	{
		for (auto itr = _childrenKFs.begin(); itr != _childrenKFs.end(); itr++)
		{
			if ((*itr)->hasChild(pKF))
			{
				return true;
			}
		}
	}
	return false;
}



}