#include "GlobalMap.h"
#include "Matcher.h"
#include "impl\Matcher.hpp"
#include "Optimizer.h"
#include "LocalMapping.h"
 
namespace SLAM
{
unsigned GlobalMap::_frameCount = 0;
unsigned GlobalMap::_keyFrameCount = 0;
unsigned GlobalMap::_mapPointCount = 0;
std::mutex GlobalMap::_frameMutex;
std::mutex GlobalMap::_mptMutex;
std::mutex GlobalMap::_laserMutex;

GlobalMap::~GlobalMap()
{
	/*if (_hEvent)
	{
		CloseHandle(_hEvent);
		_hEvent = NULL;
	}	*/
	if (!_mapPointList.empty())
	{
		for (MptList::iterator itr = _mapPointList.begin(); itr != _mapPointList.end();)
		{
			delete *itr;
			*itr = nullptr;
			itr = _mapPointList.erase(itr);
		}
	}
	if (!_frameList.empty())
	{
		for (FrameList::iterator itr = _frameList.begin(); itr != _frameList.end();)
		{
			(*itr)->_keyPointsLeft.clear();
			(*itr)->_keyPointsRight.clear();
			(*itr)->_p3DMap.clear();
			delete *itr;
			*itr = nullptr;
			itr = _frameList.erase(itr);
		}
	}
	if (!_keyFrameList.empty())
	{
		for (KeyFrameList::iterator itr = _keyFrameList.begin(); itr != _keyFrameList.end();)
		{
			itr = _keyFrameList.erase(itr);
		}
	}
	_refMapPointList.clear();
}



void GlobalMap::release() 
{
	//if (_hEvent)
	//{
	//	CloseHandle(_hEvent);
	//	_hEvent = NULL;
	//}
	if (!_mapPointList.empty())
	{
		for (MptList::iterator itr = _mapPointList.begin(); itr != _mapPointList.end();)
		{
			delete *itr;
			*itr = nullptr;
			itr = _mapPointList.erase(itr);
		}
	}
	if (!_frameList.empty())
	{
		for (FrameList::iterator itr = _frameList.begin(); itr != _frameList.end();)
		{
			Frame* framePtr = *itr;
			delete framePtr;

			*itr = nullptr;
			itr = _frameList.erase(itr);
		}
	}

	if (!_keyFrameList.empty())
	{
		for (KeyFrameList::iterator itr = _keyFrameList.begin(); itr != _keyFrameList.end();)
		{
			itr = _keyFrameList.erase(itr);
		}
	}
	
	_refMapPointList.clear();

	setMptCount(0);
	setFrameCount(0);
	setKeyFrameCount(0);
}



bool GlobalMap::searchAndFuse(MapPoint3d* mpt, std::set<MapPoint3d*>& mptSet)
{
	//std::unique_lock<std::mutex> lockObs(MapPoint_::_obsMutex);//锁观察帧集合
	double minDist = DBL_MAX;
	MapPoint3d* bestMpt = nullptr;
	if (mpt->isBad())
		return false;
	for (auto pItr = mptSet.begin(); pItr != mptSet.end(); pItr++)
	{
		MapPoint3d* tmpMpt = *pItr;
		if (tmpMpt->isBad())
			continue;
		if (tmpMpt == mpt)
			continue;
		double dist = cv::norm(*mpt - *tmpMpt);
		if (dist < Tracker::MPT_MATCH_THRES)
		{
			if (dist < minDist)
			{
				minDist = dist;
				bestMpt = tmpMpt;
			}
		}
	}
	if (bestMpt)
	{
		std::unique_lock<std::mutex> lockFrame(_frameMutex);//锁观察帧

		FrameSet observations = mpt->getObservations();//未加锁（避免锁嵌套），非引用
		if (observations.empty())
			return false;

		for (FrameSet::iterator fItr = observations.begin(); fItr != observations.end(); fItr++)
		{//遍历mpt的观察帧
			Frame* obsFrame = (*fItr);
	
  			KeyPointPair mathedKeyPair;
			obsFrame->getKeyPairFromMpt(mpt, mathedKeyPair);
	
			obsFrame->eraseMptMatched(mpt);
			
			obsFrame->insertMatchedMPt(bestMpt, mathedKeyPair.first, mathedKeyPair.second);//替换该观察帧的匹配地图点为tmpMpt
			bestMpt->addObservation(obsFrame);
			KeyFrame* obsKF = dynamic_cast<KeyFrame*>(obsFrame);
			if(obsKF)
				bestMpt->addKFObservation(obsKF);

			if (obsFrame->getOutLiers().count(mpt))
			{
				obsFrame->eraseOutlier(mpt);
				cv::Point3d* p3D = obsFrame->getP3DFromMpt(mpt);
				if (obsFrame->calcMptMatchedError(p3D, bestMpt) > Matcher_<>::MAX_PTBIAS_ERROR)
					obsFrame->insertOutLier(bestMpt);							
			}
		}	
		//eraseMapPoint(mpt);//全局地图删除该地图点mpt,避免锁嵌套通过fBad标志位删除，
		return true;
	}
	
	return false;
}



void GlobalMap::eraseMapPoint(MapPoint3d* pMP)
{
	//WaitForSingleObject(_hEvent, INFINITE);
	std::unique_lock<std::mutex> lockEraseMpt(_mptMutex);

	MptList::iterator mItr = std::find(_mapPointList.begin(), _mapPointList.end(), pMP);
	assert(mItr != _mapPointList.end());

	_mapPointList.erase(mItr);


	FrameSet observations = pMP->getObservations();//避免引用
	for (FrameSet::iterator fItr = observations.begin(); fItr != observations.end(); fItr++)
	{
		(*fItr)->eraseMptMatchedWithP3D(pMP);//pMP会重新加入_badMptSet
	}
		
	//deleteMptDist(mapPoint,Tracker::P3D_DIST_MAX);

	MptList::iterator rItr=std::find(_refMapPointList.begin(), _refMapPointList.end(), pMP);
	if (rItr != _refMapPointList.end())
	{
		_refMapPointList.erase(rItr);
	}

	_badMptSet.erase(pMP);
	delete pMP;
	pMP = nullptr;

	//SetEvent(_hEvent);
}



bool GlobalMap::eraseMapPoint(const int& id)
{
	for (auto itr = _mapPointList.begin(); itr != _mapPointList.end(); itr++)
	{
		if ((*itr)->getID() == id)
		{
			eraseMapPoint(*itr);
			return true;
		}
	}
	return false;
}



bool GlobalMap::eraseKeyFrame(KeyFrame* pKF)
{
	std::unique_lock<std::mutex> lockFrame(_frameMutex);//帧删除锁
	
	if (pKF->isNotToErase())
		return false;

	//替换全局帧元素
	FrameList::iterator fItr = std::find(_frameList.begin(), _frameList.end(), dynamic_cast<Frame*>(pKF));
	assert(fItr != _frameList.end());

	Frame* pFrame = new Frame(*pKF);
	*fItr = pFrame;

	//std::unique_lock<std::mutex> lockEraseMpt(MapPoint_::_obsMutex);
	std::unique_lock<std::mutex> lockMatched(Frame::_matchedMutex);
	/////删除剩余匹配地图点的观察关键帧///////
	for (MptKeyPairMap::iterator itr = pKF->_matchedMptMap.begin(); itr != pKF->_matchedMptMap.end(); itr++)
	{
		MapPoint3d* mptMatched = itr->first;
		mptMatched->eraseKFObservation(pKF);
		mptMatched->eraseObservation(dynamic_cast<Frame*>(pKF));
		mptMatched->addObservation(pFrame);
	}
	lockMatched.unlock();


	//lockEraseMpt.unlock();

	//////删除对应关键帧共点关系///////
	for (KeyFrame::KFWeight::iterator itr = pKF->_orderedConnectedKFs.begin(); itr != pKF->_orderedConnectedKFs.end(); itr++)
	{
		itr->first->eraseKFConnection(pKF);
	}
	pKF->_orderedConnectedKFs.clear();

	// Update Spanning Tree
	//std::set<KeyFrame*> maxPrevCandidates;
	//if (pKF->_maxPrevConnKF)
	//	maxPrevCandidates.insert(pKF->_maxPrevConnKF);
	//while (!pKF->_maxNextConnKFs.empty())
	//{
	//	bool bContinue = false;
	//	int max = -1;
	//	KeyFrame* pNext = nullptr;
	//	KeyFrame* pPrev = nullptr;
	//
	//	for (std::set<KeyFrame*>::iterator sit = pKF->_maxNextConnKFs.begin(); sit != pKF->_maxNextConnKFs.end(); sit++)
	//	{// 遍历孩子节点childKF
	//		KeyFrame* maxNextConnKF = *sit;
	//
	//		for (std::set<KeyFrame*>::iterator spcit = maxPrevCandidates.begin(); spcit != maxPrevCandidates.end(); spcit++)
	//		{
	//			auto citr = maxNextConnKF->_orderedConnectedKFs.find(*spcit);
	//			if (citr!= maxNextConnKF->_orderedConnectedKFs.end())
	//			{//如果孩子节点与候选父节点存在共点关系（共点关键帧包含候选父节点）
	//				if (citr->second > max)
	//				{//选出共点数最多的候选父节点作为该孩子节点maxNextConnKF的父亲
	//					pNext = maxNextConnKF;
	//					pPrev = citr->first;
	//					max = citr->second;
	//					bContinue = true;
	//				}
	//			}
	//		}
	//		
	//	}
	//	if (bContinue)
	//	{//更新被删除帧的孩子节点的父节点和父节点的孩子节点
	//		pNext->setMaxPrevKF (pPrev);
	//		pPrev->addMaxNextKF(pNext);
	//
	//		maxPrevCandidates.insert(pNext); //已更新父节点的孩子节点加入候选父节点，下一轮循环搜索其它孩子节点并更新
	//									 //候选父节点：包括被删除帧的父节点和已更新过的孩子节点，因为被删除帧的的孩子节点和父节点是与
	//									 //被删除帧共点数最多的，因此更新其它孩子节点的父节点只需搜索被删帧的父节点和孩子节点就可以了
	//		pKF->eraseMaxNextKF(pNext);
	//	}
	//	else
	//		break;
    //
	//}
	////如果被删帧的孩子节点不与任何候选父节点有共点连接关系
	//if (!pKF->_maxNextConnKFs.empty())
	//	for (KFSet::iterator sItr = pKF->_maxNextConnKFs.begin(); sItr != pKF->_maxNextConnKFs.end(); sItr++)
	//	{
	//		(*sItr)->changeMaxPrevKF(pKF->_maxPrevConnKF);
	//	}
	//if (pKF->_maxPrevConnKF)
	//	pKF->_maxPrevConnKF->eraseMaxNextKF(pKF);

	// 更新前后向最大连接关系
	if (pKF != _keyFrameList.front())
		if (pKF->getMaxPrevKF())
			pKF->getMaxPrevKF()->_maxNextConnKFs.erase(pKF);
		
	if (!pKF->_maxNextConnKFs.empty())
	{
		for (auto itrNext = pKF->_maxNextConnKFs.begin(); itrNext != pKF->_maxNextConnKFs.end(); itrNext++)
		{
			KeyFrame* maxNextKF = *itrNext;
			int nMaxCovis = INT_MIN;
			KeyFrame* bestPrevConnKF = nullptr;
			for (auto cItr = maxNextKF->_orderedConnectedKFs.begin(); cItr != maxNextKF->_orderedConnectedKFs.end(); cItr++)
			{
				KeyFrame* connKF = cItr->first;
				if (connKF->getID() < maxNextKF->getID())
				{
					if (cItr->second > nMaxCovis)
					{
						nMaxCovis = cItr->second;
						bestPrevConnKF = connKF;
					}
					else if (cItr->second == nMaxCovis)
					{
						if (connKF->getID() > bestPrevConnKF->getID())
						{
							bestPrevConnKF = connKF;
						}
					}
				}
			}
			maxNextKF->changeMaxPrevKF(bestPrevConnKF);
		}
	}

	/*std::unique_lock<std::mutex> lockLocalCorrect(Optimizer::localOptimizeMutex());*///此处的锁会和导致程序线程阻塞
	//重算被删关键帧的局部邻接帧相对邻接关键帧的相对运动关系
	Frame::FrameCompSet& localFrames = pKF->getLocalFrames();
	KeyFrame* refKF = nullptr;
	if (pKF->_parentKF || !pKF->_childrenKFs.empty())
	{//一般为头尾部关键帧或帧-全局匹配后的关键帧跳过
		if (pKF->_parentKF)
			refKF = pKF->_parentKF;
		else if (!pKF->_childrenKFs.empty())
			refKF = *pKF->_childrenKFs.begin();

		for (FrameSet::iterator itr = localFrames.begin(); itr != localFrames.end(); itr++)
		{
			(*itr)->_refKF = refKF;
			(*itr)->calcMotionModel(*refKF);
			refKF->addLocalFrame((*itr));
		}
		//连带修改替换的帧对象
		pFrame->_refKF = refKF;
		pFrame->calcMotionModel(*refKF);
		refKF->addLocalFrame(pFrame);
	}

	/*lockLocalCorrect.unlock();*/

	//更新前后向邻接关系
	if (!pKF->_parentKF)
	{//当前关键帧是帧-全局地图匹配且无共点（>3）帧
		for (auto itr = pKF->_childrenKFs.begin(); itr != pKF->_childrenKFs.end(); itr++)
		{//后一帧为帧-全局地图匹配时，_childrenKFs为空
			(*itr)->_parentKF = nullptr;
		}
	}
	else
	{
		pKF->_parentKF->_childrenKFs.erase(pKF);
		if (!pKF->_childrenKFs.empty())
		{
			for (auto itr = pKF->_childrenKFs.begin(); itr != pKF->_childrenKFs.end(); itr++)
			{//后一帧为帧-全局地图匹配时，_childrenKFs为空
				pKF->_parentKF->_childrenKFs.insert(*itr);
				(*itr)->_parentKF = pKF->_parentKF;
			}
		}
	}
		
	//////从关键帧列表删除该关键帧///////
	KeyFrameList::iterator kFItr = std::find(_keyFrameList.begin(), _keyFrameList.end(), pKF);
	if (kFItr != _keyFrameList.end())
	{
		_keyFrameList.erase(kFItr);
		std::cout << "Erased keyFrame ID:" << pKF->getID() << std::endl;
	}
	auto bKFItr = _badFrameSet.find(pKF);
	if (bKFItr != _badFrameSet.end())
	{
		_badFrameSet.erase(bKFItr);
		_badFrameSet.insert(pFrame);
	}

	return true;	
}



bool GlobalMap::eraseFrame(Frame* pF)
{
	std::unique_lock<std::mutex> lockFrame(_frameMutex);//帧删除锁

	if (pF->isNotToErase())
		return false;

	////////////从帧列表删除该帧/////////////
	FrameList::iterator fItr = std::find(_frameList.begin(), _frameList.end(), pF);
	assert(fItr != _frameList.end());
	_frameList.erase(fItr);
	
	KeyFrame* pKF = dynamic_cast<KeyFrame*>(pF);

	///////////////删除关联关键帧/////////////
	if (pKF)
	{	
		//////从关键帧列表删除该关键帧///////
		KeyFrameList::iterator kFItr = std::find(_keyFrameList.begin(), _keyFrameList.end(), pKF);
		assert(kFItr != _keyFrameList.end());
		_keyFrameList.erase(kFItr);

		/////删除剩余匹配地图点的观察关键帧///////
		//std::unique_lock<std::mutex> lockEraseMpt(MapPoint_<>::_obsMutex);
		for (MptKeyPairMap::iterator itr = pKF->_matchedMptMap.begin(); itr != pKF->_matchedMptMap.end(); itr++)
		{
			itr->first->eraseKFObservation(pKF);
		}
		//lockEraseMpt.unlock();

		//////删除对应关键帧共点关系///////
		for (KeyFrame::KFWeight::iterator itr = pKF->_orderedConnectedKFs.begin(); itr != pKF->_orderedConnectedKFs.end(); itr++)
		{
			itr->first->eraseKFConnection(pKF);
		}
		pKF->_orderedConnectedKFs.clear();

		// 更新前后向最大连接关系
		if (pKF != _keyFrameList.front())
			if(pKF->getMaxPrevKF())
				pKF->getMaxPrevKF()->_maxNextConnKFs.erase(pKF);

		if (!pKF->_maxNextConnKFs.empty())
		{
			for (auto itrNext = pKF->_maxNextConnKFs.begin(); itrNext != pKF->_maxNextConnKFs.end(); itrNext++)
			{
				KeyFrame* maxNextKF = *itrNext;
				int nMaxCovis = INT_MIN;
				KeyFrame* bestPrevConnKF = nullptr;
				for (auto cItr = maxNextKF->_orderedConnectedKFs.begin(); cItr != maxNextKF->_orderedConnectedKFs.end(); cItr++)
				{
					KeyFrame* connKF = cItr->first;
					if (connKF->getID() < maxNextKF->getID())
					{
						if (cItr->second > nMaxCovis)
						{
							nMaxCovis = cItr->second;
							bestPrevConnKF = connKF;
						}
						else if (cItr->second == nMaxCovis)
						{
							if (connKF->getID() > bestPrevConnKF->getID())
							{
								bestPrevConnKF = connKF;
							}
						}
					}
				}
				maxNextKF->changeMaxPrevKF(bestPrevConnKF);
			}
		}
		/*std::unique_lock<std::mutex> lockLocalCorrect(Optimizer::localOptimizeMutex());*///此处的锁会和导致程序线程阻塞
																						   //重算被删关键帧的局部邻接帧相对前向邻接关键帧的相对运动关系
		Frame::FrameCompSet& localFrames = pKF->getLocalFrames();
		KeyFrame* refKF = nullptr;
		if (pKF->_parentKF || !pKF->_childrenKFs.empty())
		{//一般为头尾部关键帧或帧-全局匹配后的关键帧跳过
			if (pKF->_parentKF)
				refKF = pKF->_parentKF;
			else if (!pKF->_childrenKFs.empty())
				refKF = *pKF->_childrenKFs.begin();

			for (FrameSet::iterator itr = localFrames.begin(); itr != localFrames.end(); itr++)
			{
				if (*itr == pF)
					continue;
				(*itr)->_refKF = refKF;
				(*itr)->calcMotionModel(*refKF);
				refKF->addLocalFrame((*itr));
			}
		}

		//更新前后向邻接关系
		if (!pKF->_parentKF)
		{//一般为头部关键帧或帧-全局地图匹配后的关键帧
			for (auto itr = pKF->_childrenKFs.begin(); itr != pKF->_childrenKFs.end(); itr++)
			{
				(*itr)->_parentKF = nullptr;
			}
		}
		else
		{
			pKF->_parentKF->_childrenKFs.erase(pKF);
			if (!pKF->_childrenKFs.empty())
			{
				for (auto itr = pKF->_childrenKFs.begin(); itr != pKF->_childrenKFs.end(); itr++)
				{//后一帧为帧-全局地图匹配时，_childrenKFs为空
					pKF->_parentKF->_childrenKFs.insert(*itr);
					(*itr)->_parentKF = pKF->_parentKF;
				}
			}
		}
	}

	//////删除匹配地图点的观察帧///////
	std::unique_lock<std::mutex> lockMatched(Frame::_matchedMutex);
	MptKeyPairMap& mathedMpts = pF->_matchedMptMap;//非引用，避免地图点删除影响
	for (MptKeyPairMap::iterator itr = mathedMpts.begin(); itr != mathedMpts.end(); itr++)
	{
		MapPoint3d* mptMatched = itr->first;
		mptMatched->eraseObservation(pF);
		//if (itr->first->obs() == 0)//如果观察帧数为0，从全局地图中删除
		//	eraseMapPoint(itr->first);
	}	
	lockMatched.unlock();

	pF->clearOutLiers();

	//////删除共点帧关系///////
	for (auto itr = pF->_orderedConnecteds.begin(); itr != pF->_orderedConnecteds.end(); itr++)
	{
		itr->first->eraseConnection(pF);
	}
	pF->_orderedConnecteds.clear();

	///////参考关键帧邻接帧集合删除该帧/////////////////
	if (pF->_refKF)
		pF->_refKF->eraseLocalFrame(pF);

	//////////////从坏帧集合删除该帧/////////////////
	auto bFItr = _badFrameSet.find(pF);
	if (bFItr != _badFrameSet.end())
		_badFrameSet.erase(bFItr);
		
	////////////////释放内存////////////////
	std::cout << "Erased Frame ID:" << pF->getID() << std::endl;
	delete pF;
	pF = nullptr;

	return true;
}



void GlobalMap::calcMptDistList(MapPoint3d* mpt, MptDistMap& mptDistMap)
{
	for (MptList::iterator mItr = _mapPointList.begin(); mItr != _mapPointList.end(); mItr++)
	{
		double distTmp = cv::norm(*mpt - **mItr);
		_mptDistMap[distTmp] = std::make_pair(mpt, *mItr);
	}
	
}



void GlobalMap::addMptDistList(MapPoint3d* mpt, double th)
{
	std::unique_lock<std::mutex> lockMpt(_mptMutex);
	for (MptList::iterator mItr = _mapPointList.begin(); mItr != _mapPointList.end(); mItr++)
	{
		if (mpt == *mItr)
			continue;
		double distTmp = cv::norm(*mpt - **mItr);
		if (distTmp > th)
			continue;
		_mptDistMap[distTmp] = std::make_pair(mpt,*mItr);
		mpt->addConnSite(_mptDistMap[distTmp], distTmp);
		(*mItr)->addConnSite(std::make_pair(*mItr, mpt), distTmp);//保证pair第一个元素总是指向调用的地图点对象
	}
}



void GlobalMap::updateMptDistList(MapPoint3d* mpt, double th)
{
	for (MptList::iterator mItr = _mapPointList.begin(); mItr != _mapPointList.end(); mItr++)
	{
		if (mpt == *mItr)
			continue;
		double distTmp = cv::norm(*mpt - **mItr);
		if (distTmp > th)
			continue;

		MptDistMap::iterator startItr = _mptDistMap.lower_bound(distTmp - 20);
		MptDistMap::iterator endItr = _mptDistMap.upper_bound(distTmp + 20);
		if (startItr == _mptDistMap.end() || endItr == _mptDistMap.begin())
			continue;

		for (MptDistMap::iterator itr = startItr; itr != endItr;)
		{
			if ((itr->second.first == mpt&&itr->second.second == *mItr) ||
				(itr->second.first == *mItr&&itr->second.second == mpt))
			{
				itr = _mptDistMap.erase(itr);

				MptDistMap::iterator startItr1 = mpt->_connSiteMap.lower_bound(distTmp - 20);
				MptDistMap::iterator endItr1 = mpt->_connSiteMap.upper_bound(distTmp + 20);
				if (startItr1 == mpt->_connSiteMap.end() || endItr1 == mpt->_connSiteMap.begin())
					continue;

				MptPair mptPair1 = std::make_pair(mpt, *mItr);
				for (MptDistMap::iterator itr1 = startItr1; itr1 != endItr1;)
				{
					if (itr1->second == mptPair1)
					{
						itr1 = mpt->_connSiteMap.erase(itr1);
						break;
					}
					else
						itr1++;
				}
				mpt->_connSiteMap[distTmp] = mptPair1;

				MptDistMap::iterator startItr2 = (*mItr)->_connSiteMap.lower_bound(distTmp - 20);
				MptDistMap::iterator endItr2 = (*mItr)->_connSiteMap.upper_bound(distTmp + 20);
				if (startItr2 == (*mItr)->_connSiteMap.end() || endItr2 == (*mItr)->_connSiteMap.begin())
					continue;

				MptPair mptPair2 = std::make_pair(*mItr, mpt);
				for (MptDistMap::iterator itr2 = startItr2; itr2 != endItr2;)
				{
					if (itr2->second == mptPair2)
					{
						itr2 = (*mItr)->_connSiteMap.erase(itr2);
						break;
					}
					else
						itr2++;
				}
				(*mItr)->_connSiteMap[distTmp] = mptPair2;
				break;
			}
			else
				itr++;
		}
		_mptDistMap[distTmp] = std::make_pair(mpt, *mItr);
		
	}
}


void GlobalMap::deleteMptDist(MapPoint3d* mpt, double th)
{
	for (MptList::iterator mItr = _mapPointList.begin(); mItr != _mapPointList.end(); mItr++)
	{
		if (mpt == *mItr)
			continue;
		double distTmp = cv::norm(*mpt - **mItr);
		if (distTmp > th)
			continue;
		for (MptDistMap::iterator itr = _mptDistMap.begin(); itr != _mptDistMap.end();)
		{
			if ((itr->second.first == mpt&&itr->second.second == *mItr) ||
				(itr->second.first == *mItr&&itr->second.second == mpt))
			{
				itr = _mptDistMap.erase(itr);
				/*
				MptPair mptPair1 = std::make_pair(mpt, *mItr);
				for (MptDistMap::iterator itr1 = mpt->_connSiteMap.begin(); itr1 != mpt->_connSiteMap.end();)
				{
					if (itr1->second == mptPair1)
					{
						itr1 = mpt->_connSiteMap.erase(itr1);
						break;
					}
					else
						itr1++;
				}*/
			
				MptPair mptPair2 = std::make_pair(*mItr, mpt);
				for (MptDistMap::iterator itr2 = (*mItr)->_connSiteMap.begin(); itr2 != (*mItr)->_connSiteMap.end();)
				{
					if (itr2->second == mptPair2)
					{
						itr2 = (*mItr)->_connSiteMap.erase(itr2);

						break;
					}
					else
						itr2++;
				}
				break;
			}
			else
				itr++;

		}
		mpt->_connSiteMap.clear();
	}
}





}


