#include "LocalMapping.h"
#include "System.h"
#include "Optimizer.h"

namespace SLAM
{


void LocalMapping::run()
{
	_bFinished = false;

	while (1)
	{
		// 禁止主线程创建关键帧
		setAcceptKeyFrames(false);

		if (checkNewKeyFrames())
		{
			KeyFrame* pKF=proccessNewKeyFrame();

			_bAbortBA = false;//赋值前主线程可能：
							  //1、abortBA()但未创建关键帧入队，此时pKF已经更新了最新的共点关系，后续局部BA正常进行
							  //2、创建关键帧入队，后续checkNewKeyFrames()跳过BA优化
							  //赋值后主线程可能：
							  //1、abortBA()但未创建关键帧入队，后续BA优化中断	
							  //2、创建关键帧入队，后续checkNewKeyFrames()跳过BA优化
			
			if (!checkNewKeyFrames() && !stopRequested())
			{//checkNewKeyFrames():调用前主线程创建关键帧并入队，局部BA不进行
			 //调用后主线程创建关键帧并入队，通过_bAbortBA终止局部BA（局部关键帧、地图点修正前跳出）
			 //stopRequested():调用前调用requestStop()，局部BA不进行
			 //调用后调用requestStop()，通过_bAbortBA终止局部BA（局部关键帧、地图点修正前跳出）

				if (_globalMap->getKeyFrameList().size() >= 2)
				{					
					clock_t startTime=clock();

					Optimizer::localKFBundleAdjustment3D(pKF, _globalMap, _bAbortBA);
				
					clock_t endTime = clock();
					std::cout << "local Optimization time: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << std::endl;

					keyFrameCulling(pKF);//删除pKF局部冗余关键帧
				}		
			}
			globalMptCulling();//删除局部BA确定的错误地图点

			if(!pKF->isBad())
				_loopCloser->addKeyFrame(pKF);

		}
		else if (stop())
		{//如果_bStopRequested=true则stop()返回true		
			while (isStopped() && !checkFinish())
			{//当_bFinishRequested=true跳出
				usleep(3000);
			}
			if (checkFinish())//_bFinishRequested在system::shutDown()中置为true
				break;//跳出BA优化子线程
		}
		setAcceptKeyFrames(true);//主线程可以创建关键帧并加入处理队列

		if (checkFinish())
			break;
		usleep(3000);
	}
	setFinish();	
}




void LocalMapping::globalMptCulling()
{
	MptSet badMptSet = _globalMap->getBadMpts();//不能引用
	for (auto pItr = badMptSet.begin(); pItr != badMptSet.end(); pItr++)
	{
		assert((*pItr)->isBad());
	
		_globalMap->eraseMapPoint(*pItr);
	}
}





void LocalMapping::keyFrameCulling(KeyFrame* pKF)
{
	KeyFrameList& covisibleKFs = pKF->getCovisibleKFs();
	for (auto kItr = covisibleKFs.begin(); kItr != covisibleKFs.end(); kItr++)
	{//优先删除共点数最多关键帧
		KeyFrame* covisibleKF = *kItr;
		if (covisibleKF == _globalMap->getKeyFrameList().front())
			continue;
		if (covisibleKF->isNotToErase())
			continue;
		if (covisibleKF->getID() > pKF->getID())
			continue;
		/*if (!covisibleKF->_newMptList.empty())
			continue;*/
		bool bNotToErased = false;
		for (auto cItr = covisibleKF->_childrenKFs.begin(); cItr != covisibleKF->_childrenKFs.end(); cItr++)
		{//避免删除covisbleKF导致_childrenKFs丢失和父节点的共点信息
			auto itr = covisibleKF->_parentKF->_orderedConnectedKFs.find(*cItr);
			if (itr == covisibleKF->_parentKF->_orderedConnectedKFs.end())
			{
				bNotToErased = true;
				break;
			}
		}
		if (bNotToErased)
			continue;

		int nRedundantObs = 0;
		int nMatches = 0;
		MptKeyPairMap& matchedMpts = covisibleKF->getMatchedMptMap();
		for (auto mItr = matchedMpts.begin(); mItr != matchedMpts.end(); mItr++)
		{
			MapPoint3d* mptMatched = mItr->first;
			if (mptMatched->isBad())
				continue;

			nMatches++;
			if (mptMatched->getObservationKFs().size() - 1 >= 5)//减掉(*kItr)自身
				nRedundantObs++;
		}

		if (nRedundantObs == nMatches)//0.8*nMatches)
		{		
			std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//避免主线程改变 _refKF
			if (covisibleKF == _tracker->_refKF)
				continue;//跳过主线程参考关键帧
			_globalMap->eraseKeyFrame(covisibleKF);
		}	

	}
	//删除被判断为bad（精确匹配地图点数量<4）的帧或关键帧（会导致相邻关键帧不共点）
	Frame::FrameCompSet frameSet = _globalMap->getBadFrames();
	for (auto fBadItr = frameSet.begin(); fBadItr != frameSet.end(); fBadItr++)
	{
		Frame* pFrameBad = *fBadItr;
		assert(pFrameBad->isBad());
	
		/*if (pFrameBad == _globalMap->getFrameList().front())
			continue;*/
		if (pFrameBad->isNotToErase())
			continue;
	
		std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//避免主线程改变 _refKF
		if (pFrameBad == _tracker->_refFrame)
			continue;//跳过主线程参考帧

		KeyFrame* pKFBad = dynamic_cast<KeyFrame*>(pFrameBad);

		if (pKFBad)
		{
			if (pKFBad == pKF)
				continue;//跳过当前关键帧

			if (pKFBad == _tracker->_refKF)
				continue;//跳过主线程参考关键帧

			auto itr1 = std::find(_keyFrameQueue.begin(), _keyFrameQueue.end(), pKFBad);
			if (itr1 != _keyFrameQueue.end())
				_keyFrameQueue.erase(itr1);//删除局部BA队列剩余bad关键帧，避免内存清空后被取出

			KFQueue& loopKFQueue = _loopCloser->getKFQueue();
			auto itr2 = std::find(loopKFQueue.begin(), loopKFQueue.end(), pKFBad);
			if (itr2 != loopKFQueue.end())
				loopKFQueue.erase(itr2);//删除回环检测队列剩余bad关键帧，避免内存清空后被取出
		}
			
		//_globalMap->getBadFrames().erase(pFrameBad);
		_globalMap->eraseFrame(pFrameBad);			
	}

}




KeyFrame* LocalMapping::proccessNewKeyFrame()
{
	std::unique_lock<std::mutex> lock(_mutexNewKFs);


	KeyFrame* pKF = _keyFrameQueue.front(); //pKF:从关键帧队列获取作为当前关键帧
	_keyFrameQueue.pop_front();//从队列弹出该关键帧

	pKF->updateKFConnections();//更新pKF共点关系（更新到主线程最新关键帧，最新关键帧不一定是pKF）

	return pKF;
}



void LocalMapping::release()
{
	std::unique_lock<std::mutex> lock1(_mutexStop);
	std::unique_lock<std::mutex> lock2(_mutexFinish);
	if (_bFinished)
		return;
	_bStopped = false;
	_bStopRequested = false;

	/*for (KFQueue::iterator lit = _keyFrameQueue.begin();  lit != _keyFrameQueue.end(); lit++)
		delete *lit;
	_keyFrameQueue.clear();*/

	std::cout << "Local Mapping released" << std::endl;
}

}

