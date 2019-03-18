#include "LoopClosing.h"
#include "System.h"
#include "Matcher.h"
#include "impl\Matcher.hpp"
#include "Optimizer.h"
#include "LocalMapping.h"
#include "GlobalMap.h"



namespace SLAM
{


void LoopClosing::run()
{
	_bFinished = false;
	while (1)
	{
		if (checkNewKeyFrames())
		{
			if (detectLoop())
			{
				loopCorrect();
			}
			else
				_currentKF->setNotToErase(false);//若没检测到回环，允许当前检测关键帧被删除内存
		}

		if (checkFinish())
			break;

		usleep(5000);
	}

	setFinish();
}




bool LoopClosing::detectLoop()
{
	std::unique_lock<std::mutex> lockQueue(_mutexLoopQueue);
	std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//避免localMapping子线程删除关键帧
	_currentKF = _keyFrameQueue.front();
	_keyFrameQueue.pop_front();
	_currentKF->setNotToErase(true);//避免该关键帧被判断为bad并被清空内存

	//回环不能小于10个关键帧
	if (!_lastKF)
	{
		if (_currentKF->getID() < 10)
			return false;
	}
	else
	{//如果_currentKF产生新点，可能属于位姿偏差过大导致，需立即检测
		if (((int)_currentKF->getID() - (int)_lastKF->getID() <10) && _currentKF->_newMptList.size() < 2)
			return false;
	}

	
	_condidateLoopKFs.clear();
	_backKFsSet.clear();

	if (findLoopForKF(_currentKF))
	{
		_loopKF->setNotToErase(true);
		_backKF->setNotToErase(true);//避免删除
		for (auto itr = _backKFsSet.begin(); itr != _backKFsSet.end(); itr++)
			(*itr)->setNotToErase(true);
		return true;
	}
	else
	{//未找到候选帧
		return false;
	}
	
}





bool LoopClosing::findLoopForKF(KeyFrame* pKF)
{
	std::unique_lock<std::mutex> lockConn(KeyFrame::_connMutex);//阻塞localMapping更新共点帧，localMapping可能会接收主线程新建关键帧并更新和pKF的共点关系
	const KeyFrameList& covisibleKFs = pKF->getCovisibleKFs();
	KeyFrame::KFCompSet covisibleKFset(covisibleKFs.begin(), covisibleKFs.end());
	lockConn.unlock();

	if (covisibleKFset.empty())
		return false;
	
	cv::Mat R, t;
	int nNewMptThres = 1;
	if (pKF->_newMptList.size() > nNewMptThres)
	{//有新地图点产生（1、准确的新点 2、pKF与所有关键帧共点，且被未匹配的地图点 3、pKF已回环，未被匹配的地图点(检测)）		

		//记录尾部共点帧(待初步回环矫正)
		KeyFrame* tmpBackKF = pKF->_parentKF;//定位尾部起始KF
		while (covisibleKFset.count(tmpBackKF))
		{
			_backKFsSet.insert(tmpBackKF);//记录尾部共点帧

			tmpBackKF = tmpBackKF->_parentKF;
			if (!tmpBackKF)
				break;			
		}	
		_backKFsSet.insert(pKF);
		KFSet backChildrenKFs;
		for (auto bCovisItr = covisibleKFset.begin(); bCovisItr != covisibleKFset.end(); bCovisItr++)
		{
			for (auto bItr = _backKFsSet.begin(); bItr != _backKFsSet.end(); bItr++)
			{
				if ((*bItr)->hasChild(*bCovisItr))
					backChildrenKFs.insert(*bCovisItr);
			}
		}
		if (!backChildrenKFs.empty())
			_backKFsSet.insert(backChildrenKFs.begin(), backChildrenKFs.end());

		KeyFrame::KFCompSet condidateKFs;
		P3DPairList matchedP3DList;

		KeyFrameList KFlist = _globalMap->getKeyFrameList();
		for (auto KFItr = KFlist.begin(); KFItr != KFlist.end(); KFItr++)
		{
			KeyFrame* pKFPrev= *KFItr;
			if ((int)pKF->getID() - (int)pKFPrev->getID() <= 10)
				continue;
			if (pKFPrev->isBad())
				continue;			
			if (!_backKFsSet.count(pKFPrev))
				condidateKFs.insert(pKFPrev);
		}
		bool bPositiveLoop = false;
		for (auto itr = condidateKFs.begin(); itr != condidateKFs.end(); itr++)
		{//正溯候选
			KeyFrame* condidateKF = *itr;
			matchedP3DList.clear();
			
			if (!Matcher_<>::matchP3DByFrame(*condidateKF, *pKF, matchedP3DList, R, t,
											 _system->isDispErrorStats(),
											 Matcher_<>::orientFunc(_system->getOrientType()), Tracker::P3D_NEAR_MAX))
				continue;
		
			
			P3DMptPairList p3DMptMatchedList;
			if (Matcher_<cv::Point3_, MapPoint_>::searchMptMatchedByFrame(*condidateKF, *pKF, matchedP3DList, p3DMptMatchedList)==0)//当前帧获取与前一帧的匹配地图点
				continue;

			bool bFound=false;
			for (auto nItr = pKF->_newMptList.begin(); nItr != pKF->_newMptList.end(); nItr++)
			{//判断当前帧匹配3D点是否包括新建地图点的3d点
				for (auto mItr = matchedP3DList.begin(); mItr != matchedP3DList.end(); mItr++)
				{
					if (mItr->first == pKF->getP3DFromMpt(*nItr) &&(condidateKF->getMptFromeP3D(mItr->second)))
					{
						bFound = true;
						break;
					}		
				}
				if (bFound)
					break;
			}
			if (!bFound)
				return false;

			P3DMptPairList outLiers;
			if (!Matcher_<cv::Point3_, MapPoint_>::calcSim3Successive(p3DMptMatchedList, outLiers, R, t,  Matcher_<cv::Point3_, MapPoint_>::absoluteOrientation, false))
				continue;
			for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
				pKF->insertOutLier(itr->second);
			
			_TcwBack = cv::Mat::eye(4, 4, CV_64F);
			R.copyTo(_TcwBack.colRange(0, 3).rowRange(0, 3));
			t.copyTo(_TcwBack.rowRange(0, 3).col(3));

			KeyFrame* tmpKF = pKF->_parentKF;
			while (tmpKF)
			{
				if (/*tmpKF == _backKF || */tmpKF == condidateKF)
				{
					bPositiveLoop = true;
					break;
				}	
				if (tmpKF == _backKF)
				{
					bPositiveLoop = false;
					break;
				}
				tmpKF = tmpKF->_parentKF;
			}
			if (bPositiveLoop)
			{//若属于正向回环
				
				_loopKF = condidateKF;//首部
				_backKF = pKF;//尾部
				return true;
			}
			else
			{//若属于反向回环
				KeyFrame* tmpKF = condidateKF;
				int count1 = 0;
				while (tmpKF)
				{
					count1++;
					tmpKF = tmpKF->_parentKF;
				}

				tmpKF = pKF;
				int count2 = 0;
				while (tmpKF)
				{
					count2++;
					tmpKF = tmpKF->_parentKF;
				}
				if (count1 > count2)
				{
					/////////////////交换回环首尾/////////////////
					std::unique_lock<std::mutex> lockConn(KeyFrame::_connMutex);//阻塞localMapping更新共点帧，localMapping可能会接收主线程新建关键帧并更新和pKF的共点关系
					const KeyFrameList& backCovisibleKFs = condidateKF->getCovisibleKFs();
					KFSet backCovisibleKFset(backCovisibleKFs.begin(), backCovisibleKFs.end());
					lockConn.unlock();

					//重算尾部共点帧(待初步回环矫正)
					_backKFsSet.clear();
					_backKFsSet.insert(condidateKF);
					tmpBackKF = condidateKF->_parentKF;//定位尾部起始KF
					while (backCovisibleKFset.count(tmpBackKF))
					{
						_backKFsSet.insert(tmpBackKF);

						tmpBackKF = tmpBackKF->_parentKF;
						if (!tmpBackKF)
							break;
					}

					KFSet backChildrenKFs;
					for (auto bCovisItr = backCovisibleKFset.begin(); bCovisItr != backCovisibleKFset.end(); bCovisItr++)
					{
						for (auto bItr = _backKFsSet.begin(); bItr != _backKFsSet.end(); bItr++)
						{
							if ((*bItr)->hasChild(*bCovisItr))
								backChildrenKFs.insert(*bCovisItr);	
						}
					}
					if (!backChildrenKFs.empty())
						_backKFsSet.insert(backChildrenKFs.begin(), backChildrenKFs.end());

					_loopKF = pKF;//首部
					_backKF = condidateKF;//尾部
								
					return true;
				}
				else
				{
					_loopKF = condidateKF;//首部
					_backKF = pKF;//尾部
						
					return true;
				}
			
			}		
		}			
	}
	
 


	return false;
}




bool LoopClosing::loopCorrect()
{

	_localMapper->requestStop(); //避免localMapping子线程不断更新_currentKF共点帧、修改帧位姿、删除关键帧、

	/////////等待子线程局部BA终止//////////////////
	while (!_localMapper->isStopped())
	{
		usleep(1000);
	}

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//避免地图点被删除
	//lockTrack锁保证：
	//1、主线程未跟踪最新帧位姿
	//2、主线程跟踪完最新帧位姿且加入全局地图
	//3、主线程跟踪完最新帧且被创建为关键帧入LocalMapping队列

	//计算尾部邻域帧backKFs

	_backKF->updateKFConnections();//更新最新共点关系，继_currentKF后主线程可能又创建了若干关键帧，LocalMapping来不及更新和_currentKF的共点关系
	
	//std::unique_lock<std::mutex> lockConn(KeyFrame::_connMutex);//阻塞LocalMapping线程更新共点帧

	//计算头部邻域关键帧frontKFs及观察地图点frontMptList
	const KeyFrameList& frontCovisibleKFs = _loopKF->getCovisibleKFs();
	KFSet frontKFs(frontCovisibleKFs.begin(), frontCovisibleKFs.end());

	frontKFs.insert(_loopKF);
	KFSet frontChildrenKFs;
	for (auto fItr = frontKFs.begin(); fItr != frontKFs.end(); fItr++)
	{
		KFSet childrenKFs = (*fItr)->getChildrenKFs();	
		for (auto cItr = childrenKFs.begin(); cItr != childrenKFs.end(); cItr++)
		{
			frontChildrenKFs.insert(*cItr);
		}
	}
	if (!frontChildrenKFs.empty())
		frontKFs.insert(frontChildrenKFs.begin(), frontChildrenKFs.end());

	std::set<MapPoint3d*> frontMptSet;
	for (auto kFItr = frontKFs.begin(); kFItr != frontKFs.end(); )
	{
		if (_backKFsSet.count(*kFItr))
		{//跳过尾部关键帧
			kFItr=frontKFs.erase(kFItr);
			continue;
		}
			
		MptKeyPairMap& mptMatchedList = (*kFItr)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mptMatchedList.begin(); mit != mptMatchedList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (!pMP->isBad())
				frontMptSet.insert(pMP);//向局部地图点集合localMapPoints添加头部关键帧的匹配地图点	
		}
		kFItr++;
	}

	//修正尾部邻域帧及邻域帧新建地图点位姿，融合首尾部地图点
	KeyFrameAndPose correctedPosMap, nonCorrectedPosMap;
	std::map<KeyFrame*, std::set<KeyFrame *> > loopConnections;
	std::set<MapPoint3d*> backMptSet;

	for (auto kFItr = _backKFsSet.begin(); kFItr != _backKFsSet.end(); kFItr++)
	{//遍历尾部共点帧
		KeyFrame* pBackKFi = *kFItr;
		if (pBackKFi == _backKF)
		{
			correctedPosMap[pBackKFi] = _TcwBack;
			nonCorrectedPosMap[pBackKFi] = pBackKFi->getPose();
		}
		else
		{
			cv::Mat Tic = pBackKFi->getPose()*_backKF->getPoseInverse();
			correctedPosMap[pBackKFi] = Tic*_TcwBack;
			nonCorrectedPosMap[pBackKFi] = pBackKFi->getPose();
		}

		///////////////////////融合尾部新建地图点//////////////////
		int nFused = 0;	
		MptList newMptList = pBackKFi->getNewMptList();//非引用，避免融合中newMptList改变
		if (!newMptList.empty())
		{
			for (auto pItr = newMptList.begin(); pItr != newMptList.end(); pItr++)
			{//只矫正关键帧新建地图点
				MapPoint3d* newBackMpt = *pItr;
				if (!frontMptSet.count(newBackMpt) && !backMptSet.count(newBackMpt))
				{
					backMptSet.insert(newBackMpt);
					cv::Mat x3Dw = cv::Mat(dynamic_cast<cv::Point3d&>(**pItr));
					cv::Mat x3Dc = pBackKFi->_Rcw*x3Dw + pBackKFi->_tcw;
		
					cv::Mat RwcCorrected = correctedPosMap[pBackKFi].colRange(0, 3).rowRange(0, 3).t();
					cv::Mat	OwCorrected = -RwcCorrected*correctedPosMap[pBackKFi].col(3).rowRange(0, 3);
		
					cv::Mat x3DwCorrected = RwcCorrected*x3Dc + OwCorrected;
					newBackMpt->setWorldPos(x3DwCorrected);
					//int nFused = _globalMap->searchAndFuse(*pItr, _globalMap->getMapPointList());
					if (_globalMap->searchAndFuse(newBackMpt, frontMptSet))
						nFused++;
				}		
			}
			if (nFused > 0)
			{
				fprintf(stdout, "%d MapPoints have been fused by loop correcting!\n", nFused);
				pBackKFi->updateKFConnections();//pBackKFi匹配新地图点，重新计算共点关系			
			}
		}	

		//int nFused = 0;
		//MptKeyPairMap matchedMptList = pBackKFi->getMatchedMptMap();//非引用，避免融合中newMptList改变
		//if (!matchedMptList.empty())
		//{
		//	for (auto mItr = matchedMptList.begin(); mItr != matchedMptList.end(); mItr++)
		//	{//只矫正关键帧新建地图点
		//		MapPoint3d* matchedMpt = mItr->first;
		//		if (!frontMptSet.count(matchedMpt) && !backMptSet.count(matchedMpt))
		//		{					
		//			cv::Mat x3Dw = cv::Mat(dynamic_cast<cv::Point3d&>(*matchedMpt));
		//			cv::Mat x3Dc = pBackKFi->_Rcw*x3Dw + pBackKFi->_tcw;
		//
		//			cv::Mat RwcCorrected = correctedPosMap[pBackKFi].colRange(0, 3).rowRange(0, 3).t();
		//			cv::Mat	OwCorrected = -RwcCorrected*correctedPosMap[pBackKFi].col(3).rowRange(0, 3);
		//
		//			cv::Mat x3DwCorrected = RwcCorrected*x3Dc + OwCorrected;
		//			matchedMpt->setWorldPos(x3DwCorrected);
		//
		//			//int nFused = _globalMap->searchAndFuse(*pItr, _globalMap->getMapPointList());
		//			if (_globalMap->searchAndFuse(matchedMpt, frontMptSet))
		//				nFused++;
		//			if(!matchedMpt->isBad())
		//				backMptSet.insert(matchedMpt);
		//		}
		//	}
		//	if (nFused > 0)
		//	{
		//		fprintf(stdout, "%d MapPoints have been fused by loop correcting!\n", nFused);
		//		pBackKFi->updateKFConnections();//pBackKFi匹配新地图点，重新计算共点关系					
		//	}
		//}
		pBackKFi->setPose(correctedPosMap[pBackKFi]);//修正尾部共点帧位姿
		pBackKFi->updateLocalPoses();//修正邻接帧位姿

		////获得尾部pKFi和头部共点关键帧匹配关系
		//const KeyFrameList& covisibleKFs = pBackKFi->getCovisibleKFs();
		//for (auto itr = covisibleKFs.begin(); itr != covisibleKFs.end(); itr++)
		//{
		//	if (frontKFs.count(*itr))
		//		loopConnections[pBackKFi].insert(*itr);
		//}
	}

	lockEraseMpt.unlock();
	//lockConn.unlock();
	lockTrack.unlock();//锁解除，主线程最新跟踪帧已被回环初步矫正，后续帧将以初步矫正后位姿为参考跟踪	

	////进行全局位姿图优化
	//Optimizer::optimizeEssentialGraph(_globalMap, _loopKF, _backKF,
	//								  nonCorrectedPosMap, correctedPosMap,
	//							      loopConnections, true);

	//全局BA优化
	runGlobalBundleAdjustment();
	
	_backKF->addLoopKF(_loopKF);
	_loopKF->addLoopKF(_backKF);
	for (auto itr = _backKFsSet.begin(); itr != _backKFsSet.end(); itr++)
	{
		if (!(*itr)->_loopKFs.empty())
			continue;
		(*itr)->setNotToErase(false);
	}

	_localMapper->release();
	 
	_lastKF = _currentKF;

	return true;
}




void LoopClosing::runGlobalBundleAdjustment()
{
	std::cout << "Starting Global Bundle Adjustment" << std::endl;

	
	KeyFrameList kFList = _globalMap->getKeyFrameList();
	MptList mptList= _globalMap->getMapPointList();
	int maxKFId = kFList.back()->getID();

	Optimizer::globalBundleAdjustment(kFList, mptList, _loopKF, _backKFsSet, 10);

	std::cout << "Global Bundle Adjustment finished" << std::endl;

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//暂停主线程跟踪最新帧
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//避免地图点被删除
	
	kFList = _globalMap->getKeyFrameList();
	mptList = _globalMap->getMapPointList();
	for (auto fItr = kFList.begin(); fItr != kFList.end(); fItr++)
	{//矫正关键帧位姿
		KeyFrame* pKF = *fItr;
		if (pKF->_id > maxKFId)
		{
			pKF->_TcwGBA = pKF->getPose()*pKF->_parentKF->getPoseInverse() * pKF->_parentKF->_TcwGBA;				
			for (auto pItr = pKF->_newMptList.begin(); pItr != pKF->_newMptList.end(); pItr++)
			{
				MapPoint3d* pMP = *pItr;
				if (pMP->_refKF != pKF)
					std::cout << "Wrong refKF for map point" << std::endl;
				cv::Mat x3Dw = cv::Mat(dynamic_cast<cv::Point3d&>(*pMP));
				cv::Mat x3Dc = pKF->_Rcw*x3Dw + pKF->_tcw;

				cv::Mat RwcCorrected = pKF->_TcwGBA.colRange(0, 3).rowRange(0, 3).t();
				cv::Mat	OwCorrected = -RwcCorrected * pKF->_TcwGBA.col(3).rowRange(0, 3);

				cv::Mat x3DwCorrected = RwcCorrected*x3Dc + OwCorrected;
				pMP->_posGBA = x3DwCorrected;
			}	
		}
		pKF->setPose(pKF->_TcwGBA);
	}
	for (auto mItr = mptList.begin(); mItr != mptList.end(); mItr++)
	{//矫正地图点位姿
		MapPoint3d* pMP = *mItr;
		if (pMP->_posGBA.empty())
			continue;
		pMP->setWorldPos(pMP->_posGBA);
		pMP->_coordLast = *dynamic_cast<cv::Point3d*>(pMP);	
		/*if (cv::norm(pMP->_coordLast - pMP->_coordOri)>0.5)
			_globalMap->updateMptDistList(pMP, Tracker::P3D_DIST_MAX);*/
	}
	for (auto kFItr = kFList.begin(); kFItr != kFList.end(); kFItr++)
		(*kFItr)->updateAndCorrectLocalPoses(Matcher_<cv::Point3_, MapPoint_>::calcSim3Func(_system->getCalcSim3Type()), 
		                                     Matcher_<>::orientFunc(_system->getOrientType()));

}




}