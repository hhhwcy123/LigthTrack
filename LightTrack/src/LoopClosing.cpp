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
				_currentKF->setNotToErase(false);//��û��⵽�ػ�������ǰ���ؼ�֡��ɾ���ڴ�
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
	std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//����localMapping���߳�ɾ���ؼ�֡
	_currentKF = _keyFrameQueue.front();
	_keyFrameQueue.pop_front();
	_currentKF->setNotToErase(true);//����ùؼ�֡���ж�Ϊbad��������ڴ�

	//�ػ�����С��10���ؼ�֡
	if (!_lastKF)
	{
		if (_currentKF->getID() < 10)
			return false;
	}
	else
	{//���_currentKF�����µ㣬��������λ��ƫ������£����������
		if (((int)_currentKF->getID() - (int)_lastKF->getID() <10) && _currentKF->_newMptList.size() < 2)
			return false;
	}

	
	_condidateLoopKFs.clear();
	_backKFsSet.clear();

	if (findLoopForKF(_currentKF))
	{
		_loopKF->setNotToErase(true);
		_backKF->setNotToErase(true);//����ɾ��
		for (auto itr = _backKFsSet.begin(); itr != _backKFsSet.end(); itr++)
			(*itr)->setNotToErase(true);
		return true;
	}
	else
	{//δ�ҵ���ѡ֡
		return false;
	}
	
}





bool LoopClosing::findLoopForKF(KeyFrame* pKF)
{
	std::unique_lock<std::mutex> lockConn(KeyFrame::_connMutex);//����localMapping���¹���֡��localMapping���ܻ�������߳��½��ؼ�֡�����º�pKF�Ĺ����ϵ
	const KeyFrameList& covisibleKFs = pKF->getCovisibleKFs();
	KeyFrame::KFCompSet covisibleKFset(covisibleKFs.begin(), covisibleKFs.end());
	lockConn.unlock();

	if (covisibleKFset.empty())
		return false;
	
	cv::Mat R, t;
	int nNewMptThres = 1;
	if (pKF->_newMptList.size() > nNewMptThres)
	{//���µ�ͼ�������1��׼ȷ���µ� 2��pKF�����йؼ�֡���㣬�ұ�δƥ��ĵ�ͼ�� 3��pKF�ѻػ���δ��ƥ��ĵ�ͼ��(���)��		

		//��¼β������֡(�������ػ�����)
		KeyFrame* tmpBackKF = pKF->_parentKF;//��λβ����ʼKF
		while (covisibleKFset.count(tmpBackKF))
		{
			_backKFsSet.insert(tmpBackKF);//��¼β������֡

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
		{//���ݺ�ѡ
			KeyFrame* condidateKF = *itr;
			matchedP3DList.clear();
			
			if (!Matcher_<>::matchP3DByFrame(*condidateKF, *pKF, matchedP3DList, R, t,
											 _system->isDispErrorStats(),
											 Matcher_<>::orientFunc(_system->getOrientType()), Tracker::P3D_NEAR_MAX))
				continue;
		
			
			P3DMptPairList p3DMptMatchedList;
			if (Matcher_<cv::Point3_, MapPoint_>::searchMptMatchedByFrame(*condidateKF, *pKF, matchedP3DList, p3DMptMatchedList)==0)//��ǰ֡��ȡ��ǰһ֡��ƥ���ͼ��
				continue;

			bool bFound=false;
			for (auto nItr = pKF->_newMptList.begin(); nItr != pKF->_newMptList.end(); nItr++)
			{//�жϵ�ǰ֡ƥ��3D���Ƿ�����½���ͼ���3d��
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
			{//����������ػ�
				
				_loopKF = condidateKF;//�ײ�
				_backKF = pKF;//β��
				return true;
			}
			else
			{//�����ڷ���ػ�
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
					/////////////////�����ػ���β/////////////////
					std::unique_lock<std::mutex> lockConn(KeyFrame::_connMutex);//����localMapping���¹���֡��localMapping���ܻ�������߳��½��ؼ�֡�����º�pKF�Ĺ����ϵ
					const KeyFrameList& backCovisibleKFs = condidateKF->getCovisibleKFs();
					KFSet backCovisibleKFset(backCovisibleKFs.begin(), backCovisibleKFs.end());
					lockConn.unlock();

					//����β������֡(�������ػ�����)
					_backKFsSet.clear();
					_backKFsSet.insert(condidateKF);
					tmpBackKF = condidateKF->_parentKF;//��λβ����ʼKF
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

					_loopKF = pKF;//�ײ�
					_backKF = condidateKF;//β��
								
					return true;
				}
				else
				{
					_loopKF = condidateKF;//�ײ�
					_backKF = pKF;//β��
						
					return true;
				}
			
			}		
		}			
	}
	
 


	return false;
}




bool LoopClosing::loopCorrect()
{

	_localMapper->requestStop(); //����localMapping���̲߳��ϸ���_currentKF����֡���޸�֡λ�ˡ�ɾ���ؼ�֡��

	/////////�ȴ����ֲ߳̾�BA��ֹ//////////////////
	while (!_localMapper->isStopped())
	{
		usleep(1000);
	}

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//�����ͼ�㱻ɾ��
	//lockTrack����֤��
	//1�����߳�δ��������֡λ��
	//2�����̸߳���������֡λ���Ҽ���ȫ�ֵ�ͼ
	//3�����̸߳���������֡�ұ�����Ϊ�ؼ�֡��LocalMapping����

	//����β������֡backKFs

	_backKF->updateKFConnections();//�������¹����ϵ����_currentKF�����߳̿����ִ��������ɹؼ�֡��LocalMapping���������º�_currentKF�Ĺ����ϵ
	
	//std::unique_lock<std::mutex> lockConn(KeyFrame::_connMutex);//����LocalMapping�̸߳��¹���֡

	//����ͷ������ؼ�֡frontKFs���۲��ͼ��frontMptList
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
		{//����β���ؼ�֡
			kFItr=frontKFs.erase(kFItr);
			continue;
		}
			
		MptKeyPairMap& mptMatchedList = (*kFItr)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mptMatchedList.begin(); mit != mptMatchedList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (!pMP->isBad())
				frontMptSet.insert(pMP);//��ֲ���ͼ�㼯��localMapPoints���ͷ���ؼ�֡��ƥ���ͼ��	
		}
		kFItr++;
	}

	//����β������֡������֡�½���ͼ��λ�ˣ��ں���β����ͼ��
	KeyFrameAndPose correctedPosMap, nonCorrectedPosMap;
	std::map<KeyFrame*, std::set<KeyFrame *> > loopConnections;
	std::set<MapPoint3d*> backMptSet;

	for (auto kFItr = _backKFsSet.begin(); kFItr != _backKFsSet.end(); kFItr++)
	{//����β������֡
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

		///////////////////////�ں�β���½���ͼ��//////////////////
		int nFused = 0;	
		MptList newMptList = pBackKFi->getNewMptList();//�����ã������ں���newMptList�ı�
		if (!newMptList.empty())
		{
			for (auto pItr = newMptList.begin(); pItr != newMptList.end(); pItr++)
			{//ֻ�����ؼ�֡�½���ͼ��
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
				pBackKFi->updateKFConnections();//pBackKFiƥ���µ�ͼ�㣬���¼��㹲���ϵ			
			}
		}	

		//int nFused = 0;
		//MptKeyPairMap matchedMptList = pBackKFi->getMatchedMptMap();//�����ã������ں���newMptList�ı�
		//if (!matchedMptList.empty())
		//{
		//	for (auto mItr = matchedMptList.begin(); mItr != matchedMptList.end(); mItr++)
		//	{//ֻ�����ؼ�֡�½���ͼ��
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
		//		pBackKFi->updateKFConnections();//pBackKFiƥ���µ�ͼ�㣬���¼��㹲���ϵ					
		//	}
		//}
		pBackKFi->setPose(correctedPosMap[pBackKFi]);//����β������֡λ��
		pBackKFi->updateLocalPoses();//�����ڽ�֡λ��

		////���β��pKFi��ͷ������ؼ�֡ƥ���ϵ
		//const KeyFrameList& covisibleKFs = pBackKFi->getCovisibleKFs();
		//for (auto itr = covisibleKFs.begin(); itr != covisibleKFs.end(); itr++)
		//{
		//	if (frontKFs.count(*itr))
		//		loopConnections[pBackKFi].insert(*itr);
		//}
	}

	lockEraseMpt.unlock();
	//lockConn.unlock();
	lockTrack.unlock();//����������߳����¸���֡�ѱ��ػ���������������֡���Գ���������λ��Ϊ�ο�����	

	////����ȫ��λ��ͼ�Ż�
	//Optimizer::optimizeEssentialGraph(_globalMap, _loopKF, _backKF,
	//								  nonCorrectedPosMap, correctedPosMap,
	//							      loopConnections, true);

	//ȫ��BA�Ż�
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

	std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//��ͣ���̸߳�������֡
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//�����ͼ�㱻ɾ��
	
	kFList = _globalMap->getKeyFrameList();
	mptList = _globalMap->getMapPointList();
	for (auto fItr = kFList.begin(); fItr != kFList.end(); fItr++)
	{//�����ؼ�֡λ��
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
	{//������ͼ��λ��
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