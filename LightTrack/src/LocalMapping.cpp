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
		// ��ֹ���̴߳����ؼ�֡
		setAcceptKeyFrames(false);

		if (checkNewKeyFrames())
		{
			KeyFrame* pKF=proccessNewKeyFrame();

			_bAbortBA = false;//��ֵǰ���߳̿��ܣ�
							  //1��abortBA()��δ�����ؼ�֡��ӣ���ʱpKF�Ѿ����������µĹ����ϵ�������ֲ�BA��������
							  //2�������ؼ�֡��ӣ�����checkNewKeyFrames()����BA�Ż�
							  //��ֵ�����߳̿��ܣ�
							  //1��abortBA()��δ�����ؼ�֡��ӣ�����BA�Ż��ж�	
							  //2�������ؼ�֡��ӣ�����checkNewKeyFrames()����BA�Ż�
			
			if (!checkNewKeyFrames() && !stopRequested())
			{//checkNewKeyFrames():����ǰ���̴߳����ؼ�֡����ӣ��ֲ�BA������
			 //���ú����̴߳����ؼ�֡����ӣ�ͨ��_bAbortBA��ֹ�ֲ�BA���ֲ��ؼ�֡����ͼ������ǰ������
			 //stopRequested():����ǰ����requestStop()���ֲ�BA������
			 //���ú����requestStop()��ͨ��_bAbortBA��ֹ�ֲ�BA���ֲ��ؼ�֡����ͼ������ǰ������

				if (_globalMap->getKeyFrameList().size() >= 2)
				{					
					clock_t startTime=clock();

					Optimizer::localKFBundleAdjustment3D(pKF, _globalMap, _bAbortBA);
				
					clock_t endTime = clock();
					std::cout << "local Optimization time: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << std::endl;

					keyFrameCulling(pKF);//ɾ��pKF�ֲ�����ؼ�֡
				}		
			}
			globalMptCulling();//ɾ���ֲ�BAȷ���Ĵ����ͼ��

			if(!pKF->isBad())
				_loopCloser->addKeyFrame(pKF);

		}
		else if (stop())
		{//���_bStopRequested=true��stop()����true		
			while (isStopped() && !checkFinish())
			{//��_bFinishRequested=true����
				usleep(3000);
			}
			if (checkFinish())//_bFinishRequested��system::shutDown()����Ϊtrue
				break;//����BA�Ż����߳�
		}
		setAcceptKeyFrames(true);//���߳̿��Դ����ؼ�֡�����봦�����

		if (checkFinish())
			break;
		usleep(3000);
	}
	setFinish();	
}




void LocalMapping::globalMptCulling()
{
	MptSet badMptSet = _globalMap->getBadMpts();//��������
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
	{//����ɾ�����������ؼ�֡
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
		{//����ɾ��covisbleKF����_childrenKFs��ʧ�͸��ڵ�Ĺ�����Ϣ
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
			if (mptMatched->getObservationKFs().size() - 1 >= 5)//����(*kItr)����
				nRedundantObs++;
		}

		if (nRedundantObs == nMatches)//0.8*nMatches)
		{		
			std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//�������̸߳ı� _refKF
			if (covisibleKF == _tracker->_refKF)
				continue;//�������̲߳ο��ؼ�֡
			_globalMap->eraseKeyFrame(covisibleKF);
		}	

	}
	//ɾ�����ж�Ϊbad����ȷƥ���ͼ������<4����֡��ؼ�֡���ᵼ�����ڹؼ�֡�����㣩
	Frame::FrameCompSet frameSet = _globalMap->getBadFrames();
	for (auto fBadItr = frameSet.begin(); fBadItr != frameSet.end(); fBadItr++)
	{
		Frame* pFrameBad = *fBadItr;
		assert(pFrameBad->isBad());
	
		/*if (pFrameBad == _globalMap->getFrameList().front())
			continue;*/
		if (pFrameBad->isNotToErase())
			continue;
	
		std::unique_lock<std::mutex> lockTrack(Tracker::trackMutex());//�������̸߳ı� _refKF
		if (pFrameBad == _tracker->_refFrame)
			continue;//�������̲߳ο�֡

		KeyFrame* pKFBad = dynamic_cast<KeyFrame*>(pFrameBad);

		if (pKFBad)
		{
			if (pKFBad == pKF)
				continue;//������ǰ�ؼ�֡

			if (pKFBad == _tracker->_refKF)
				continue;//�������̲߳ο��ؼ�֡

			auto itr1 = std::find(_keyFrameQueue.begin(), _keyFrameQueue.end(), pKFBad);
			if (itr1 != _keyFrameQueue.end())
				_keyFrameQueue.erase(itr1);//ɾ���ֲ�BA����ʣ��bad�ؼ�֡�������ڴ���պ�ȡ��

			KFQueue& loopKFQueue = _loopCloser->getKFQueue();
			auto itr2 = std::find(loopKFQueue.begin(), loopKFQueue.end(), pKFBad);
			if (itr2 != loopKFQueue.end())
				loopKFQueue.erase(itr2);//ɾ���ػ�������ʣ��bad�ؼ�֡�������ڴ���պ�ȡ��
		}
			
		//_globalMap->getBadFrames().erase(pFrameBad);
		_globalMap->eraseFrame(pFrameBad);			
	}

}




KeyFrame* LocalMapping::proccessNewKeyFrame()
{
	std::unique_lock<std::mutex> lock(_mutexNewKFs);


	KeyFrame* pKF = _keyFrameQueue.front(); //pKF:�ӹؼ�֡���л�ȡ��Ϊ��ǰ�ؼ�֡
	_keyFrameQueue.pop_front();//�Ӷ��е����ùؼ�֡

	pKF->updateKFConnections();//����pKF�����ϵ�����µ����߳����¹ؼ�֡�����¹ؼ�֡��һ����pKF��

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

