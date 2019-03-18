#include "Tracker.h"
#include "Optimizer.h"
#include "System.h"
#include "Converter.h"
#include "impl\Matcher.hpp"

#include <Eigen\Core>

namespace SLAM
{

//��ͼ����ɾ��
const int Tracker::FRAME_THRES = 40;
const int Tracker::OBSERVE_THRES = 10;//20;//������۲�֡��

//�ؼ�֡���֡��
#ifdef STAGE 
const int Tracker::KEYFRAME_INTERVAL = 1;
#else
const int Tracker::KEYFRAME_INTERVAL = 10;
#endif

//�ֲ�����ͼ������
const int Tracker::LOCAL_MAX_MPTS = 500;

//��֡����3d�������С����
#ifdef STAGE 
const double Tracker::P3D_NEAR_MAX = 300;//350
#else
const double Tracker::P3D_NEAR_MAX = 150;//300;//350
#endif
const double Tracker::P3D_NEAR_MIN = 10;

//��֡3d��������
#ifdef STAGE 
const double Tracker::P3D_DIST_MAX = 1000;//350
#else
const double Tracker::P3D_DIST_MAX = 250;//350
#endif

//��ͼ��-2d��ͶӰƥ��
const double Tracker::KEY_MATCH_THRES = 5;

//3d-��ͼ�㷴ͶӰƥ��
const double Tracker::MPT_MATCH_THRES = 3.;//5//10;//20

//����֡-֡R��t���
#ifdef STAGE 

const double Tracker::ROTATION_THRES = 180;

const double Tracker::TRANSPOSE_THRES = 1000;
#else

const double Tracker::ROTATION_THRES = 20;

const double Tracker::TRANSPOSE_THRES = 100;
#endif




std::mutex Tracker::_trackMutex;
KeyFrame* Tracker::_relocRefKF = nullptr;

bool Tracker::createTrackImg(Frame& frameCur, Frame& framePrev)
{
	int wl, hl, wr, hr, w, h;

	wl = cvRound(frameCur._markImgLeft.cols);
	hl = cvRound(frameCur._markImgLeft.rows);
	wr = cvRound(framePrev._markImgLeft.cols);
	hr = cvRound(framePrev._markImgLeft.rows);
	w = wl + wr;
	h = hl > hr ? hl : hr;
	_canvasImg.create(h, w, CV_8UC3);
	if (_canvasImg.empty())
	{
		std::cout << "Failed creating track canvas image!" << std::endl;
		return false;
	}
	return true;
}



bool Tracker::writeTrackMatchedImg(Frame& frameCur, Frame& framePrev, const P3DPairList& matchedP3DList)
{
	if (_canvasImg.empty())
		return false;
	
	int wl, hl, wr, hr;
	wl = cvRound(frameCur._markImgLeft.cols);
	hl = cvRound(frameCur._markImgLeft.rows);
	wr = cvRound(framePrev._markImgLeft.cols);
	hr = cvRound(framePrev._markImgLeft.rows);
		
	//cv::Mat imgLeftBGR, imgRightBGR;
	//cvtColor(frameCur._markImgLeft, imgLeftBGR, cv::COLOR_GRAY2BGR);
	//cvtColor(framePrev._markImgLeft, imgRightBGR, cv::COLOR_GRAY2BGR);
	cv::Mat canvasLeft = _canvasImg(cv::Rect(0, 0, wl, hl));
	cv::Mat canvasRight = _canvasImg(cv::Rect(wl, 0, wr, hr));
	resize(frameCur._markImgLeft, canvasLeft, canvasLeft.size(), 0, 0, cv::INTER_AREA);
	resize(framePrev._markImgLeft, canvasRight, canvasRight.size(), 0, 0, cv::INTER_AREA);


	for (P3DPairList::const_iterator itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++)
	{//���Ƶ�ǰ֡��ǰ֡��ƥ���ͼ�����ߣ���ͼ���Ӧ��2D�㣩����ɫ��
		cv::Point3d* p3DCur = itr->first;
		cv::Point3d* p3DPrev = itr->second;
		KeyPointPair keyPointPairCur; 
		KeyPointPair keyPointPairPrev;

		if (!frameCur.getKeyPairFromP3D(p3DCur, keyPointPairCur))
			return false;
		if (!framePrev.getKeyPairFromP3D(p3DPrev, keyPointPairPrev))
			return false;
		
		KeyPoint* keyPointLeftCur = keyPointPairCur.first;
		KeyPoint* keyPointLeftPrev = keyPointPairPrev.first;

		cv::Point& p2DCur = cv::Point(keyPointLeftCur->_undistortX, keyPointLeftCur->_undistortY);
		cv::Point& p2DPrev = cv::Point(keyPointLeftPrev->_undistortX+wl, keyPointLeftPrev->_undistortY);
	
			
		//cv::circle(_canvasImg, p2DCur, 4, cv::Scalar(255, 0, 0), 1);
		//cv::circle(_canvasImg, p2DPrev, 4, cv::Scalar(255, 0, 0), 1);
		cv::line(_canvasImg, p2DCur, p2DPrev, cv::Scalar(0, 255, 0));
			
	}
	
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//����localMapping���߳�ɾ����ͼ��
	
	for (MptKeyPairMap::iterator itr = frameCur._matchedMptMap.begin(); itr != frameCur._matchedMptMap.end(); itr++)
	{//���Ƶ�ǰ֡����ƥ��ĵ�ͼ�㣬������ȫ�ֵ�ͼƥ��õ��ģ���ͼ���Ӧ��2D�㣩����ȷ����ɫ����ɢ����ɫ��
		char indexStr[5];
		itoa(itr->first->getID(), indexStr, 10);
		cv::Point posBias = cv::Point(8, -8);
		cv::Point& p2DCur = cv::Point(itr->second.first->_undistortX, itr->second.first->_undistortY);
		putText(_canvasImg, indexStr, p2DCur + posBias, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);

		cv::Point point(itr->second.first->_undistortX, itr->second.first->_undistortY);
		if (frameCur._outLiers.count(itr->first))
			cv::circle(_canvasImg, point, 8, cv::Scalar(0, 255, 255), 2);
		else
			cv::circle(_canvasImg, point, 8, cv::Scalar(0, 255, 0), 2);
	}

	return true;
}



bool Tracker::writeTrackNewMptImg(Frame& frameCur, Frame& framePrev)
{
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//����localMapping���߳�ɾ����ͼ��

	if (_canvasImg.empty())
		return false;

	int wl = cvRound(frameCur._markImgLeft.cols);

	KeyFrame* pKFCur = dynamic_cast<KeyFrame*>(&frameCur);
	//���Ƶ�ǰ֡�½���ͼ�㣨��ɫ��
	if (pKFCur)
	{
		MptList& curNewMptList = pKFCur->_newMptList;
		for (MptList::iterator itr = curNewMptList.begin(); itr != curNewMptList.end(); itr++)
		{//����frameCur���½���ͼ�㼯��
			MptKeyPairMap::iterator newItr = frameCur._matchedMptMap.find(*itr);
			if (newItr != frameCur._matchedMptMap.end())
			{
				char indexStr[5];
				itoa(newItr->first->getID(), indexStr, 10);
				cv::Point posBias = cv::Point(8, -8);

				cv::Point& p2DCur = cv::Point(newItr->second.first->_undistortX, newItr->second.first->_undistortY);
				putText(_canvasImg, indexStr, p2DCur + posBias, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 2);
				cv::circle(_canvasImg, p2DCur, 8, cv::Scalar(0, 0, 255), 2);
			}
		}
	}
	KeyFrame* pKFPrev = dynamic_cast<KeyFrame*>(&framePrev);
	if (pKFPrev)
	{//����ǰ֡�½���ͼ�㣨��ɫ��
		MptList& prevNewMptList = pKFPrev->_newMptList;
		for (MptList::iterator itr = prevNewMptList.begin(); itr != prevNewMptList.end(); itr++)
		{//����frameCur���½���ͼ�㼯��
			MptKeyPairMap::iterator newItr = framePrev._matchedMptMap.find(*itr);
			if (newItr != framePrev._matchedMptMap.end())
			{
				char indexStr[5];
				itoa(newItr->first->getID(), indexStr, 10);
				cv::Point posBias = cv::Point(8, -8);

				cv::Point& p2DPrev = cv::Point(newItr->second.first->_undistortX + wl, newItr->second.first->_undistortY);
				putText(_canvasImg, indexStr, p2DPrev + posBias, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 2);
				cv::circle(_canvasImg, p2DPrev, 8, cv::Scalar(0, 0, 255), 2);
			}
		}
	}
	
}









bool Tracker::initTracker(Frame& curFrame)
{
	_refFrame = nullptr;
	_refKF = nullptr;
	P3DKeyPairMap& point3DMap = curFrame._p3DMap;
	if (point3DMap.size()>3)
	{
		if (_globalMap->getMapPointList().size() > 3)
		{//���ص�ͼ�ļ���ɨ
			if (!trackMap(curFrame, 
						  Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()), 
						  Matcher_<>::orientFunc(_system->getOrientType())))
			{
				fprintf(stdout, "Frame %d tracking map failed!\n", curFrame.getID());
				return false;
			}	
		}
		else
			curFrame.setPose(cv::Mat::eye(4, 4, CV_64F));//���ó�ʼframeλ��		
		
		 KeyFrame* newKeyFrame = createKeyFrame(curFrame);

		////////���¸��ٲο�////////
		if(newKeyFrame)
		{	
			_refFrame = dynamic_cast<Frame*>(newKeyFrame);
			_refKF = newKeyFrame;
			fprintf(stdout, "Creating new key frame succeeded: %d!\n", newKeyFrame->getID());
		}
		else
		{
			fprintf(stdout, "Failed Creating new key frame: %d!\n", newKeyFrame->getID());
			return false;
		}	

		

		_globalMap->setReferenceMapPoints(_globalMap->getMapPointList());
		_globalMap->addFrame(dynamic_cast<Frame*>(newKeyFrame));

		_mapDrawer->setCurrentCameraPose(curFrame.getPose());
	}
	else
	{
		fprintf(stdout, "Frame %d skipped: Has no valid keyPoint\n", curFrame.getID());
		return false;
	}
	return true;
}


bool Tracker::preTrack(Frame& curFrame)
{
	std::unique_lock<std::mutex> lockTrack(_trackMutex);//֧�ֶ��߳�Ԥ����

	if (curFrame._p3DMap.size() < 4)
		return false;

	if (_globalMap->getFrameList().empty())
		return true;

	bool bTrackRef = false;
	if (!trackRefFrame(*_refKF, curFrame, 
					   Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()), 
					   Matcher_<>::orientFunc(_system->getOrientType())))
	{
		if (_refKF != dynamic_cast<KeyFrame*>(_refFrame))
		{
			if (trackRefFrame(*_refFrame, curFrame, 
							  Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()),
							  Matcher_<>::orientFunc(_system->getOrientType())))
				bTrackRef = true;
		}
		if (!bTrackRef)
		{
			//////////////////�ض�λ/////////////////
			std::cout << "Relocalizing..." << std::endl;
			RelocState relocState = relocalizing(curFrame);
			if (relocState == Reloc_Ref)
				return true;
			else if (relocState == Reloc_Map)
				return true;		
			else
			{
				fprintf(stderr, "Frame %d relocalized failed!\n", curFrame.getID());
				return false;
			}
		}
	}
	return true;
	
}



bool Tracker::tracking(Frame& curFrame)
{
	_currentFrame = &curFrame;
	if (_globalMap->getFrameList().empty())
	{//��ʼ��	
		if (!initTracker(curFrame))
		{
			std::cout << "Initiating tracking failed!" << std::endl;
			return false;
		}
	}
	else
	{
		P3DKeyPairMap& point3DMap = curFrame._p3DMap;
		if (point3DMap.size()<4)
		{
			fprintf(stdout, "Frame %d skipped: Has no valid keyPoint\n", curFrame.getID());
			return false;
		}

		std::unique_lock<std::mutex> lockTrack(_trackMutex);//��ǰ����֡�����ֲ߳̾�BA֡����ͬ��

		bool bTrackRef = false;
		bool bTrackRefKF = true;
		////////////////////֡-֡3d��ƥ�����////////////////////
		if (!trackRefFrame(*_refKF, curFrame, 
						   Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()),
						   Matcher_<>::orientFunc(_system->getOrientType())))
		{
			fprintf(stderr, "Frame %d tracked by reference keyframe failed!\n", curFrame.getID());
			bTrackRefKF = false;
			if (_refKF != dynamic_cast<KeyFrame*>(_refFrame))
			{
				if (trackRefFrame(*_refFrame, curFrame, 
								  Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()),
								  Matcher_<>::orientFunc(_system->getOrientType())))
				{
					fprintf(stderr, "Frame %d tracked by reference frame succeeded!\n", curFrame.getID());
					bTrackRef = true;
				}
					
			}				
			if(!bTrackRef)
			{
				//////////////////�ض�λ/////////////////
				std::cout << "Relocalizing..." << std::endl;
				RelocState relocState = relocalizing(curFrame);
				if (relocState == Reloc_Ref)
				{
					fprintf(stderr, "Frame %d relocalized by previous keyframes succeeded!\n", curFrame.getID());
					bTrackRefKF = true;
				}
				else if (relocState == Reloc_Map)
				{
					fprintf(stderr, "Frame %d relocalized by global map succeeded!\n", curFrame.getID());
					bTrackRefKF = true;
				}
				else
				{
					fprintf(stderr, "Frame %d relocalized failed!\n", curFrame.getID());
					return false;
				}
			}		
		}
		
		////////////////����ǰ��֡����ͼ�񲢻��Ƶ�ǰ֡ƥ���ͼ��///////////////
		if (_system->isOutputTrackImg())
		{
			Frame* refFrame = bTrackRef ? _refFrame : dynamic_cast<Frame*>(_refKF);
			createTrackImg(curFrame, *refFrame);
			writeTrackMatchedImg(curFrame, *refFrame, _matchedP3DList);
		}		
		

		////////////////////�����ؼ�֡///////////////
		KeyFrame* newKeyFrame = nullptr;
		if (!bTrackRefKF)
		{//����_refFrame�����ؼ�֡
			newKeyFrame = createKeyFrame(*_refFrame);
			if (!newKeyFrame)
				return false;
			//ǰ֡����Ϊ�ؼ�֡�����¸��ٵ�ǰ֡(ǰ֡���ܴ����µ�ͼ��)
		/*	if (!trackRefFrame(*newKeyFrame, curFrame))
				return false;*/
		}
		else if (needNewKeyFrame(curFrame))
		{//����curFrame�����ؼ�֡			
			newKeyFrame = createKeyFrame(curFrame);
			if (!newKeyFrame)
				return false;
		}	
		
		////////////////////���¸��ٲο�////////////////////////
		if (newKeyFrame)
		{
			fprintf(stdout, "Creating new keyframe succeeded: %d!\n", newKeyFrame->getID());

			if (bTrackRef)
			{
				auto fItr = std::find(_globalMap->_frameList.begin(), _globalMap->_frameList.end(), _refFrame);
				*fItr = newKeyFrame;
			
				newKeyFrame->_refKF->eraseLocalFrame(_refFrame);

				curFrame._refKF = newKeyFrame;
				newKeyFrame->addLocalFrame(&curFrame);

				_refFrame = &curFrame;	
			}
			else
			{
				_refFrame = newKeyFrame;	
			}
			newKeyFrame->_refKF = nullptr;		

			_refKF = newKeyFrame;
		}		
		else
		{
			curFrame._refKF = _refKF;
			_refKF->addLocalFrame(&curFrame);
			_refFrame = &curFrame;
			_refKF = curFrame._refKF;
		}

		
		if (newKeyFrame&&!bTrackRef)
			_globalMap->addFrame(dynamic_cast<Frame*> (newKeyFrame));
		else
			_globalMap->addFrame(&curFrame);

		lockTrack.unlock();//curFrame����newKeyFrame���ڽ�֡��localMapping�ľֲ�BA���ܽ����ؼ�֡���ڽ�֡
		
		 //////////////����쳣��ͼ��//////////////////
		mapPointCulling(curFrame);
	
		_globalMap->setReferenceMapPoints(_globalMap->getMapPointList());//������һ�θ��ٵĲο���ͼ�㼯��

		_mapDrawer->setCurrentCameraPose(curFrame.getPose());//���ó����۲����λ��
	
	}
	
	if (_system->isOutputPos())
	{
		if (!curFrame.outputPos(_system->_posFile))
			std::cout << "Failed output R,t for curFrame: " << curFrame.getID() << std::endl;
		
	}
	if (_system->isOutputTrackImg())
	{
		if (!ReadWriter::saveTrackImg(_system->getTrackImgOutDir(), this))
			std::cout << "Failed saving track canvas image" << std::endl;
	}

	
	return true;
}




bool Tracker::needNewKeyFrame(const Frame& curFrame)
{
	//���ֲ�BA���ػ���ֹ���򲻴����ؼ�֡
	if (_localMapper->isStopped() || _localMapper->stopRequested())
		return false;
	//��һ֡����
	if (_globalMap->getFrameList().empty())
		return true;

	bool flag = false;

	KeyFrame* lastkeyFrame = _globalMap->getKeyFrameList().back();
	Frame* lastFrame = dynamic_cast<Frame*>(const_cast<KeyFrame*> (lastkeyFrame));

	if (!flag)
	{//����һ�ؼ�֡��֡�����	
		if(curFrame.getID() - lastFrame->getID() >= Tracker::KEYFRAME_INTERVAL || curFrame._isRelocByMap)
			flag = true;
	}
	if(!flag)
	{
		const int& p3DSize = curFrame._p3DMap.size();//3d������
		const int& matchedSize = curFrame._matchedMptMap.size();//ƥ���3d��������������ɢƥ�䣩

		if (p3DSize - matchedSize >1)
		{//��������2��δƥ���3d�㣬��������������
			flag = true;
		}
	}
	if (!flag)
	{
		std::unique_lock<std::mutex> lock(Frame::_matchedMutex);//refFrame��_matchedMptMap�����ھֲ�BA���߳�ɾ��

		unsigned int nCovisMpts = 0;
		std::set<MapPoint3d*> refMptMatchedSet, curMptMathcedSet;

		for (auto itr = curFrame._matchedMptMap.begin(); itr != curFrame._matchedMptMap.end(); itr++)
		{
			if (_refKF->_matchedMptMap.count(itr->first));
				nCovisMpts++;
		}
		double matchedRatio = (double)nCovisMpts / (double)_refKF->_matchedMptMap.size();//����һ�ؼ�֡ƥ�乲ͬ��ͼ��ı���
		if (matchedRatio<0.4)
		{//����һ�ؼ�֡���ӵ������С�������ؼ�֡�乲��̶�
			flag = true;
		}
	}
	if (flag)
	{
		if (!_localMapper->isAcceptKeyFrames())
		{//���߳��Ƿ���ܹؼ�֡
			if (_localMapper->keyframesInQueue() < 3)
			{//�����̴߳�����а���3�����Ϲؼ�֡δ�����򲻴����µĹؼ�֡

				_localMapper->abortBA();//�������ؼ�֡�������������߳����ھֲ�BA,���ֲ߳̾�BA��δ��ɵ�����λ��������������
				return true;
			}
			else
				return false;
		}
		return true;
	}
	else
		return false;
	
}




KeyFrame* Tracker::createKeyFrame(Frame& frame)
{
	//frame.updateConnections();//���㵱ǰ�ؼ�֡��ǰ֡�Ĺ����ϵ�����¹���ؼ�֡����
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//�����ͼ�㱻ɾ�����±���pKFƥ���ͼ��ʱ�����������frame����pKF֮ǰ��

	KeyFrame* pKF = new KeyFrame(frame);
	if (!pKF)
		return nullptr;

	/////////////////ɾ�����滻ԭframe��������������Ĺ�ϵ//////////////////////
	MptKeyPairMap& matchedMptList = pKF->getMatchedMptMap();
	for (MptKeyPairMap::iterator itr = matchedMptList.begin(); itr != matchedMptList.end(); itr++)
	{
		MapPoint3d* mapPoint = itr->first;
		mapPoint->addObservation(dynamic_cast<Frame*>(pKF));
		mapPoint->addKFObservation(pKF);
		mapPoint->eraseObservation(&frame);
	}

	lockEraseMpt.unlock();

	for (auto fItr = frame._orderedConnecteds.begin(); fItr != frame._orderedConnecteds.end(); fItr++)
	{
		Frame* frameConn = fItr->first;
		frameConn->addConnection(dynamic_cast<Frame*>(pKF), frameConn->getCovisibleCount(&frame));
		frameConn->eraseConnection(&frame);
	}
	for (auto fItr = frame._maxNextConns.begin(); fItr != frame._maxNextConns.end(); fItr++)
	{
		Frame* frameNextMax = *fItr;
		frameNextMax->_maxPrevConn = dynamic_cast<Frame*>(pKF);
	}
	//���¹ؼ�֡ǰ�����ڽӹ�ϵ	
	pKF->_parentKF = _refKF;
	if (_refKF)
		_refKF->_childrenKFs.insert(pKF);

	/////////////////����ȫ�ֵ�ͼ//////////////////
	int newMptCount = 0;
	if (newMptCount = expandGlobalMap(pKF))
	{
		fprintf(stdout, "Frame %d created %d new map points\n", frame.getID(), newMptCount);
		if (_system->isOutputTrackImg())
		{//���Ƶ�ǰ֡�������µ�ͼ�㲢����ͼ��
			if(&frame==_refFrame)
				writeTrackNewMptImg(frame, *frame._refKF);
			else
				writeTrackNewMptImg(frame, *_refFrame);
		}
	}


	_localMapper->addKeyFrame(pKF);//��֤������ؼ�֡�����Ϣ�����
	_globalMap->addKeyFrame(pKF);//��Ӻ����ȫ�ֵ�ͼ����֤BA�Ż�ǰ�������ؼ�֡Ҳ��ȫ�ֵ�ͼ�����ؼ�֡��


	return pKF;
}




bool Tracker::trackRefFrame(Frame& refFrame, Frame& currentFrame, 
						    Matcher_<cv::Point3_, MapPoint_, double>::CalcSim3Func calcSim3Func,
						    Matcher_<>::OrientFunc orientationFunc,
						    const double p3dMathcedRatio)
{
	if (refFrame.isBad())
		return false;
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//����localMapping���߳�ɾ����ͼ��
																	
	_matchedP3DList.clear();
	P3DMptPairList p3DMptMatchedList;
	//Ѱ�ҵ�ǰ֡�Ͳο�֡��ƥ���ͼ��
	if (!findMatchMptsByFrame(refFrame, currentFrame, p3DMptMatchedList, calcSim3Func, orientationFunc, p3dMathcedRatio))
		return false;


	///////////////////��ƥ���ͼ��ΪԼ������λ��///////////////////
	cv::Mat Rcw, tcw;
	P3DMptPairList outLiers;

	if (calcSim3Func(p3DMptMatchedList, outLiers, Rcw, tcw, orientationFunc, _system->isDispErrorStats()))
	{
		for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
			currentFrame.insertOutLier(itr->second);//��¼��ǰ֡��ɢƥ���ͼ��
		currentFrame.setPose(Rcw, tcw);
		currentFrame.calcMotionModel(refFrame);
		currentFrame.updateNormMatched();//���ݵ�ǰ֡ƥ���ϵ���µ�ͼ�㷨����
		if (currentFrame.isBad())
			return false;//���·��������ܵ��µ�ǰ֡��Ϊ��֡����ȷƥ��������٣�
		return true;
	}
	else
	{
		currentFrame.clearMatchedMpts();
		return false;
	}
	//currentFrame._outlierIndices.clear();//�Ż�ǰ��ɾ����ǰһ֡ƥ���Ż���ȷ������ɢ��ͼ������
	///////////////////////����3d-3dƥ��ԣ���С�����Ż�λ��/////////////////////
	//if (_system->_bOptimize3D)
	//{
	//	//if (Optimizer::optimizeRGBDSim3(&refFrame, &currentFrame,_matchedP3DList, velocity,10, true, _system->isDispErrorStats())<3)
	//	if (Optimizer::poseOptimization3D(&currentFrame,20,_system->isDispErrorStats())<3)
	//	{
	//		for (auto matchItr = currentFrame._matchedMptMap.begin(); matchItr != currentFrame._matchedMptMap.end(); matchItr++)
	//			matchItr->first->eraseObservation(&currentFrame);
	//		
	//		std::cout << "Failed optimizing frame pose!" << std::endl;
	//		return false;
	//	}
	//}	
	///////////////////////����3d-2dƥ��ԣ���С�����Ż�λ��/////////////////////
	//else
	//{
	//	if (0==Optimizer::poseOptimization2D(&currentFrame, 20, _system->isDispErrorStats())<3)
	//	{
	//		//currentFrame.eraseMptMatchedWithP3D();
	//		for (auto matchItr = currentFrame._matchedMptMap.begin(); matchItr != currentFrame._matchedMptMap.end(); matchItr++)
	//			matchItr->first->eraseObservation(&currentFrame);
	//	
	//		std::cout << "Failed optimizing frame pose!" << std::endl;
	//		return false;
	//	}
	//}
	//////////////////�޳���ǰ֡ƥ���ͼ���е���ɢ��/////////////////////////
	//currentFrame.eraseOutlierMapPoint();


}





bool Tracker::findMatchMptsByFrame(Frame& refFrame, Frame& curFrame, P3DMptPairList& p3DMptMatchedList,
								   Matcher_<cv::Point3_, MapPoint_, double>::CalcSim3Func calcSim3Func, Matcher_<>::OrientFunc orientationFunc,
								   const double p3dMathcedRatio = 0.)
{
	cv::Mat RSim3, tSim3;
	if (!Matcher_<>::matchP3DByFrame(refFrame, curFrame, 
									 _matchedP3DList, RSim3, tSim3, 
									 _system->isDispErrorStats(), orientationFunc, 
									 P3D_NEAR_MAX, p3dMathcedRatio))
	{
		fprintf(stderr, "Can't matched 3D points from frame: %d\n", refFrame.getID());
		return false;
	}

	if (!curFrame._matchedMptMap.empty())
		curFrame.clearMatchedMpts();//�����һ��֡-֡ƥ�����ʧ�ܵ�ƥ���ͼ��

	if (!curFrame.searchMptMatchedByFrame(refFrame, _matchedP3DList, p3DMptMatchedList))//��ǰ֡��ȡ��ǰһ֡��ƥ���ͼ��
		return false;

	cv::Mat Rcw, tcw;
	P3DMptPairList outLiers;
	if (calcSim3Func(p3DMptMatchedList, outLiers, Rcw, tcw, orientationFunc, _system->isDispErrorStats()))
		curFrame.setPose(Rcw, tcw);
	else
	{
		curFrame.clearMatchedMpts();
		return false;
	}

	int nMatched = checkUnMatchedKeys(curFrame, p3DMptMatchedList, Matcher_<cv::Point3_, MapPoint_, double>::matchedKeysFunc(_system->getMatchedKeysType()));
	if (nMatched>0)
		fprintf(stdout, "Found %d new matches in frame %d\n", nMatched, curFrame.getID());
	return true;
}





bool Tracker::trackMap(Frame& currentFrame, 
					   Matcher_<cv::Point3_, MapPoint_, double>::CalcSim3Func calcSim3Func, 
					   Matcher_<>::OrientFunc orientationFunc, 
					   const double p3dMathcedRatio)
{
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//����localMapping���߳�ɾ����ͼ��


	P3DMptPairList p3DMptMatchedList;
	if (!findMatchMptsByGlobalMap(_globalMap, currentFrame, p3DMptMatchedList, calcSim3Func, orientationFunc, p3dMathcedRatio))
	{
		return false;
	}

	cv::Mat Rcw, tcw;
	P3DMptPairList outLiers;
	if (calcSim3Func(p3DMptMatchedList, outLiers, Rcw, tcw, orientationFunc, _system->isDispErrorStats()))
	{
		for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
			currentFrame.insertOutLier(itr->second);//��¼��ǰ֡��ɢƥ���ͼ��
		currentFrame.setPose(Rcw, tcw);
		currentFrame.updateNormMatched();//���ݵ�ǰ֡ƥ���ϵ���µ�ͼ�㷨����
		if (currentFrame.isBad())
			return false;//���·��������ܵ��µ�ǰ֡��Ϊ��֡����ȷƥ��������٣�
		return true;
	}
	else
	{
		currentFrame.clearMatchedMpts();
		return false;
	}

	//lockEraseMpt.unlock();
	//if (!velocity.empty())
	//{
	//	currentFrame.setPose(velocity);
	//	//currentFrame.setPose(cv::Mat::eye(4, 4, CV_64F));
	//}
	//else
	//{
	//	currentFrame.setPose(cv::Mat::eye(4, 4, CV_64F));
	//}
	///////////////////////����3d-3dƥ�䣬��С�����Ż�λ��/////////////////////
	//if (_system->_bOptimize3D)
	//{
	//	if (Optimizer::poseOptimization3D(&currentFrame, 20, _system->isDispErrorStats())<3)//2
	//	{
	//		std::cout << "Failed optimizing frame pose!" << std::endl;
	//		return false;
	//	}
	//}
	///////////////////////����3d-2dƥ�䣬��С�����Ż�λ��/////////////////////
	//else
	//{
	//	if (Optimizer::poseOptimization2D(&currentFrame, 20, _system->isDispErrorStats())<3)//2
	//	{
	//		std::cout << "Failed optimizing frame pose!" << std::endl;
	//		return false;
	//	}
	//}

	return true;
}





bool Tracker::findMatchMptsByGlobalMap(GlobalMap* globalMap, Frame& curFrame, P3DMptPairList& p3DMptMatchedList, 
									   Matcher_<cv::Point3_, MapPoint_, double>::CalcSim3Func calcSim3Func, 
									   Matcher_<>::OrientFunc orientationFunc,const double p3dMathcedRatio)
{
	cv::Mat Rcw, tcw;
	if (!Matcher_<cv::Point3_,MapPoint_>::matchP3DByGlobalMap(globalMap, curFrame, 
															  p3DMptMatchedList, Rcw, tcw, 
															  orientationFunc,
															  _system->isDispErrorStats()))
	{
		return false;
	}

	curFrame.searchMptMatchedByGlobalMap(p3DMptMatchedList);//��ǰ֡��ȡ��ȫ�ֵ�ͼ��ƥ���ͼ��

	return true;
}





void Tracker::updateLocalMpts(MptList& localMptList)
{
	std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//����LocalMappingɾ֡

	const MptList& globalMptList = _globalMap->getMapPointList();
	KFSet localKFSet;
	const KeyFrameList& localKFList = _refKF->getCovisibleKFs();
	for (KeyFrameList::const_iterator itr = localKFList.begin(); itr != localKFList.end(); itr++)
		localKFSet.insert(*itr);
	KFSet neiborKFSet;
	for (KFSet::iterator lit = localKFSet.begin(), lend = localKFSet.end(); lit != lend; lit++)
	{
		MptKeyPairMap& matchedMptMap = (*lit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = matchedMptMap.begin(); mit != matchedMptMap.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			KFSet& observationKFs = pMP->getObservationKFs();//��ȡ�۲쵽�õ�ͼ��pMP�Ĺؼ�֡����

			for (KFSet::iterator kFItr = observationKFs.begin(); kFItr != observationKFs.end(); kFItr++)
			{
				if ((*kFItr)->getID() == (*lit)->getID())
					continue;
				neiborKFSet.insert((*kFItr));
			}
		}
	}
	localKFSet.insert(neiborKFSet.begin(), neiborKFSet.end());

	for (KFSet::iterator lit = localKFSet.begin(), lend = localKFSet.end(); lit != lend; lit++)
	{
		MptKeyPairMap& mapPointList = (*lit)->_matchedMptMap;
		for (MptKeyPairMap::iterator mit = mapPointList.begin(); mit != mapPointList.end(); mit++)
		{
			MapPoint3d* pMP = mit->first;
			if (pMP)
				if (!pMP->isBad())
					if (pMP->_localForKF != _refKF->getID())
					{
						localMptList.emplace_back(pMP);//localMapPoints���ֲ���ͼ�㼯�ϣ��ֲ��ؼ�֡�۲�����е�ͼ��
						pMP->_localForKF = _refKF->getID();
					}
		}
		
	}
		
}




int Tracker::checkUnMatchedKeys(Frame& curFrame, P3DMptPairList& p3DMptMatchedList,
							    Matcher_<cv::Point3_, MapPoint_, double>::MatchedKeysFunc func)
{
	int nMatched = 0;
	const MptList& globalMptList = _globalMap->getMapPointList();
	if (globalMptList.size() > LOCAL_MAX_MPTS)
	{
		MptList localMptList;
		updateLocalMpts(localMptList);
		nMatched = func(localMptList, curFrame, 
						p3DMptMatchedList,
						_system->getMatchedKeysType()== Matcher_<>::UNPROJ? MPT_MATCH_THRES: KEY_MATCH_THRES);
	}
	else
		nMatched = func(globalMptList, curFrame,
						p3DMptMatchedList,
						_system->getMatchedKeysType() == Matcher_<>::UNPROJ ? MPT_MATCH_THRES : KEY_MATCH_THRES);
	
	return nMatched;
}




int Tracker::expandGlobalMap(KeyFrame* const pKF)
{
	int nMpts = 0;	
	P3DKeyPairMap& point3DMap = pKF->_p3DMap;
	for (P3DKeyPairMap::iterator itr = point3DMap.begin(); itr != point3DMap.end(); itr++)
	{//������ǰ֡3D��	
		KeyPoint* keyPointLeft = itr->second.first;
		KeyPoint* keyPointRight = itr->second.second;
		cv::Point3d* pT3D = itr->first;

		if (keyPointLeft->_matchedMpt)
		{//����ƥ���ͼ���3d��
			continue;
		}		
		
		if (!isP3DValid(pT3D, point3DMap, P3D_NEAR_MIN, P3D_NEAR_MAX))
		{//����ƫ��ϴ���С��3d��
			continue;
		}
		////////�����µ�ͼ��//////////
		cv::Point3d worldPos = pKF->localToWorld(pT3D);
		MapPoint3d* mapPoint = new MapPoint3d(worldPos);
		if (!mapPoint)
			std::cout << "New map point create failed!" << std::endl;
	
		cv::Mat normVector = keyPointLeft->_bestNorm* keyPointLeft->_weight + 
			                 pKF->_camera->R().t()* keyPointRight->_bestNorm* keyPointRight->_weight;
		normVector = normVector / cv::norm(normVector);//��һ��
		normVector = pKF->_Rwc * normVector;
		mapPoint->setNormOri(normVector);
		mapPoint->setNormLast(normVector);

		mapPoint->setGlobalMap(_globalMap);

		//mapPoint->_coordLast = mapPoint->_coordOri = *dynamic_cast<cv::Point3d*>(mapPoint);
		pKF->addNewMapPoint(mapPoint);
		mapPoint->setRefKF(pKF);

		////////��ͼ����뵽��ǰ֡ƥ���ͼ�㼯��////////
		pKF->insertMatchedMPt(mapPoint, keyPointLeft, keyPointRight);
		mapPoint->addObservation(dynamic_cast<Frame*>(pKF));
		mapPoint->addKFObservation(pKF);
		
	
		////////��ͼ����뵽ȫ�ֵ�ͼ�㼯��//////
		_globalMap->addMapPoint(mapPoint);
		/*_globalMap->addMptDistList(mapPoint,Tracker::P3D_DIST_MAX);*/

		nMpts++;
	}
	
	return nMpts;
}




bool Tracker::isP3DValid(cv::Point3d* p3d, P3DKeyPairMap& p3dMap, double thMin, double thMax)
{
	double distP3d(0);
	double minDist(DBL_MAX);
	for (P3DKeyPairMap::iterator itr = p3dMap.begin(); itr != p3dMap.end(); itr++)
	{
		if (itr->first == p3d)
			continue;
		MapPoint3d* matchedMpt = itr->second.first->_matchedMpt;
		if (matchedMpt)
			if (matchedMpt->isBad())
				continue;//������ͼ���㣨localMapping���߳���trackRefFrame֮��ȷ����
		distP3d = cv::norm(*p3d - *itr->first);
		if (distP3d < minDist)
			minDist = distP3d;
	}
	if (minDist >thMax || minDist<thMin)
	{
		return false;
	}
	return true;
}




Tracker::RelocState Tracker::relocalizing(Frame& curFrame)
{
	std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//����LocalMappingɾ���ؼ�֡

	_relocRefKF = nullptr;
	std::vector<std::future<bool>> retList;
	for (auto kFItr = _globalMap->_keyFrameList.rbegin(); kFItr != _globalMap->_keyFrameList.rend(); kFItr++)
	{//����ȫ�ֹؼ�֡
		retList.emplace_back(_relocThreadPool.commit(relocByRefKF, std::ref(**kFItr), std::ref(curFrame), _matchedP3DList,
													 Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()), 
													 Matcher_<>::orientFunc(_system->getOrientType()), _system, _globalMap, 0));
		if (_relocRefKF)
			break;
	}
	while (!_relocRefKF)
	{
		auto itr = retList.begin();
		while (!retList.empty())
		{
			std::future<bool>& ret = *itr;
			if (ret._Is_ready())
			{
				if (ret.get() == false)
					itr = retList.erase(itr);
				else
					break;
				if (retList.empty())
				{
					return Reloc_Failed;
				}		
			}
			else
				itr++;
			if (itr == retList.end())
				itr = retList.begin();
			//std::this_thread::sleep_for(std::chrono::microseconds(10));
		}		
	}

	//�ض�λ�ɹ�
	_relocThreadPool.forceClear();
	_refFrame = dynamic_cast<Frame*>(_relocRefKF);//���¸��ٲο�֡
	_refKF = _relocRefKF;	//���¸��ٲο��ؼ�֡

	curFrame._isRelocByRefKF = true;

	return Reloc_Ref;


	//for (auto kFItr = _globalMap->_keyFrameList.rbegin(); kFItr != _globalMap->_keyFrameList.rend(); kFItr++)
	//{//����ȫ�ֹؼ�֡
	//	KeyFrame* pKF = *kFItr;
	//	//if (trackRefFrame(*refFrame, curFrame, Tracker::MATCHED_RATIO))
	//	if (trackRefFrame(*pKF, curFrame,
	//					  Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()), 
	//					  Matcher_<>::orientFunc(_system->getOrientType())))
	//	{
	//		_refFrame = dynamic_cast<Frame*>(pKF);//���¸��ٲο�֡
	//		_refKF = pKF;	//���¸��ٲο��ؼ�֡
	//		curFrame._isRelocByRefKF = true;
	//		return Reloc_Ref;
	//	}
	//}
	//if (trackMap(curFrame))
	//{
	//	curFrame.updateConnections();//������ǰ��֡�����ϵ
	//	FrameList& covisibles = curFrame.getBestCovisibles(1);
	//	if (!covisibles.empty())
	//	{
	//		Frame* maxCovisibleFrame = covisibles.front();
	//		if (curFrame.getCovisibleCount(maxCovisibleFrame) > 3)
	//		{
	//			_refFrame = maxCovisibleFrame;//���¸��ٲο�֡
	//			_refKF= maxCovisibleFrame->_refKF;//���¸��ٲο��ؼ�֡
	//			curFrame._isRelocByMap = true;
	//			return Reloc_Map;
	//		}		
	//	}	
	//}

}

bool Tracker::relocByRefKF(KeyFrame& refKF, Frame& curFrame, P3DPairList& matchedP3DList,
						   Matcher_<cv::Point3_, MapPoint_, double>::CalcSim3Func calcSim3Func,
						   Matcher_<>::OrientFunc orientationFunc,
						   System* system, GlobalMap* globalMap, const double p3dMathcedRatio)
{
	if (refKF.isBad())
		return false;
	if (_relocRefKF)
		return false;//�������̸߳��ٳɹ�,����
	
	P3DPairList	matchedP3DListTmp;
	P3DMptPairList p3DMptMatchedList;
	//Ѱ�ҵ�ǰ֡�Ͳο�֡��ƥ���ͼ��
	cv::Mat RSim3, tSim3;

	if (!Matcher_<>::matchP3DByFrame(refKF, curFrame,
									 matchedP3DListTmp, RSim3, tSim3,
									 system->isDispErrorStats(), orientationFunc, 
		                             P3D_NEAR_MAX, p3dMathcedRatio))
	{
		//fprintf(stderr, "Can't matched 3D points from keyFrame: %d\n",dynamic_cast<Frame&>(refKF).getID());
		return false;
	}

	//std::unique_lock<std::mutex> lockTrack(_trackMutex);//�ض�λ���̸߳�����(ƥ��ɹ����λ�˽���Ƕ��߳�)
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//��סlocalMapping�߳�ɾ����ͼ��,��ס���߳��ض�λ
		if (_relocRefKF)
			return false;//�������̸߳��ٳɹ�,����
			
	if (!curFrame._matchedMptMap.empty())
		curFrame.clearMatchedMpts();//�����һ��֡-֡ƥ�����ʧ�ܵ�ƥ���ͼ��

	if (!curFrame.searchMptMatchedByFrame(refKF, matchedP3DListTmp, p3DMptMatchedList))//��ǰ֡��ȡ��ǰһ֡��ƥ���ͼ��
		return false;
	matchedP3DList = matchedP3DListTmp;

	cv::Mat Rcw, tcw;
	P3DMptPairList outLiers;
	if (calcSim3Func(p3DMptMatchedList, outLiers, Rcw, tcw, orientationFunc, system->isDispErrorStats()))
		curFrame.setPose(Rcw, tcw);
	else
	{
		curFrame.clearMatchedMpts();
		return false;
	}

	const MptList& globalMptList = globalMap->getMapPointList();

	int nMatched = Matcher_<cv::Point3_, MapPoint_, double>::findMatchKeysByUnProj(globalMptList, curFrame,
																				   p3DMptMatchedList, MPT_MATCH_THRES);
	if (nMatched>0)
		fprintf(stdout, "Found %d new matches in frame %d\n", nMatched, curFrame.getID());

	///////////////////��ƥ���ͼ��ΪԼ������λ��///////////////////
	outLiers.clear();

	if (calcSim3Func(p3DMptMatchedList, outLiers, Rcw, tcw, orientationFunc, system->isDispErrorStats()))
	{
		for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
			curFrame.insertOutLier(itr->second);//��¼��ǰ֡��ɢƥ���ͼ��
		curFrame.setPose(Rcw, tcw);
		curFrame.calcMotionModel(refKF);
		curFrame.updateNormMatched();//���ݵ�ǰ֡ƥ���ϵ���µ�ͼ�㷨����
		if (curFrame.isBad())
			return false;//���·��������ܵ��µ�ǰ֡��Ϊ��֡����ȷƥ��������٣�
		_relocRefKF = &refKF;
		return true;
	}
	else
	{
		curFrame.clearMatchedMpts();
		return false;
	}
}



int Tracker::checkMapUntilFrame(const Frame& currentFrame,const unsigned& FrameThres, const unsigned& obsThres)
{
	int nErased(0);
	Frame* refFrame=nullptr;
	std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);
	if (_globalMap->getFrameList().size() > FrameThres)
	{
		auto itr = _globalMap->getFrameList().end()-1- FrameThres;
		if (itr != _globalMap->getFrameList().end())
		{
			refFrame = *itr;
		}
	}	
	lockFrame.unlock();
	if (!refFrame)
		return -1;	
	KeyFrame* refKF = dynamic_cast<KeyFrame*>(refFrame);
	if (refKF)
	{
		if (refKF->_newMptList.empty())
			return -1;
	}	
	else
		return -1;

	if (FrameThres < obsThres)
		return -1;

	MptList::iterator g_mItr, g_mItrNext, g_mItrPrev;
	MptList refNewMapPoints = refKF->_newMptList;//�����ã�����eraseMapPoint��ɾ��refFrame->_newMptList��Ԫ��
	for (MptList::iterator mItr = refNewMapPoints.begin(); mItr != refNewMapPoints.end(); )
	{
		if ((*mItr)->isBad())
		{
			mItr++;
			continue;
		}
			
		if ((*mItr)->obs() < obsThres)
		{
			int maxInterval = (*mItr)->obs() < FrameThres ? (*mItr)->obs() : FrameThres;

			g_mItr = std::find(_globalMap->_mapPointList.begin(), _globalMap->_mapPointList.end(), *mItr);
			if (g_mItr == _globalMap->_mapPointList.end())
				return -1;

			g_mItrNext = g_mItr + 1;
			while (g_mItrNext != _globalMap->_mapPointList.end())
			{
				if (!(*g_mItrNext)->isBad())
					break;
				g_mItrNext++;
			}
			if (g_mItrNext != _globalMap->_mapPointList.end())
				if ((*g_mItrNext)->obs() - (*g_mItr)->obs() > maxInterval)
				{
					//_globalMap->eraseMapPoint(*g_mItr);
					(*g_mItr)->setBad(true);
					_globalMap->insertBadMpt((*g_mItr));
					nErased++;
					mItr = refNewMapPoints.erase(mItr);
					continue;
				}

			if (g_mItr == _globalMap->_mapPointList.begin())
			{
				mItr++;
				continue;
			}				
			g_mItrPrev = g_mItr - 1;	
			while (g_mItrPrev != _globalMap->_mapPointList.end())
			{
				if (!(*g_mItrPrev)->isBad())
					break;
				g_mItrPrev--;
			}
			if(g_mItrPrev != _globalMap->_mapPointList.end())
				if (((*g_mItrPrev)->obs() - (*g_mItr)->obs() > maxInterval))
				{
					//_globalMap->eraseMapPoint(*g_mItr);
					(*g_mItr)->setBad(true);
					_globalMap->insertBadMpt((*g_mItr));
					nErased++;
					mItr = refNewMapPoints.erase(mItr);
					continue;
				}		

		}
		mItr++;

	}
	return nErased;
}




void Tracker::mapPointCulling(const Frame& Frame)
{																			
	for (int i = 0; i < 3; i++)
	{
		if (int nErase = checkMapUntilFrame(Frame, (i + 1)*Tracker::FRAME_THRES, (i + 1)*Tracker::OBSERVE_THRES))
		{
			if (nErase != -1)
			{
				fprintf(stdout, "Delete %d bad map point\n", nErase);
			}
		}
	}
}



bool Tracker::checkRt(cv::Mat& Rcw, cv::Mat& tcw, Frame* currentFrame, Frame* prevFrame, GlobalMap* globalMap)
{
	cv::Mat relativeMotionPrev = cv::Mat::eye(4, 4, CV_64F);
	Rcw.copyTo(relativeMotionPrev.colRange(0, 3).rowRange(0, 3));
	tcw.copyTo(relativeMotionPrev.col(3).rowRange(0, 3));

	Frame* lastFrame = *(globalMap->getFrameList().end() - 1);

	cv::Mat relativeMotionLast = relativeMotionPrev*prevFrame->getPose()*lastFrame->getPoseInverse();
	if (relativeMotionLast.empty())
	{
		std::cout << "Calculating relative motion from last Frame failed!" << std::endl;
		return false;
	}

	cv::Mat relativeRcw;
	cv::Mat relativetcw;
	cv::Mat Rwc; // ���λ��������һ֡��ת����
	cv::Mat Ow;//���λ��������һ֡ƫ��

	relativeMotionLast.colRange(0, 3).rowRange(0, 3).copyTo(relativeRcw);
	relativeMotionLast.col(3).rowRange(0, 3).copyTo(relativetcw);
	Rwc = relativeRcw.t();
	Ow = -Rwc*relativetcw;

	Eigen::Matrix3d rotationMatrix = Converter::toMatrix3d(Rwc);
	Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0).transpose();
	eulerAngles = eulerAngles / PI * 180;

	double shift = cv::norm(Ow);

	int frameInterval = currentFrame->getID() - lastFrame->getID();
	if (frameInterval == 1)
	{
		if (shift > TRANSPOSE_THRES)
			return false;
		for (int i = 0; i < eulerAngles.size(); i++)
		{
			if (eulerAngles[i] < 0) eulerAngles[i] = -eulerAngles[i];
			double rotation = 180 - eulerAngles[i]>90 ? eulerAngles[i] : 180 - eulerAngles[i];
			if (rotation> ROTATION_THRES)
			{
				return false;
			}
		}
	}
	else if (1<frameInterval&&frameInterval<5)
	{
		double maxShift = frameInterval * TRANSPOSE_THRES;
		if (shift > maxShift)
			return false;
	}

	return true;
}


}