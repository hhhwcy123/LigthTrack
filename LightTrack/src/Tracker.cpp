#include "Tracker.h"
#include "Optimizer.h"
#include "System.h"
#include "Converter.h"
#include "impl\Matcher.hpp"

#include <Eigen\Core>

namespace SLAM
{

//地图坏点删除
const int Tracker::FRAME_THRES = 40;
const int Tracker::OBSERVE_THRES = 10;//20;//错点最大观察帧数

//关键帧间隔帧数
#ifdef STAGE 
const int Tracker::KEYFRAME_INTERVAL = 1;
#else
const int Tracker::KEYFRAME_INTERVAL = 10;
#endif

//局部最大地图点数量
const int Tracker::LOCAL_MAX_MPTS = 500;

//单帧相邻3d点最大最小距离
#ifdef STAGE 
const double Tracker::P3D_NEAR_MAX = 300;//350
#else
const double Tracker::P3D_NEAR_MAX = 150;//300;//350
#endif
const double Tracker::P3D_NEAR_MIN = 10;

//单帧3d点最大距离
#ifdef STAGE 
const double Tracker::P3D_DIST_MAX = 1000;//350
#else
const double Tracker::P3D_DIST_MAX = 250;//350
#endif

//地图点-2d重投影匹配
const double Tracker::KEY_MATCH_THRES = 5;

//3d-地图点反投影匹配
const double Tracker::MPT_MATCH_THRES = 3.;//5//10;//20

//相邻帧-帧R，t检测
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
	{//绘制当前帧与前帧的匹配地图点连线（地图点对应的2D点）（绿色）
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
	
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//避免localMapping子线程删除地图点
	
	for (MptKeyPairMap::iterator itr = frameCur._matchedMptMap.begin(); itr != frameCur._matchedMptMap.end(); itr++)
	{//绘制当前帧所有匹配的地图点，包括与全局地图匹配得到的（地图点对应的2D点）（精确：绿色，离散：黄色）
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
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//避免localMapping子线程删除地图点

	if (_canvasImg.empty())
		return false;

	int wl = cvRound(frameCur._markImgLeft.cols);

	KeyFrame* pKFCur = dynamic_cast<KeyFrame*>(&frameCur);
	//绘制当前帧新建地图点（红色）
	if (pKFCur)
	{
		MptList& curNewMptList = pKFCur->_newMptList;
		for (MptList::iterator itr = curNewMptList.begin(); itr != curNewMptList.end(); itr++)
		{//遍历frameCur的新建地图点集合
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
	{//绘制前帧新建地图点（红色）
		MptList& prevNewMptList = pKFPrev->_newMptList;
		for (MptList::iterator itr = prevNewMptList.begin(); itr != prevNewMptList.end(); itr++)
		{//遍历frameCur的新建地图点集合
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
		{//加载地图文件续扫
			if (!trackMap(curFrame, 
						  Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()), 
						  Matcher_<>::orientFunc(_system->getOrientType())))
			{
				fprintf(stdout, "Frame %d tracking map failed!\n", curFrame.getID());
				return false;
			}	
		}
		else
			curFrame.setPose(cv::Mat::eye(4, 4, CV_64F));//设置初始frame位姿		
		
		 KeyFrame* newKeyFrame = createKeyFrame(curFrame);

		////////更新跟踪参考////////
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
	std::unique_lock<std::mutex> lockTrack(_trackMutex);//支持多线程预跟踪

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
			//////////////////重定位/////////////////
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
	{//初始化	
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

		std::unique_lock<std::mutex> lockTrack(_trackMutex);//当前跟踪帧与子线程局部BA帧修正同步

		bool bTrackRef = false;
		bool bTrackRefKF = true;
		////////////////////帧-帧3d点匹配跟踪////////////////////
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
				//////////////////重定位/////////////////
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
		
		////////////////创建前后帧跟踪图像并绘制当前帧匹配地图点///////////////
		if (_system->isOutputTrackImg())
		{
			Frame* refFrame = bTrackRef ? _refFrame : dynamic_cast<Frame*>(_refKF);
			createTrackImg(curFrame, *refFrame);
			writeTrackMatchedImg(curFrame, *refFrame, _matchedP3DList);
		}		
		

		////////////////////建立关键帧///////////////
		KeyFrame* newKeyFrame = nullptr;
		if (!bTrackRefKF)
		{//根据_refFrame建立关键帧
			newKeyFrame = createKeyFrame(*_refFrame);
			if (!newKeyFrame)
				return false;
			//前帧创建为关键帧后重新跟踪当前帧(前帧可能创建新地图点)
		/*	if (!trackRefFrame(*newKeyFrame, curFrame))
				return false;*/
		}
		else if (needNewKeyFrame(curFrame))
		{//根据curFrame建立关键帧			
			newKeyFrame = createKeyFrame(curFrame);
			if (!newKeyFrame)
				return false;
		}	
		
		////////////////////更新跟踪参考////////////////////////
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

		lockTrack.unlock();//curFrame加入newKeyFrame的邻接帧后，localMapping的局部BA才能矫正关键帧的邻接帧
		
		 //////////////检查异常地图点//////////////////
		mapPointCulling(curFrame);
	
		_globalMap->setReferenceMapPoints(_globalMap->getMapPointList());//设置下一次跟踪的参考地图点集合

		_mapDrawer->setCurrentCameraPose(curFrame.getPose());//设置场景观察相机位姿
	
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
	//若局部BA被回环终止，则不创建关键帧
	if (_localMapper->isStopped() || _localMapper->stopRequested())
		return false;
	//第一帧创建
	if (_globalMap->getFrameList().empty())
		return true;

	bool flag = false;

	KeyFrame* lastkeyFrame = _globalMap->getKeyFrameList().back();
	Frame* lastFrame = dynamic_cast<Frame*>(const_cast<KeyFrame*> (lastkeyFrame));

	if (!flag)
	{//与上一关键帧的帧数间隔	
		if(curFrame.getID() - lastFrame->getID() >= Tracker::KEYFRAME_INTERVAL || curFrame._isRelocByMap)
			flag = true;
	}
	if(!flag)
	{
		const int& p3DSize = curFrame._p3DMap.size();//3d点总数
		const int& matchedSize = curFrame._matchedMptMap.size();//匹配的3d点数量（包括离散匹配）

		if (p3DSize - matchedSize >1)
		{//存在至少2个未匹配的3d点，决定跟踪连续性
			flag = true;
		}
	}
	if (!flag)
	{
		std::unique_lock<std::mutex> lock(Frame::_matchedMutex);//refFrame的_matchedMptMap可能在局部BA子线程删除

		unsigned int nCovisMpts = 0;
		std::set<MapPoint3d*> refMptMatchedSet, curMptMathcedSet;

		for (auto itr = curFrame._matchedMptMap.begin(); itr != curFrame._matchedMptMap.end(); itr++)
		{
			if (_refKF->_matchedMptMap.count(itr->first));
				nCovisMpts++;
		}
		double matchedRatio = (double)nCovisMpts / (double)_refKF->_matchedMptMap.size();//与上一关键帧匹配共同地图点的比例
		if (matchedRatio<0.4)
		{//与上一关键帧共视点比例过小，决定关键帧间共点程度
			flag = true;
		}
	}
	if (flag)
	{
		if (!_localMapper->isAcceptKeyFrames())
		{//子线程是否接受关键帧
			if (_localMapper->keyframesInQueue() < 3)
			{//若子线程处理队列包含3个以上关键帧未处理则不创建新的关键帧

				_localMapper->abortBA();//如果满足关键帧建立条件而子线程正在局部BA,子线程局部BA若未完成迭代后位姿修正，将跳出
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
	//frame.updateConnections();//计算当前关键帧和前帧的共点关系，更新共点关键帧集合
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//避免地图点被删除导致遍历pKF匹配地图点时出错（需放在由frame生成pKF之前）

	KeyFrame* pKF = new KeyFrame(frame);
	if (!pKF)
		return nullptr;

	/////////////////删除并替换原frame对象与其它对象的关系//////////////////////
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
	//更新关键帧前后向邻接关系	
	pKF->_parentKF = _refKF;
	if (_refKF)
		_refKF->_childrenKFs.insert(pKF);

	/////////////////扩充全局地图//////////////////
	int newMptCount = 0;
	if (newMptCount = expandGlobalMap(pKF))
	{
		fprintf(stdout, "Frame %d created %d new map points\n", frame.getID(), newMptCount);
		if (_system->isOutputTrackImg())
		{//绘制当前帧创建的新地图点并保存图像
			if(&frame==_refFrame)
				writeTrackNewMptImg(frame, *frame._refKF);
			else
				writeTrackNewMptImg(frame, *_refFrame);
		}
	}


	_localMapper->addKeyFrame(pKF);//保证更新完关键帧相关信息再入队
	_globalMap->addKeyFrame(pKF);//入队后加入全局地图（保证BA优化前队列最后关键帧也是全局地图的最后关键帧）


	return pKF;
}




bool Tracker::trackRefFrame(Frame& refFrame, Frame& currentFrame, 
						    Matcher_<cv::Point3_, MapPoint_, double>::CalcSim3Func calcSim3Func,
						    Matcher_<>::OrientFunc orientationFunc,
						    const double p3dMathcedRatio)
{
	if (refFrame.isBad())
		return false;
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//避免localMapping子线程删除地图点
																	
	_matchedP3DList.clear();
	P3DMptPairList p3DMptMatchedList;
	//寻找当前帧和参考帧的匹配地图点
	if (!findMatchMptsByFrame(refFrame, currentFrame, p3DMptMatchedList, calcSim3Func, orientationFunc, p3dMathcedRatio))
		return false;


	///////////////////以匹配地图点为约束计算位姿///////////////////
	cv::Mat Rcw, tcw;
	P3DMptPairList outLiers;

	if (calcSim3Func(p3DMptMatchedList, outLiers, Rcw, tcw, orientationFunc, _system->isDispErrorStats()))
	{
		for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
			currentFrame.insertOutLier(itr->second);//记录当前帧离散匹配地图点
		currentFrame.setPose(Rcw, tcw);
		currentFrame.calcMotionModel(refFrame);
		currentFrame.updateNormMatched();//根据当前帧匹配关系更新地图点法向量
		if (currentFrame.isBad())
			return false;//更新法向量可能导致当前帧变为坏帧（精确匹配点数过少）
		return true;
	}
	else
	{
		currentFrame.clearMatchedMpts();
		return false;
	}
	//currentFrame._outlierIndices.clear();//优化前先删除和前一帧匹配优化后确定的离散地图点索引
	///////////////////////利用3d-3d匹配对，最小二乘优化位姿/////////////////////
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
	///////////////////////利用3d-2d匹配对，最小二乘优化位姿/////////////////////
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
	//////////////////剔除当前帧匹配地图点中的离散点/////////////////////////
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
		curFrame.clearMatchedMpts();//清空上一次帧-帧匹配跟踪失败的匹配地图点

	if (!curFrame.searchMptMatchedByFrame(refFrame, _matchedP3DList, p3DMptMatchedList))//当前帧获取与前一帧的匹配地图点
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
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//避免localMapping子线程删除地图点


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
			currentFrame.insertOutLier(itr->second);//记录当前帧离散匹配地图点
		currentFrame.setPose(Rcw, tcw);
		currentFrame.updateNormMatched();//根据当前帧匹配关系更新地图点法向量
		if (currentFrame.isBad())
			return false;//更新法向量可能导致当前帧变为坏帧（精确匹配点数过少）
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
	///////////////////////利用3d-3d匹配，最小二乘优化位姿/////////////////////
	//if (_system->_bOptimize3D)
	//{
	//	if (Optimizer::poseOptimization3D(&currentFrame, 20, _system->isDispErrorStats())<3)//2
	//	{
	//		std::cout << "Failed optimizing frame pose!" << std::endl;
	//		return false;
	//	}
	//}
	///////////////////////利用3d-2d匹配，最小二乘优化位姿/////////////////////
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

	curFrame.searchMptMatchedByGlobalMap(p3DMptMatchedList);//当前帧获取与全局地图的匹配地图点

	return true;
}





void Tracker::updateLocalMpts(MptList& localMptList)
{
	std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//避免LocalMapping删帧

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
			KFSet& observationKFs = pMP->getObservationKFs();//获取观察到该地图点pMP的关键帧集合

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
						localMptList.emplace_back(pMP);//localMapPoints：局部地图点集合，局部关键帧观察的所有地图点
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
	{//遍历当前帧3D点	
		KeyPoint* keyPointLeft = itr->second.first;
		KeyPoint* keyPointRight = itr->second.second;
		cv::Point3d* pT3D = itr->first;

		if (keyPointLeft->_matchedMpt)
		{//跳过匹配地图点的3d点
			continue;
		}		
		
		if (!isP3DValid(pT3D, point3DMap, P3D_NEAR_MIN, P3D_NEAR_MAX))
		{//跳过偏差较大或较小的3d点
			continue;
		}
		////////创建新地图点//////////
		cv::Point3d worldPos = pKF->localToWorld(pT3D);
		MapPoint3d* mapPoint = new MapPoint3d(worldPos);
		if (!mapPoint)
			std::cout << "New map point create failed!" << std::endl;
	
		cv::Mat normVector = keyPointLeft->_bestNorm* keyPointLeft->_weight + 
			                 pKF->_camera->R().t()* keyPointRight->_bestNorm* keyPointRight->_weight;
		normVector = normVector / cv::norm(normVector);//归一化
		normVector = pKF->_Rwc * normVector;
		mapPoint->setNormOri(normVector);
		mapPoint->setNormLast(normVector);

		mapPoint->setGlobalMap(_globalMap);

		//mapPoint->_coordLast = mapPoint->_coordOri = *dynamic_cast<cv::Point3d*>(mapPoint);
		pKF->addNewMapPoint(mapPoint);
		mapPoint->setRefKF(pKF);

		////////地图点加入到当前帧匹配地图点集合////////
		pKF->insertMatchedMPt(mapPoint, keyPointLeft, keyPointRight);
		mapPoint->addObservation(dynamic_cast<Frame*>(pKF));
		mapPoint->addKFObservation(pKF);
		
	
		////////地图点加入到全局地图点集合//////
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
				continue;//跳过地图坏点（localMapping子线程在trackRefFrame之后确定）
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
	std::unique_lock<std::mutex> lockFrame(GlobalMap::_frameMutex);//避免LocalMapping删除关键帧

	_relocRefKF = nullptr;
	std::vector<std::future<bool>> retList;
	for (auto kFItr = _globalMap->_keyFrameList.rbegin(); kFItr != _globalMap->_keyFrameList.rend(); kFItr++)
	{//反溯全局关键帧
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

	//重定位成功
	_relocThreadPool.forceClear();
	_refFrame = dynamic_cast<Frame*>(_relocRefKF);//更新跟踪参考帧
	_refKF = _relocRefKF;	//更新跟踪参考关键帧

	curFrame._isRelocByRefKF = true;

	return Reloc_Ref;


	//for (auto kFItr = _globalMap->_keyFrameList.rbegin(); kFItr != _globalMap->_keyFrameList.rend(); kFItr++)
	//{//反溯全局关键帧
	//	KeyFrame* pKF = *kFItr;
	//	//if (trackRefFrame(*refFrame, curFrame, Tracker::MATCHED_RATIO))
	//	if (trackRefFrame(*pKF, curFrame,
	//					  Matcher_<cv::Point3_, MapPoint_, double>::calcSim3Func(_system->getCalcSim3Type()), 
	//					  Matcher_<>::orientFunc(_system->getOrientType())))
	//	{
	//		_refFrame = dynamic_cast<Frame*>(pKF);//更新跟踪参考帧
	//		_refKF = pKF;	//更新跟踪参考关键帧
	//		curFrame._isRelocByRefKF = true;
	//		return Reloc_Ref;
	//	}
	//}
	//if (trackMap(curFrame))
	//{
	//	curFrame.updateConnections();//更新与前向帧共点关系
	//	FrameList& covisibles = curFrame.getBestCovisibles(1);
	//	if (!covisibles.empty())
	//	{
	//		Frame* maxCovisibleFrame = covisibles.front();
	//		if (curFrame.getCovisibleCount(maxCovisibleFrame) > 3)
	//		{
	//			_refFrame = maxCovisibleFrame;//更新跟踪参考帧
	//			_refKF= maxCovisibleFrame->_refKF;//更新跟踪参考关键帧
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
		return false;//若其它线程跟踪成功,返回
	
	P3DPairList	matchedP3DListTmp;
	P3DMptPairList p3DMptMatchedList;
	//寻找当前帧和参考帧的匹配地图点
	cv::Mat RSim3, tSim3;

	if (!Matcher_<>::matchP3DByFrame(refKF, curFrame,
									 matchedP3DListTmp, RSim3, tSim3,
									 system->isDispErrorStats(), orientationFunc, 
		                             P3D_NEAR_MAX, p3dMathcedRatio))
	{
		//fprintf(stderr, "Can't matched 3D points from keyFrame: %d\n",dynamic_cast<Frame&>(refKF).getID());
		return false;
	}

	//std::unique_lock<std::mutex> lockTrack(_trackMutex);//重定位多线程跟踪锁(匹配成功后的位姿解算非多线程)
	std::unique_lock<std::mutex> lockEraseMpt(GlobalMap::_mptMutex);//锁住localMapping线程删除地图点,锁住多线程重定位
		if (_relocRefKF)
			return false;//若其它线程跟踪成功,返回
			
	if (!curFrame._matchedMptMap.empty())
		curFrame.clearMatchedMpts();//清空上一次帧-帧匹配跟踪失败的匹配地图点

	if (!curFrame.searchMptMatchedByFrame(refKF, matchedP3DListTmp, p3DMptMatchedList))//当前帧获取与前一帧的匹配地图点
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

	///////////////////以匹配地图点为约束计算位姿///////////////////
	outLiers.clear();

	if (calcSim3Func(p3DMptMatchedList, outLiers, Rcw, tcw, orientationFunc, system->isDispErrorStats()))
	{
		for (auto itr = outLiers.begin(); itr != outLiers.end(); itr++)
			curFrame.insertOutLier(itr->second);//记录当前帧离散匹配地图点
		curFrame.setPose(Rcw, tcw);
		curFrame.calcMotionModel(refKF);
		curFrame.updateNormMatched();//根据当前帧匹配关系更新地图点法向量
		if (curFrame.isBad())
			return false;//更新法向量可能导致当前帧变为坏帧（精确匹配点数过少）
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
	MptList refNewMapPoints = refKF->_newMptList;//非引用，后续eraseMapPoint会删除refFrame->_newMptList的元素
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
	cv::Mat Rwc; // 相机位姿相对最后一帧旋转矩阵
	cv::Mat Ow;//相机位姿相对最后一帧偏移

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