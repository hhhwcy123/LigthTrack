#include "GlobalMap.h"
#include "Camera.h"
#include "System.h"
#include "ReadWriter.h"
#include "Matcher.h"
#include "Cuda\Cuda_Wrapper.h"

#include "opencv2\cudaimgproc.hpp"

namespace SLAM
{

const double Frame::MaxEpilineDist = 1.0;
const double Frame::MinEnergyValue = 0.1; //根据实际情况调整
const double Frame::MarkerAngleThres = 25;//15.;
const double Frame::MarkerCenterThres = 80;// 30.;

std::mutex Frame::_matchedMutex;

const Frame::CalcMatchedFunc Frame::_calcMatchedFuncs[] = { calcStereoMatchesByNorms,
															calcStereoMatchesByEpipolar,
															calcStereoMatchesByEnergy };

Frame::~Frame()
{
	if (!_keyPointsLeft.empty())
	{
		for (KeyPointList::iterator itr = _keyPointsLeft.begin(); itr != _keyPointsLeft.end();)
		{
			delete *itr;
			*itr = nullptr;
			itr = _keyPointsLeft.erase(itr);
		}
	}
	if (!_keyPointsRight.empty())
	{
		for (KeyPointList::iterator itr = _keyPointsRight.begin(); itr != _keyPointsRight.end();)
		{
			delete *itr;
			*itr = nullptr;
			itr = _keyPointsRight.erase(itr);
		}
	}
	if (!_p3DMap.empty())
	{
		for (P3DKeyPairMap::iterator itr = _p3DMap.begin(); itr != _p3DMap.end();)
		{
			cv::Point3d* p3dPtr = itr->first;
			delete p3dPtr;
			p3dPtr = nullptr;
			itr = _p3DMap.erase(itr);
		}
	}
}



bool Frame::calcStereoMatchesByEnergy(Frame* pFrame, Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight, Feature::MarkerMultiMap& markerMathcedMap)
{
#ifndef STAGE 
	if (markListLeft.size() < 4 || markListRight.size() < 4)
		return false;
#endif

	Feature::MarkerList markList1, markList2;
	int cameraId;
	if (markListLeft.size() < markListRight.size())
	{
		markList1 = markListLeft;
		markList2 = markListRight;
		cameraId = 1;

	}
	else
	{
		markList1 = markListRight;
		markList2 = markListLeft;
		cameraId = 2;
	}
	//Feature::MarkerMap markMathcedMapTmp;
	Feature::MarkerMultiMap markMathcedMapTmp;
	std::multimap<Feature::DotMarker, double> MatchedMarkonly; //用于去除重复标志点

	const double& width = pFrame->maxX();
	const double& height = pFrame->maxY();
	double L = sqrt(width *width + height*height);
	double radius = 0.25*L;   //可以根据实际情况修改，范围：[0-L]，保证邻域点数量至少为4个

	for (Feature::MarkerList::iterator itr1 = markList1.begin(); itr1 != markList1.end(); itr1++)
	{//标志点数量markList2>markList1，以较少的标志点集合为基准遍历
		Feature::DotMarker& marker1 = *itr1;
		
#if CV_MAJOR_VERSION < 3
		std::vector<cv::Point2f> points;
		points.emplace_back(cv::Point2d(marker1.undistortX, marker1.undistortY));
		std::vector<cv::Vec<float, 3>> lines;
		cv::computeCorrespondEpilines(points, cameraId, pFrame->getCamera()->F(), lines);
		const double& a = lines.back()[0];
		const double& b = lines.back()[1];
		const double& c = lines.back()[2];
#else
		cv::Mat x2D1 = (cv::Mat_<double>(2, 1) << marker1.undistortX, marker1.undistortY);
		cv::Mat x2D1_h = (cv::Mat_<double>(3, 1) << marker1.undistortX, marker1.undistortY, 1);
		cv::Mat line;
		cv::computeCorrespondEpilines(x2D1.t(), cameraId, pFrame->_camera->F(), line);
		line = line.reshape(1, 3);
		const double& a = line.at<double>(0);
		const double& b = line.at<double>(1);
		const double& c = line.at<double>(2);
#endif	

		Feature::DotMarker bestMark2, minDistMark;
		double minDist = DBL_MAX;
		double maxEnergy = DBL_MIN;
		bool bMathced = false;

		Feature::MarkerList::iterator bestItr = markList2.end();
		double similar = .0;
		double bestSimilar = DBL_MAX;
		double SamDist;

		std::vector<std::pair<Feature::DotMarker, double>> candidateMarks;  //储存能量大于阈值的候选点
		double energy = 0.0;
		double dist = 0.;
		for (Feature::MarkerList::iterator itr2 = markList2.begin(); itr2 != markList2.end(); itr2++)
		{
			Feature::DotMarker& marker2 = *itr2;

			dist = abs(a*marker2.undistortX + b*marker2.undistortY + c) / sqrt(a*a + b*b);	
			cv::Mat x2D2_h = (cv::Mat_<double>(3, 1) << marker2.undistortX, marker2.undistortY, 1);
			if (dist < MaxEpilineDist)//阈值需根据标定精准度调整，标定过差会损失标志点
			{
				//********* 1 使用形状相似性做为判别标准*********//
				//double araeSizeDiff = abs(marker2.areaSize - marker1.areaSize);
				//double eccentricityDiff = abs(marker2.eccentricity - marker1.eccentricity);
				//similar = 0.5*araeSizeDiff + 0.5*eccentricityDiff;
				//if (similar < bestSimilar)
				//{
				//	bestSimilar = similar;
				//	bestItr = itr2;
				//	bestMark2 = marker2;
				//	bMathced = true;
				//}

				//********* 2 使用点到核线的最短距离做为判别标准******//
				/*	if (dist < minDist)
				{
				minDist = dist;
				bestMark2 = marker2;
				bestItr = itr2;
				bMathced = true;
				}*/
				if (dist < minDist)
				{
					minDist = dist;
					minDistMark = marker2;
				}

				//*********** 3 使用sampsonDistance距离做为判别标准******//
				//单纯考虑了图像点到极线的几何距离，对于三维空间点信息并未充分利用
				//这里采用更加准确的几何距离计算三维点在图像内的投影，为Sampson距离
				//在现有数据上实验后，发现此距离与之前的距离相比并没有提升，故先不采用，节省计算量
				/*	if (cameraId == 1)
				{
				SamDist = sampsonDistance(x2D1_h, x2D2_h, F);
				}
				else
				{
				SamDist = sampsonDistance(x2D2_h, x2D1_h, F);
				}*/
				//std::cout << "SamDist: " << SamDist << std::endl;
				/*if (SamDist < minDist)
				{
				minDist = dist;
				bestMark2 = marker2;
				bestItr = itr2;
				bMathced = true;
				}*/

				//********** 4 基于特征点能量的立体匹配稳健算法*************//

				energy = calcEnergyValue(marker1, marker2, markList1, markList2, pFrame->_camera->F(), radius, cameraId);
				std::cout << "energy: " << energy << std::endl;

				if (energy > MinEnergyValue)  //能量阈值
				{
					candidateMarks.emplace_back(std::make_pair(marker2, energy));
				}
			}
		}

		if (candidateMarks.empty())
		{
			continue;
		}

		//寻找候选特征点中能量值最大的marker
		for (size_t i = 0; i < candidateMarks.size(); i++)
		{
			double eTemp = candidateMarks[i].second;
			Feature::DotMarker markTemp = candidateMarks[i].first;
			if (eTemp > maxEnergy)
			{
				maxEnergy = eTemp;
				bestMark2 = markTemp;
				//bMathced = true;
			}
		}

		//须同时满足最短距离和最大能量值 
		if (bestMark2 == minDistMark)
		{
			bMathced = true;
		}
		else
			continue;
		
		//去除多个标志点匹配到一个点的情况
		//判断之前的候选点集合中是否已经存在当前的marker，如果存在 判断能量值大小，将小能量值的从最终结果中去掉
		if (MatchedMarkonly.count(bestMark2))
		{
			std::multimap<Feature::DotMarker, double>::iterator itr = MatchedMarkonly.find(bestMark2);
			if (itr->second < maxEnergy)
			{
				Feature::MarkerMultiMap::iterator itrP = markMathcedMapTmp.find(bestMark2);
				markMathcedMapTmp.erase(itrP);

				MatchedMarkonly.erase(itr);
				MatchedMarkonly.insert(std::make_pair(bestMark2, maxEnergy));

			}
			else
				continue;
			
		}
		else
		{
			MatchedMarkonly.insert(std::make_pair(bestMark2, maxEnergy));
		}

		if (bMathced)
		{
			marker1.poseList = Feature::calcMarkerPose(marker1, pFrame->_camera->Kl());
			bestMark2.poseList = Feature::calcMarkerPose(bestMark2, pFrame->_camera->Kr());
			bool ret = getBestMarkerPose(pFrame->_camera, marker1, bestMark2);
			if (!ret)
				continue;
			markMathcedMapTmp.insert(std::make_pair(bestMark2, marker1));
		}
	}
	//双向匹配验证
	//for (Feature::MarkerMultiMap::iterator iter = markMathcedMapTmp.begin(); iter != markMathcedMapTmp.end();iter++)
	//{
	//	Feature::DotMarker marker1 = iter->first;
	//	Feature::DotMarker marker2 = iter->second;
	//	cv::Mat x2D2 = (cv::Mat_<double>(2, 1) << marker2.undistortX, marker2.undistortY);
	//	cv::Mat line;
	//	cv::computeCorrespondEpilines(x2D2.t(), cameraId, _F, line);
	//	line = line.reshape(1, 3);
	//	const double& a = line.at<double>(0);
	//	const double& b = line.at<double>(1);
	//	const double& c = line.at<double>(2);
	//	double dist = abs(a*marker1.undistortX + b*marker1.undistortY + c) / sqrt(a*a + b*b);
	//	std::cout << "ReverseDist: " << dist << std::endl;
	//
	//	if (dist > MaxEpilineDist)
	//	{
	//		markMathcedMapTmp.erase(iter);
	//	}
	//}

	if (markMathcedMapTmp.empty())
		return false;

	for (Feature::MarkerMap::iterator itr = markMathcedMapTmp.begin(); itr != markMathcedMapTmp.end(); itr++)
	{//markerMathcedMap中的key为左相机标志点，值为右相机标志点
		if (markListLeft.size() < markListRight.size())
			markerMathcedMap.insert(std::make_pair(itr->second, itr->first));
		else
			markerMathcedMap.insert(std::make_pair(itr->first, itr->second));
	}
	return true;
}



bool Frame::calcStereoMatchesByEpipolar(Frame* pFrame, Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight, Feature::MarkerMultiMap& markerMathcedMap)
{
#ifndef STAGE 
	if (markListLeft.size() < 4 || markListRight.size() < 4)
		return false;
#endif

	Feature::MarkerList markList1, markList2;
	int cameraId;
	if (markListLeft.size() < markListRight.size())
	{
		markList1 = markListLeft;
		markList2 = markListRight;
		cameraId = 1;

	}
	else
	{
		markList1 = markListRight;
		markList2 = markListLeft;
		cameraId = 2;
	}

	//Feature::MarkerMap markMathcedMapTmp;
	Feature::MarkerMultiMap markMathcedMapTmp;
	for (Feature::MarkerList::iterator itr1 = markList1.begin(); itr1 != markList1.end(); itr1++)
	{//标志点数量markList2>markList1，以较少的标志点集合为基准遍历
		Feature::DotMarker& marker1 = *itr1;	

#if CV_MAJOR_VERSION < 3
		std::vector<cv::Point2f> points;
		points.emplace_back(cv::Point2d(marker1.undistortX, marker1.undistortY));
		std::vector<cv::Vec<float, 3>> lines;
		cv::computeCorrespondEpilines(points, cameraId, pFrame->getCamera()->F(), lines);
		const double& a = lines.back()[0];
		const double& b = lines.back()[1];
		const double& c = lines.back()[2];
#else
		cv::Mat x2D1 = (cv::Mat_<double>(2, 1) << marker1.undistortX, marker1.undistortY);
		cv::Mat line;
		cv::computeCorrespondEpilines(x2D1.t(), cameraId, pFrame->_camera->F(), line);
		line = line.reshape(1, 3);
		const double& a = line.at<double>(0);
		const double& b = line.at<double>(1);
		const double& c = line.at<double>(2);
#endif

		double dist=0;
		Feature::DotMarker bestMark2;
		double minDist = DBL_MAX;
		bool bMathced = false;
		for (Feature::MarkerList::iterator itr2 = markList2.begin(); itr2 != markList2.end(); itr2++)
		{
			Feature::DotMarker& marker2 = *itr2;
	
			dist = abs(a*marker2.undistortX + b*marker2.undistortY + c) / sqrt(a*a + b*b);	
			if (dist < MaxEpilineDist)//阈值需根据标定精准度调整，标定过差会损失标志点
			{
				marker1.poseList = Feature::calcMarkerPose(marker1, pFrame->_camera->Kl());
				marker2.poseList = Feature::calcMarkerPose(marker2, pFrame->_camera->Kr());
				bool ret = getBestMarkerPose(pFrame->_camera, marker1, marker2);
				if (!ret)
					continue;
				if (dist < minDist)
				{
					minDist = dist;
					bestMark2 = marker2;
					bMathced = true;
				}
			}
		}
		if (bMathced)
			//markMathcedMapTmp[bestMark2] = marker1;//key为marker2可以避免多个marker1匹配上一个marker2
			markMathcedMapTmp.insert(std::make_pair(bestMark2, marker1));
	}

	if (markMathcedMapTmp.empty())
		return false;

	for (Feature::MarkerMultiMap::iterator itr = markMathcedMapTmp.begin(); itr != markMathcedMapTmp.end(); itr++)
	{//markerMathcedMap中的key为左相机标志点，值为右相机标志点
		if (markListLeft.size() < markListRight.size())
			markerMathcedMap.insert(std::make_pair(itr->second, itr->first));
		else
			markerMathcedMap.insert(std::make_pair(itr->first, itr->second));
	}
	return true;
}



bool Frame::calcStereoMatchesByNorms(Frame* pFrame, Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight, Feature::MarkerMultiMap& markerMathcedMap)
{
#ifndef STAGE 
	if (markListLeft.size() < 4 || markListRight.size() < 4)
		return false;
#endif

	Feature::MarkerMap markMathcedMapTmp;
	for (Feature::MarkerList::iterator itr1 = markListLeft.begin(); itr1 != markListLeft.end(); itr1++)
	{
		Feature::DotMarker& marker1 = *itr1;
#if CV_MAJOR_VERSION < 3
		std::vector<cv::Point2f> points;
		points.emplace_back(cv::Point2d(marker1.undistortX, marker1.undistortY));
		std::vector<cv::Vec<float, 3>> lines;
		cv::computeCorrespondEpilines(points, 1, pFrame->getCamera()->F(), lines);
		const double& a = lines.back()[0];
		const double& b = lines.back()[1];
		const double& c = lines.back()[2];
#else
		cv::Mat x2D1 = (cv::Mat_<double>(2, 1) << marker1.undistortX, marker1.undistortY);
		cv::Mat line;
		cv::computeCorrespondEpilines(x2D1.t(), 1, pFrame->_camera->F(), line);
		line = line.reshape(1, 3);
		const double& a = line.at<double>(0);
		const double& b = line.at<double>(1);
		const double& c = line.at<double>(2);
#endif
		double epiLineDist = 0.;
		Feature::DotMarker bestMarker2;
		bool bMatched = false; 
		double minCenterDist = DBL_MAX;
		double centerDist = 0;
		for (Feature::MarkerList::iterator itr2 = markListRight.begin(); itr2 != markListRight.end(); itr2++)
		{
			Feature::DotMarker& marker2 = *itr2;

			epiLineDist = abs(a*marker2.undistortX + b*marker2.undistortY + c) / sqrt(a*a + b*b);
			//cv::Mat x2D2 = (cv::Mat_<double>(3, 1) << marker2.undistortX, marker2.undistortY, 1);
			//double dist = abs(x2D2.dot(line))/sqrt(a*a+b*b);
			if (epiLineDist < MaxEpilineDist)//阈值需根据标定精准度调整，标定过差会损失标志点
			{		
				marker1.poseList = Feature::calcMarkerPose(marker1, pFrame->_camera->Kl());
				marker2.poseList = Feature::calcMarkerPose(marker2, pFrame->_camera->Kr());
				const bool& ret = getBestMarkerPose(pFrame->_camera, marker1, marker2);
				if (!ret)
					continue; 
				bMatched = true;
			
				centerDist = cv::norm(marker1.bestPose.center3d - pFrame->_camera->R().t()*marker2.bestPose.center3d + pFrame->_camera->R().t()*pFrame->_camera->t());
				if (centerDist < minCenterDist)
				{					
					minCenterDist = centerDist;
					bestMarker2 = marker2;		
				}				
			}			
		}
		if(bMatched)
			markerMathcedMap.insert(std::make_pair(marker1, bestMarker2));
	}
	
	if (markerMathcedMap.empty())
		return false;
	return true;
}



bool Frame::getBestMarkerPose(Camera* camera, Feature::DotMarker& dotMarker1, Feature::DotMarker& dotMarker2)
{
	double minAngleDiff = DBL_MAX;
	double err = 0;
	bool bMatched=false;
	Feature::MarkerPose bestPose1, bestPose2;
	for (auto itr1 = dotMarker1.poseList.begin(); itr1 != dotMarker1.poseList.end(); itr1++)
	{
		for (auto itr2 = dotMarker2.poseList.begin(); itr2 != dotMarker2.poseList.end(); itr2++)
		{
			const Feature::MarkerPose& pose1 = *itr1;
			const Feature::MarkerPose& pose2 = *itr2;
			//double cosAngle = abs(pose1.normVector.dot(pose2.normVector) / (cv::norm(pose1.normVector)*cv::norm(pose2.normVector)));
			//double angleDiff = acos(cosAngle) / PI * 180;
			double dist = cv::norm(pose1.center3d - camera->R().t()*pose2.center3d + camera->R().t()* camera->t());
			if (dist > MarkerCenterThres)
				return false;
			double angleDiff = Feature::calcVecAngle(pose1.normVector, camera->R().t()* pose2.normVector) / PI * 180;
			if (angleDiff > MarkerAngleThres)
				continue;
			if (angleDiff < minAngleDiff)
			{	
				minAngleDiff = angleDiff;
				bestPose1 = pose1;
				bestPose2 = pose2;
				bMatched = true;					
			}
		}
	}
	if (bMatched)
	{
		dotMarker1.bestPose = bestPose1;
		dotMarker2.bestPose = bestPose2;
		return true;
	}
		
		
	return false;
}



void Frame::unprojectStereo(const Feature::MarkerPair& MarkerPair, cv::Point3d* p3d)
{
	const Feature::DotMarker& markerLeft = MarkerPair.first;
	const Feature::DotMarker& markerRight = MarkerPair.second;
	cv::Mat x2DL = (cv::Mat_<double>(2, 1) << markerLeft.undistortX, markerLeft.undistortY);
	cv::Mat x2DR = (cv::Mat_<double>(2, 1) << markerRight.undistortX, markerRight.undistortY);
	cv::Mat x3Dc;

	cv::Mat Pl = _camera->Kl()*cv::Mat::eye(3, 4, CV_64F);
	cv::Mat T = cv::Mat(3, 4, CV_64F);
	_camera->R().copyTo(T.rowRange(0, 3).colRange(0, 3));
	_camera->t().copyTo(T.rowRange(0, 3).col(3));
	cv::Mat Pr = _camera->Kr()*T;
	//Pr = Pr.rowRange(0, 3);

	cv::triangulatePoints(Pl, Pr, x2DL, x2DR, x3Dc);
	p3d->x = x3Dc.at<double>(0) / x3Dc.at<double>(3);
	p3d->y = x3Dc.at<double>(1) / x3Dc.at<double>(3);
	p3d->z = x3Dc.at<double>(2) / x3Dc.at<double>(3);
	/*
	if (_Q.empty())
	return false;

	const float u =  MarkerPair.first.x;
	const float v = MarkerPair.first.y;
	const float d = MarkerPair.first.x- MarkerPair.second.x;
	cv::Mat x2D= (cv::Mat_<double>(4, 1) << u, v, d,1);
	cv::Mat x3Dc = _Q*x2D;
	x3Dc = x3Dc / x3Dc.at<double>(3);

	p3d->x = x3Dc.at<double>(0);
	p3d->y = x3Dc.at<double>(1);
	p3d->z = x3Dc.at<double>(2);*/
}




bool Frame::compute3Dpoints(const cv::Mat& imgLeft, const cv::Mat& imgRight, CalcMatchedFunc func1, Feature::ExtractMarkFunc func2)
{
	if (_system->isOutputStereoImg() || _system->isOutputTrackImg())
	{
		_markImgLeft = imgLeft.clone();
		_markImgRight = imgRight.clone();
		if (imgLeft.channels() == 1)
		{
			cv::cvtColor(_markImgLeft, _markImgLeft, CV_GRAY2BGR);
			cv::cvtColor(_markImgRight, _markImgRight, CV_GRAY2BGR);
		}

	}
	cv::Mat imGrayLeft = imgLeft;
	cv::Mat imGrayRight = imgRight;
	if (imgLeft.channels() == 3)
	{
		if (_camera->isRGB())
		{
			cv::cvtColor(imgLeft, imGrayLeft, CV_RGB2GRAY);
			cv::cvtColor(imgRight, imGrayRight, CV_RGB2GRAY);
		}
		else
		{
			cv::cvtColor(imGrayLeft, imGrayLeft, CV_BGR2GRAY);
			cv::cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
		}
	}
	else if (imGrayLeft.channels() == 4)
	{
		if (_camera->isRGB())
		{
			cv::cvtColor(imGrayLeft, imGrayLeft, CV_RGBA2GRAY);
			cv::cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
		}
		else
		{
			cv::cvtColor(imGrayLeft, imGrayLeft, CV_BGRA2GRAY);
			cv::cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
		}
	}
	Feature::MarkerList markListLeft, markListRight;
	func2(imGrayLeft, markListLeft);
	func2(imGrayRight, markListRight);
	//std::thread* featureThread1 = new std::thread(func2, imGrayLeft, std::ref(markListLeft));
	//std::thread* featureThread2 = new std::thread(func2, imGrayRight, std::ref(markListRight));
	//featureThread1->join();
	//featureThread2->join();

	if (_system->isOutputStereoImg())
	{
		createFrameMarkImg(_markImgLeft, markListLeft);
		createFrameMarkImg(_markImgRight, markListRight);
	}
	if (!isUndistorted())
	{
		for (Feature::MarkerList::iterator lItr = markListLeft.begin(); lItr != markListLeft.end(); lItr++)
			lItr->undistort(_camera->Kl(), _camera->Dl(), _camera->Kl());
		for (Feature::MarkerList::iterator rItr = markListRight.begin(); rItr != markListRight.end(); rItr++)
			rItr->undistort(_camera->Kr(), _camera->Dr(), _camera->Kr());
	}
	

#ifndef STAGE 
	Feature::MarkerMultiMap markerMathcedMap;	
	if (!func1(this, markListLeft, markListRight, markerMathcedMap))
	{
		fprintf(stderr, "Frame %d failed finding matched marks for stereo!\n", _id);
		return false;
	}
	if (markerMathcedMap.size() < 4)
	{
		fprintf(stderr, "Frame %d failed finding more than 3 matched marks for stereo!\n", _id);
		return false;
	}
#else
	Feature::MarkerMultiMap markerMathcedMap;
	func1(this, markListLeft, markListRight, markerMathcedMap);
#endif

	int i = 0;
	for (Feature::MarkerMap::iterator itr = markerMathcedMap.begin(); itr != markerMathcedMap.end(); itr++)
	{
		cv::Point3d* newP3d = new cv::Point3d();

		unprojectStereo(std::make_pair(itr->first, itr->second), newP3d);

		Feature::DotMarker markerLeft = itr->first;
		Feature::DotMarker markerRight = itr->second;
		KeyPoint* keyPointleft = new KeyPoint(markerLeft, i);
		KeyPoint* keyPointRight = new KeyPoint(markerRight, i);

		for (auto itr = markerLeft.poseList.begin(); itr != markerLeft.poseList.end(); itr++)
			keyPointleft->_normsList.emplace_back(itr->normVector);
		for (auto itr = markerRight.poseList.begin(); itr != markerRight.poseList.end(); itr++)
			keyPointRight->_normsList.emplace_back(itr->normVector);
		keyPointleft->_bestNorm = markerLeft.bestPose.normVector;
		keyPointRight->_bestNorm = markerRight.bestPose.normVector;

		//keyPointleft->_weight = 1.;
		//keyPointRight->_weight = 1.;
		///////////特征点权重设置////////////////
		keyPointleft->calcWeight(markerLeft);
		keyPointRight->calcWeight(markerRight);

		addKeyPointsLeft(keyPointleft);
		addKeyPointsRight(keyPointRight);
		insert3DPoints(newP3d, keyPointleft, keyPointRight);

		if (_system->isOutputP3d())
		{
			if (i == 0)
				_system->p3dOfstream() << "Frame " << getID() << ": " << std::endl;

			_system->p3dOfstream() << "3D: " << newP3d->x <<" "<< newP3d->y << " " << newP3d->z
									<< " 2D LEFT: " << keyPointleft->_undistortX << " " << keyPointleft->_undistortY
									<< " 2D RIGHT: " << keyPointRight->_undistortX << " " << keyPointRight->_undistortY <<std::endl;
			
			if(itr == markerMathcedMap.end())
				_system->p3dOfstream() << std::endl;
		}

		i++;
	}


	return true;
}




bool Frame::compute3Dpoints(Feature::MarkerList& markListLeft, Feature::MarkerList& markListRight, CalcMatchedFunc func)
{
	if (!isUndistorted())
	{
		for (Feature::MarkerList::iterator lItr = markListLeft.begin(); lItr != markListLeft.end(); lItr++)
			lItr->undistort(_camera->Kl(), _camera->Dl(), _camera->Kl());
		for (Feature::MarkerList::iterator rItr = markListRight.begin(); rItr != markListRight.end(); rItr++)
			rItr->undistort(_camera->Kr(), _camera->Dr(), _camera->Kr());
	}

#ifndef STAGE 
	Feature::MarkerMultiMap markerMathcedMap;
	if (!func(this, markListLeft, markListRight, markerMathcedMap))
	{
		fprintf(stderr, "Frame %d failed finding matched marks for stereo!\n", _id);
		return false;
	}
	if (markerMathcedMap.size() < 4)
	{
		fprintf(stderr, "Frame %d failed finding more than 3 matched marks for stereo!\n", _id);
		return false;
	}
#else
	Feature::MarkerMultiMap markerMathcedMap;
	func(this, markListLeft, markListRight, markerMathcedMap);
#endif

	int i = 0;
	for (Feature::MarkerMap::iterator itr = markerMathcedMap.begin(); itr != markerMathcedMap.end(); itr++)
	{
		cv::Point3d* newP3d = new cv::Point3d();

		unprojectStereo(std::make_pair(itr->first, itr->second), newP3d);

		Feature::DotMarker markerLeft = itr->first;
		Feature::DotMarker markerRight = itr->second;
		KeyPoint* keyPointleft = new KeyPoint(markerLeft,i);
		KeyPoint* keyPointRight = new KeyPoint(markerRight, i);

		for (auto itr = markerLeft.poseList.begin(); itr != markerLeft.poseList.end(); itr++)
			keyPointleft->_normsList.emplace_back(itr->normVector);
		for (auto itr = markerRight.poseList.begin(); itr != markerRight.poseList.end(); itr++)
			keyPointRight->_normsList.emplace_back(itr->normVector);
		keyPointleft->_bestNorm = markerLeft.bestPose.normVector;
		keyPointRight->_bestNorm = markerRight.bestPose.normVector;

		//keyPointleft->_weight = 1.;
		//keyPointRight->_weight = 1.;
		keyPointleft->calcWeight(markerLeft);
		keyPointRight->calcWeight(markerRight);

		addKeyPointsLeft(keyPointleft);
		addKeyPointsRight(keyPointRight);
		insert3DPoints(newP3d, keyPointleft, keyPointRight);
		i++;
	}

	return true;
}



void Frame::insertMatchedMPt(MapPoint3d* const mapPoint, KeyPoint* const keyPointL, KeyPoint* const keyPointR)
{
	_matchedMptMap[mapPoint] = std::make_pair(keyPointL, keyPointR);
	keyPointL->setMatchedMpt(mapPoint);
	keyPointR->setMatchedMpt(mapPoint);
}



void Frame::insertOutLier(MapPoint3d* pMP)
{
	if (!_outLiers.count(pMP))
	{
		_outLiers.insert(pMP);
		pMP->_outLierCount++;
		if (pMP->obs() > Tracker::OBSERVE_THRES)
			if ((double)pMP->_outLierCount / (double)pMP->obs() > 0.5)
			{
				pMP->setBad(true);
				_globalMap->insertBadMpt(pMP);
			}
		if (_matchedMptMap.size() - _outLiers.size() < 4)
		{
			setBad(true);
			_globalMap->insertBadFrame(this);
		}			
	}	
	else return;
}



void Frame::eraseOutlier(MapPoint3d* pMP)
{
	if (_outLiers.count(pMP))
	{
		_outLiers.erase(pMP);
		pMP->_outLierCount--;
	}
	else
		return;
}



void Frame::clearOutLiers()
{
	for (auto oItr = _outLiers.begin(); oItr != _outLiers.end(); oItr++)
		(*oItr)->_outLierCount--;

	_outLiers.clear();
}



void Frame::setPose(cv::Mat Tcw)
{
	_Tcw = Tcw.clone();
	updatePoseMatrices();
}



void Frame::setPose(cv::Mat Rcw,cv::Mat tcw)
{
	if(_Tcw.empty())
		_Tcw = cv::Mat::zeros(4, 4, CV_64F);
	Rcw.copyTo(_Tcw.colRange(0, 3).rowRange(0, 3));
	tcw.copyTo(_Tcw.rowRange(0, 3).col(3));
	updatePoseMatrices();
}



void Frame::updatePoseMatrices()
{
	_Rcw = _Tcw.rowRange(0, 3).colRange(0, 3);
	_Rwc = _Rcw.t();
	_tcw = _Tcw.rowRange(0, 3).col(3);
	_Ow = -_Rwc*_tcw;//相机中心世界坐标

	_Twc = cv::Mat::eye(4, 4, _Tcw.type());
	_Rwc.copyTo(_Twc.rowRange(0, 3).colRange(0, 3));
	_Ow.copyTo(_Twc.rowRange(0, 3).col(3));
}



cv::Point3d Frame::localToWorld(cv::Point3d* point3D)
{
	if (point3D->z>0)
	{
		cv::Mat x3Dc = (cv::Mat_<double>(3, 1) << point3D->x, point3D->y, point3D->z);
		cv::Mat x3Dw=_Rwc * x3Dc + _Ow;
		return cv::Point3d(x3Dw.at<double>(0,0), x3Dw.at<double>(1, 0), x3Dw.at<double>(2, 0));
	}
	else
		return cv::Point3d();
}



void Frame::undistortKeyPoint()
{
	if (_keyPointsLeft.empty() || _keyPointsRight.empty())
	{
		fprintf(stderr, "Undistort keyPoint failed! Frame %d has no valid keyPoints\n", _id);
	}
	for (KeyPointList::iterator itr = _keyPointsLeft.begin(); itr != _keyPointsLeft.end(); itr++)
	{
		(*itr)->undistort(_camera->Kl(),_camera->Dl());
	}
	for (KeyPointList::iterator itr = _keyPointsRight.begin(); itr != _keyPointsRight.end(); itr++)
	{
		(*itr)->undistort(_camera->Kr(), _camera->Dr());
	}
}



cv::Mat Frame::calcRelativeVelocity(Frame& prevFrame)
{
	cv::Mat relativeVelocity;
	if (!prevFrame._Tcw.empty())
	{
		/*cv::Mat lastTwc = cv::Mat::eye(4, 4, CV_64F);
		prevFrame.getRotationInverse().copyTo(lastTwc.rowRange(0, 3).colRange(0, 3));
		prevFrame.getCameraCenter().copyTo(lastTwc.rowRange(0, 3).col(3));
		relativeVelocity = _Tcw*lastTwc;*/
		relativeVelocity = _Tcw*prevFrame.getPoseInverse();
	}
	else
		relativeVelocity = cv::Mat();
	return relativeVelocity;
}


void Frame::calcMotionModel(const Frame& prevFrame)
{
	if (!prevFrame._Tcw.empty())
	{
		/*
		cv::Mat lastTwc = cv::Mat::eye(4, 4, CV_64F);
		prevFrame.getRotationInverse().copyTo(lastTwc.rowRange(0, 3).colRange(0, 3));
		prevFrame.getCameraCenter().copyTo(lastTwc.rowRange(0, 3).col(3));*/
		_refMotion = _Tcw*prevFrame.getPoseInverse();
	}
	else
		_refMotion = cv::Mat();

}



bool Frame::searchMptMatchedByFrame(const Frame& prevFrame, const P3DPairList& matchedP3DList, P3DMptPairList& p3DMptMatchedList)
{
	cv::Point3d* prevP3D=nullptr;
	cv::Point3d* currentP3D = nullptr;
	KeyPoint* prevKeyLeft = nullptr;
	KeyPoint* currentKeyLeft = nullptr;

	for (P3DPairList::const_iterator itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++)
	{
		currentP3D = itr->first;
		prevP3D = itr->second;

		auto itrCur = _p3DMap.find(currentP3D);
		auto itrPrev = prevFrame._p3DMap.find(prevP3D);

		if (itrCur != _p3DMap.end() && itrPrev != prevFrame._p3DMap.end())
		{
			currentKeyLeft = itrCur->second.first;
			prevKeyLeft = itrPrev->second.first;
		}
		else
			return false;

		KeyPoint* currentKeyRight = getKeyPointRight(currentKeyLeft);
		assert(currentKeyRight);
	
		MapPoint3d* refMapPoint = prevKeyLeft->_matchedMpt;
		if (refMapPoint)
		{
			if (refMapPoint->isBad())
				continue;
			insertMatchedMPt(refMapPoint, currentKeyLeft, currentKeyRight);
			refMapPoint->addObservation(this);
			p3DMptMatchedList.emplace_back(std::make_pair(currentP3D, refMapPoint));
		}	
		
		/*else
			return false;*/
	}
	return true;
}


void Frame::searchMptMatchedByGlobalMap(P3DMptPairList& p3dMptPairList)
{
	MapPoint3d* mapPoint=nullptr;
	cv::Point3d* currentP3D = nullptr;
	KeyPoint* currentKeyLeft = nullptr;
	KeyPoint* currentKeyRight = nullptr;


	for (P3DMptPairList::iterator itr = p3dMptPairList.begin(); itr != p3dMptPairList.end(); itr++)
	{
		currentP3D = itr->first;
		
		currentKeyLeft = _p3DMap.find(currentP3D)->second.first;
		currentKeyRight = getKeyPointRight(currentKeyLeft);
		assert(currentKeyRight);

		mapPoint = itr->second;

		if (mapPoint->isBad())
		{
			insertMatchedMPt(mapPoint, currentKeyLeft, currentKeyRight);
			mapPoint->addObservation(this);
		}
		
	}

}



void Frame::updateNormMatched()
{
	const double& angleThres = MarkerAngleThres;
	for (auto mItr = _matchedMptMap.begin(); mItr != _matchedMptMap.end(); mItr++)
	{
		MapPoint3d* mpt = mItr->first;
		if (_outLiers.count(mpt))
			continue;
		if (mpt->isBad())
			continue;
		KeyPoint* kptLeft = mItr->second.first;
		KeyPoint* kptRight = mItr->second.second;
		assert(kptLeft || kptRight);

		cv::Mat frameNorm = kptLeft->_bestNorm* kptLeft->_weight +
						    _camera->R().t()* kptRight->_bestNorm* kptRight->_weight;
		frameNorm /= cv::norm(frameNorm);
		frameNorm = _Rwc *frameNorm;
		if (Feature::calcVecAngle(frameNorm, mpt->_normOri) / PI * 180 < angleThres)
		{
			mpt->_normLast += frameNorm;
			mpt->_normLast /= cv::norm(mpt->_normLast);
		}
			
		else
		{
			bool bMatched = false;
			for (auto itr1 = kptLeft->_normsList.begin(); itr1 != kptLeft->_normsList.end(); itr1++)
			{
				for (auto itr2 = kptRight->_normsList.begin(); itr2 != kptRight->_normsList.end(); itr2++)
				{
					cv::Mat norm1= *itr1;
					cv::Mat norm2= *itr2;
				
					double cosAngle1 = abs(norm1.dot(norm2) / (cv::norm(norm1)*cv::norm(norm2)));
					double angleDiff1 = acos(cosAngle1) / PI * 180;
					
					if (angleDiff1 > angleThres)
						continue;//跳过左右相机法向量夹角较大的法向量
					cv::Mat frameNorm = norm1* kptLeft->_weight+ _camera->R().t() * norm2* kptRight->_weight;//融合左右相机法向量
					frameNorm /= cv::norm(frameNorm);
					frameNorm = _Rwc *frameNorm;

					double cosAngle2 = abs(frameNorm.dot(mpt->_normOri) / (cv::norm(frameNorm)*cv::norm(mpt->_normOri)));
					double angleDiff2 = acos(cosAngle2) / PI * 180;
					if (angleDiff2 < angleThres)
					{//筛选与匹配地图点夹角较小的法向量
						kptLeft->_bestNorm = norm1;
						kptRight->_bestNorm = norm2;
					
						mpt->_normLast += frameNorm;//融合当前帧和地图点的法向量
						mpt->_normLast /= cv::norm(mpt->_normLast);
						bMatched = true;
						break;
					}
				}
			}
			if (!bMatched)
				insertOutLier(mpt);
		}			
	}
}



KeyFrame* Frame::getKFInGlobalMap()
{
	KeyFrameList& kFList = _globalMap->getKeyFrameList();

	for (KeyFrameList::iterator itr = kFList.begin(); itr != kFList.end(); itr++)
	{
		Frame* frame = dynamic_cast<Frame*>(*itr);
		if (this->getID() == frame->getID())
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


bool Frame::outputPos(std::ofstream& ofs)
{
	if (!ofs.is_open())
	{
		std::cout << "Failed Opening source file" << std::endl;
		return false;
	}


	ofs <<std::setprecision(8) <<getID() << " " << 
		_Rwc.at<double>(0, 0) << " " << _Rwc.at<double>(0, 1) << " " << _Rwc.at<double>(0, 2) << " " <<
		_Rwc.at<double>(1, 0) << " " << _Rwc.at<double>(1, 1) << " " << _Rwc.at<double>(1, 2) << " " <<
		_Rwc.at<double>(2, 0) << " " << _Rwc.at<double>(2, 1) << " " << _Rwc.at<double>(2, 2) << " " <<
		_Ow.at<double>(0) << " " << _Ow.at<double>(1) << " " << _Ow.at<double>(2) << std::endl;
	return true;
}



void Frame::createFrameMarkImg(cv::Mat& Img, const Feature::MarkerList& markList)
{
	for (Feature::MarkerList::const_iterator itr = markList.begin(); itr != markList.end(); itr++)
	{
		//cv::Point2f vertices[4];
		//(*itr).rect.points(vertices);
		//for (int i = 0; i < 4; i++)
		//	line(Img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 255, 255));
		////cv::rectangle(outImg, itr->rect.boundingRect(), cv::Scalar(255, 255, 255));
		//std::ostringstream os1,os2;
		//os1 << std::setprecision(3) << itr->ellipseFitError;
		//cv::putText(Img, os1.str(), itr->rect.center + cv::Point2f(itr->rect.size.width / 2 + 5, itr->rect.size.height / 2 + 5), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
		//os2<< std::setprecision(3) << itr->definition;
		//cv::putText(Img, os2.str(), itr->rect.center + cv::Point2f(itr->rect.size.width / 2 + 5, itr->rect.size.height / 2 + 20), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
		//if (itr->fBad)
			//cv::circle(Img, cv::Point(itr->rect.center.x, itr->rect.center.y), 50, cv::Scalar(255, 255, 255), 1);

		std::vector<std::vector<cv::Point>> contourList;
		contourList.emplace_back(itr->contours);
		cv::drawContours(Img, contourList, 0, cv::Scalar(255, 0, 0), -1);
	}
}


cv::Mat& Frame::createMatchedImg(bool isTackedOK)
{
	//const cv::Mat& imgLeft = _markImgLeft;
	//const cv::Mat& imgRight = _markImgRight;
	//int wl, hl, wr, hr, w, h;
	//
	//wl = cvRound(imgLeft.cols);
	//hl = cvRound(imgLeft.rows);
	//wr = cvRound(imgRight.cols);
	//hr = cvRound(imgRight.rows);
	//w = wl + wr;
	//h = hl > hr ? hl : hr;
	////_stereoImg.create(h, w + 10, CV_8UC3);
	//_stereoImg = cv::Mat(h, w + 5, CV_8UC3, cv::Scalar(255, 255, 255));
	//
	//cv::Mat imgLeftBGR, imgRightBGR;
	////cvtColor(imgLeft, imgLeftBGR, cv::COLOR_GRAY2BGR);
	////cvtColor(imgRight, imgRightBGR, cv::COLOR_GRAY2BGR);
	//cv::Mat canvasLeft = _stereoImg(cv::Rect(0, 0, wl, h));
	//cv::Mat canvasRight = _stereoImg(cv::Rect(wl + 5, 0, wr, h));
	//resize(imgLeft, canvasLeft, canvasLeft.size(), 0, 0, cv::INTER_AREA);
	//resize(imgRight, canvasRight, canvasRight.size(), 0, 0, cv::INTER_AREA);

	for (P3DKeyPairMap::const_iterator pItr = _p3DMap.begin(); pItr != _p3DMap.end(); pItr++)
	{
		KeyPoint* keyPointLeft = pItr->second.first;
		KeyPoint* keyPointRight = pItr->second.second;

		cv::Point2f vertices[4];
		if (keyPointLeft->_matchedMpt)
		{
			keyPointLeft->_rect.points(vertices);			
			cv::rectangle(_markImgLeft, keyPointLeft->_rect.boundingRect(), cv::Scalar(0, 255, 0), 4);

			keyPointRight->_rect.points(vertices);
			cv::rectangle(_markImgRight, keyPointRight->_rect.boundingRect(), cv::Scalar(0, 255, 0), 4);
		}
		else
		{		
			if (_globalMap->getKeyFrameList().empty())
			{
				keyPointLeft->_rect.points(vertices);
				cv::rectangle(_markImgLeft, keyPointLeft->_rect.boundingRect(), cv::Scalar(0, 255, 0), 4);

				keyPointRight->_rect.points(vertices);
				cv::rectangle(_markImgRight, keyPointRight->_rect.boundingRect(), cv::Scalar(0, 255, 0), 4);
			}
			else
			{
				keyPointLeft->_rect.points(vertices);
				cv::rectangle(_markImgLeft, keyPointLeft->_rect.boundingRect(), cv::Scalar(0, 0, 255), 4);

				keyPointRight->_rect.points(vertices);
				cv::rectangle(_markImgRight, keyPointRight->_rect.boundingRect(), cv::Scalar(0, 0, 255), 4);
			}
		}
	}


	//////////////////////重投影3d点（蓝绿），并连接匹配3d点/////////////////////////
	//std::vector<cv::Vec<double, 2>> x2DListLeft, x2DListRight;
	//cv::projectPoints(p3DList, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), _camera->Kl(), _camera->Dl(), x2DListLeft);
	//cv::projectPoints(p3DList, _camera->R(), _camera->t(), _camera->Kr(), _camera->Dr(), x2DListRight);
	//
	//for (int i = 0; i < p3DList.size(); i++)
	//{
	//	cv::Point p2Dl, p2Dr;
	//	p2Dl.x = x2DListLeft[i][0];
	//	p2Dl.y = x2DListLeft[i][1];
	//	p2Dr.x = x2DListRight[i][0] + wl + 5;
	//	p2Dr.y = x2DListRight[i][1];
	//
	//	cv::circle(_stereoImg, p2Dl, 5, cv::Scalar(255, 255, 0), 1);
	//	cv::circle(_stereoImg, p2Dr, 5, cv::Scalar(255, 255, 0), 1);
	//	//cv::line(_stereoImg, p2Dl, p2Dr, cv::Scalar(0, 255, 0));
	//}
	//绘制当前帧3d点分别在左相机对应的标志点中心（畸变矫正，橙色）


	//auto& p3DMatchedList = _system->getTracker().getMatchedP3DList();
	//for (auto pItr = p3DMatchedList.begin(); pItr != p3DMatchedList.end(); pItr++)
	//{
	//	KeyPointPair kptPair;
	//	getKeyPairFromP3D(pItr->first, kptPair);
	//	KeyPoint* keyPointLeft = kptPair.first;
	//	KeyPoint* keyPointRight = kptPair.second;
	//
	//	cv::Point2f vertices[4];
	//	keyPointLeft->_rect.points(vertices);
	//
	//	/*for (int i = 0; i < 4; i++)
	//	line(canvasLeft, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0),2);*/
	//	cv::rectangle(_markImgLeft, keyPointLeft->_rect.boundingRect(), cv::Scalar(0, 255, 0), 2);
	//
	//	keyPointRight->_rect.points(vertices);
	//	/*for (int i = 0; i < 4; i++)
	//	line(canvasRight, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);*/
	//	cv::rectangle(_markImgRight, keyPointRight->_rect.boundingRect(), cv::Scalar(0, 255, 0), 2);
	//}

	/*
	char  matchedCount[3];
	itoa(_p3DMap.size(), matchedCount, 10);

	std::string text1 = "Marks: " + std::string(matchedCount);
	std::string text2;
	std::ostringstream os1;
	if (isTackedOK)
		os1 << "Matched RMS:" << std::setprecision(4) << calcMatchedRMSE();
	else
		os1 << "";
	putText(_stereoImg, text1, cv::Point(30, 40), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 2);
	putText(_stereoImg, os1.str(), cv::Point(30, 70), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 2);
	if (isTackedOK)
	{
		text2 =  "Ready";
		putText(_stereoImg, text2, cv::Point(30, 130), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 255, 0), 2);
	}
	else
	{
		text2 = "Lost";
		putText(_stereoImg, text2, cv::Point(30, 130), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 0, 255), 2);
	}*/
	
	return _stereoImg;
}


bool Frame::createMatchedImg()
{
	if (_p3DMap.empty())
		return false;

	const cv::Mat& imgLeft = _markImgLeft;
	const cv::Mat& imgRight = _markImgRight;
	int wl, hl, wr, hr, w, h;

	wl = cvRound(imgLeft.cols);
	hl = cvRound(imgLeft.rows);
	wr = cvRound(imgRight.cols);
	hr = cvRound(imgRight.rows);
	w = wl + wr;
	h = hl > hr ? hl : hr;
	//_stereoImg.create(h, w + 10, CV_8UC3);
	_stereoImg=cv::Mat(h, w + 5, CV_8UC3,cv::Scalar(255,255,255));

	cv::Mat imgLeftBGR, imgRightBGR;
	//cvtColor(imgLeft, imgLeftBGR, cv::COLOR_GRAY2BGR);
	//cvtColor(imgRight, imgRightBGR, cv::COLOR_GRAY2BGR);
	cv::Mat canvasLeft = _stereoImg(cv::Rect(0, 0, wl, h));
	cv::Mat canvasRight = _stereoImg(cv::Rect(wl + 5, 0, wr, h));
	resize(imgLeft, canvasLeft, canvasLeft.size(), 0, 0, cv::INTER_AREA);
	resize(imgRight, canvasRight, canvasRight.size(), 0, 0, cv::INTER_AREA);

	//绘制当前帧3d点分别在左相机对应的标志点中心（畸变矫正，橙色）
	std::vector<cv::Point3d> p3DList;
	for (P3DKeyPairMap::const_iterator pItr = _p3DMap.begin(); pItr != _p3DMap.end(); pItr++)
	{
		KeyPoint* keyPointLeft = pItr->second.first;
		KeyPoint* keyPointRight = pItr->second.second;
		cv::circle(canvasLeft, cv::Point(keyPointLeft->x, keyPointLeft->y), 2, cv::Scalar(0, 0, 255), -1);
		cv::circle(canvasRight, cv::Point(keyPointRight->x, keyPointRight->y), 2, cv::Scalar(0, 0, 255), -1);
		cv::circle(canvasLeft, cv::Point(keyPointLeft->_undistortX, keyPointLeft->_undistortY), 6, cv::Scalar(0, 255, 255), 1);
		cv::circle(canvasRight, cv::Point(keyPointRight->_undistortX, keyPointRight->_undistortY), 6, cv::Scalar(0, 255, 255), 1);

		p3DList.emplace_back(*pItr->first);
		//cv::Mat x2DL,x2DR;

	}

	////////////////////重投影3d点（蓝绿），并连接匹配3d点/////////////////////////
	std::vector<cv::Vec<double, 2>> x2DListLeft, x2DListRight;
	cv::projectPoints(p3DList, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), _camera->Kl(), _camera->Dl(), x2DListLeft);
	cv::projectPoints(p3DList, _camera->R(), _camera->t(), _camera->Kr(), _camera->Dr(), x2DListRight);

	for (int i = 0; i < p3DList.size(); i++)
	{
		cv::Point p2Dl, p2Dr;
		p2Dl.x = x2DListLeft[i][0];
		p2Dl.y = x2DListLeft[i][1];
		p2Dr.x = x2DListRight[i][0] + wl + 5;
		p2Dr.y = x2DListRight[i][1];

		cv::circle(_stereoImg, p2Dl, 5, cv::Scalar(255, 255, 0 ),1);
		cv::circle(_stereoImg, p2Dr, 5, cv::Scalar(255, 255, 0), 1);
		cv::line(_stereoImg, p2Dl, p2Dr, cv::Scalar(0, 255, 0));
	}

	return true;
}


void Frame::releaseImg()
{
	if (!_markImgLeft.empty())
		_markImgLeft.release();
	if (!_markImgRight.empty())
		_markImgRight.release();
	if (!_stereoImg.empty())
		_stereoImg.release();
}


int Frame::getCovisibleCount(Frame* pF)
{
	auto itr = _orderedConnecteds.find(pF);
	if (itr != _orderedConnecteds.end())
		return itr->second;
	else
		return 0;
}



FrameList Frame::getCovisibles()
{
	FrameList bestCovisibles;

	std::vector<std::pair<Frame*, int>> covisibleList(_orderedConnecteds.begin(), _orderedConnecteds.end());
	sort(covisibleList.begin(), covisibleList.end(), DerefFrameFunctor<std::pair<Frame*, int>>());//按照观察帧数从小到大，若观察帧数相同，则按id号从小到大

	for (auto itr = covisibleList.rbegin(); itr != covisibleList.rend(); itr++)
	{//按照观察帧数从大到小遍历共点帧，当观察帧数相同，按ID号从大到小遍历
		bestCovisibles.emplace_back(itr->first);
	}
	return bestCovisibles;
}



FrameList Frame::getBestCovisibles(const int &N)
{
	FrameList bestCovisibles;

	std::vector<std::pair<Frame*, int>> covisibleList(_orderedConnecteds.begin(), _orderedConnecteds.end());
	sort(covisibleList.begin(), covisibleList.end(), DerefFrameFunctor<std::pair<Frame*, int>>());//按照观察帧数从小到大，若观察帧数相同，则按id号从小到大

	int count = 0;
	for (auto itr = covisibleList.rbegin(); itr != covisibleList.rend(); itr++, count++)
	{//按照观察帧数从大到小遍历前N各共点帧，当观察帧数相同，按ID号从大到小遍历
		if (count >= N)
			break;
		bestCovisibles.emplace_back(itr->first);
	}

	return bestCovisibles;
}



void Frame::updateConnections()
{
	FrameWeight frameCounter;

	//获取当前帧对应的地图点，然后根据该点是否被其它帧观察到增加相应帧的观察数
	std::unique_lock<std::mutex> lock(Frame::_matchedMutex);
	for (MptKeyPairMap::iterator itr = _matchedMptMap.begin(); itr != _matchedMptMap.end(); itr++)
	{//遍历当前关键帧的匹配地图点
		MapPoint3d* mapPoint = itr->first;//mapPoint:当前关键帧的匹配地图点

		if (!mapPoint)
			continue;

		FrameSet observations = mapPoint->getObservations();//获取观察到该地图点pMP的关键帧集合（包括当前帧）

		for (FrameSet::iterator mit = observations.begin(); mit != observations.end(); mit++)
		{
			if ((*mit)->getID() == Frame::_id)//判断观察到该地图点mapPoint的帧是否是当前帧
				continue;
			frameCounter[*mit]++;//增加观察到该mapPoint的关键帧（除当前帧之外）的计数值(与当前关键帧的共点数)
		}
	}

	if (frameCounter.empty())
		return;

	int nmax = 0;
	Frame* frameMax = nullptr;
	int th = 0;


	for (FrameWeight::iterator mit = frameCounter.begin(); mit != frameCounter.end(); mit++)
	{
		if (mit->second>nmax)
		{
			nmax = mit->second;
			frameMax = mit->first;
		}
		if (mit->second >= th)
		{
			_orderedConnecteds.insert(std::make_pair(mit->first,mit->second));
			(mit->first)->addConnection(this, mit->second);
		}
	}

	if (_orderedConnecteds.empty())
	{
		_orderedConnecteds.insert(std::make_pair(frameMax, nmax));
		frameMax->addConnection(this, nmax);
	}



	if (_isFirstConnection && _id != 0)
	{
		_maxPrevConn = frameMax;
		_maxPrevConn->addMaxNextConn(this);
		_isFirstConnection = false;
	}

}



void Frame::eraseConnection(Frame* pF)
{
	auto itr = _orderedConnecteds.find(pF);
	if (itr != _orderedConnecteds.end())
		_orderedConnecteds.erase(itr);
}

//删除匹配地图点与对应3d点
bool Frame::eraseMptMatchedWithP3D(MapPoint3d* pMP)
{
	std::unique_lock<std::mutex> lock(_matchedMutex);
	auto matchItr = _matchedMptMap.find(pMP);
	if (matchItr != _matchedMptMap.end())
	{
		matchItr->second.first->setMatchedMpt(nullptr);
		matchItr->second.second->setMatchedMpt(nullptr);
		cv::Point3d* p3D = matchItr->second.first->_matchedP3D;
		if (p3D)
		{//删除对应3d点
			P3DKeyPairMap::iterator p3DItr = _p3DMap.find(p3D);
			if (p3DItr != _p3DMap.end())
			{
				_p3DMap.erase(p3DItr);
			}
		}
		_matchedMptMap.erase(matchItr);
		pMP->eraseObservation(this);

		if (getOutLiers().count(pMP))
			eraseOutlier(pMP);

		KeyFrame* pKF = dynamic_cast<KeyFrame*>(this);
		if (pKF)
			pMP->eraseKFObservation(pKF);

		if (_matchedMptMap.size()-_outLiers.size() < 4)
		{
			setBad(true);
			_globalMap->insertBadFrame(this);
		}
	}	
	else
	{
		fprintf(stdout, "Failed erasing map point matched in frame %d,for map point isn't matched in this frame", _id);
		return false;
	}
	return true;
}



void Frame::eraseMptMatchedWithP3D()
{
	std::unique_lock<std::mutex> lock(_matchedMutex);
	for (auto matchItr = _matchedMptMap.begin(); matchItr != _matchedMptMap.end(); matchItr++)
	{
		matchItr->second.first->setMatchedMpt(nullptr);
		matchItr->second.second->setMatchedMpt(nullptr);
		cv::Point3d* p3D = matchItr->second.first->_matchedP3D;
		if (p3D)
		{//删除对应3d点
			P3DKeyPairMap::iterator p3DItr = _p3DMap.find(p3D);
			if (p3DItr != _p3DMap.end())
			{
				_p3DMap.erase(p3DItr);
			}
		}
		_matchedMptMap.erase(matchItr);
		matchItr->first->eraseObservation(this);

		if (getOutLiers().count(matchItr->first))
			eraseOutlier(matchItr->first);

		KeyFrame* pKF = dynamic_cast<KeyFrame*>(this);
		if (pKF)
			matchItr->first->eraseKFObservation(pKF);

		if (_matchedMptMap.size() - _outLiers.size() < 4)
		{
			setBad(true);
			_globalMap->insertBadFrame(this);
			break;
		}
	}
}



bool Frame::eraseMptMatched(MapPoint3d* pMP)
{
	std::unique_lock<std::mutex> lock(_matchedMutex);
	auto matchItr = _matchedMptMap.find(pMP);
	if (matchItr != _matchedMptMap.end())
	{
		matchItr->second.first->setMatchedMpt(nullptr);
		matchItr->second.second->setMatchedMpt(nullptr);
		_matchedMptMap.erase(matchItr);	

		pMP->eraseObservation(this);
		KeyFrame* pKF = dynamic_cast<KeyFrame*>(this);
		if (pKF)
			pMP->eraseKFObservation(pKF);

		if (getOutLiers().count(pMP))
			eraseOutlier(pMP);

		if (_matchedMptMap.size() - _outLiers.size() < 4)
		{
			setBad(true);
			_globalMap->insertBadFrame(this);
		}	
		
	}
	else
	{
		fprintf(stdout, "Failed erasing map point matched in frame %d",_id);
		return false;
	}
	return true;
}


void Frame::clearMatchedMpts()
{
	std::unique_lock<std::mutex> lock(_matchedMutex);
	for (auto matchItr = _matchedMptMap.begin(); matchItr != _matchedMptMap.end(); )
	{
		matchItr->second.first->setMatchedMpt(nullptr);
		matchItr->second.second->setMatchedMpt(nullptr);

		matchItr->first->eraseObservation(this);
		KeyFrame* pKF = dynamic_cast<KeyFrame*>(this);
		if (pKF)
			matchItr->first->eraseKFObservation(pKF);
		matchItr = _matchedMptMap.erase(matchItr);
	}
	clearOutLiers();//同时清空所有离散匹配地图点
}

double Frame::calcMptMatchedError(MapPoint3d* mpt)
{
	if (mpt->isBad())
		return -1;//跳过地图坏点
	cv::Mat x3Dw = cv::Mat(*mpt);
	cv::Mat x3Dc = _Rcw*x3Dw + _tcw;
	cv::Point3d* p3D = getP3DFromMpt(mpt);
	if (!p3D)
		return -1;
	double errX = x3Dc.at<double>(0) - p3D->x;
	double errY = x3Dc.at<double>(1) - p3D->y;
	double errZ = x3Dc.at<double>(2) - p3D->z;

	double p3DError = sqrt(errX*errX + errY*errY + errZ*errZ);
	return p3DError;
}



double Frame::calcMptMatchedError(cv::Point3d* p3D, MapPoint3d* mpt)
{
	if (mpt->isBad())
		return -1;//跳过地图坏点
	cv::Mat x3Dw = cv::Mat(*mpt);
	cv::Mat x3Dc = _Rcw*x3Dw + _tcw;

	double errX = x3Dc.at<double>(0) - p3D->x;
	double errY = x3Dc.at<double>(1) - p3D->y;
	double errZ = x3Dc.at<double>(2) - p3D->z;

	double p3DError = sqrt(errX*errX + errY*errY + errZ*errZ);
	return p3DError;
}








double Frame::calcEnergyValue(const Feature::DotMarker& marker1, const Feature::DotMarker& marker2,
							  Feature::MarkerList& markList1, Feature::MarkerList& markList2, 
						      cv::Mat F, double radius, int cameraID)
{
	double energy = 0.0;
	int phi = 0.0;
	Feature::MarkerList mark1Neighborhood, mark2Neighborhood;
	mark1Neighborhood = seekNeighborMarks(marker1, markList1, radius);
	mark2Neighborhood = seekNeighborMarks(marker2, markList2, radius);

	for (Feature::MarkerList::iterator itr1 = mark1Neighborhood.begin(); itr1 != mark1Neighborhood.end(); itr1++)
	{
		Feature::DotMarker& marker1 = *itr1;
		cv::Mat x2D1 = (cv::Mat_<double>(2, 1) << marker1.undistortX, marker1.undistortY);

		cv::Mat line;
		cv::computeCorrespondEpilines(x2D1.t(), cameraID, F, line);
		line = line.reshape(1, 3);
		const double& a = line.at<double>(0);
		const double& b = line.at<double>(1);
		const double& c = line.at<double>(2);

		for (Feature::MarkerList::iterator itr2 = mark2Neighborhood.begin(); itr2 != mark2Neighborhood.end(); itr2++)
		{
			Feature::DotMarker& marker2 = *itr2;
			double dist = abs(a*marker2.undistortX + b*marker2.undistortY + c) / sqrt(a*a + b*b);
			phi = dist > MaxEpilineDist ? 0 : 1;
			energy += phi;
		}
	}

	energy /= (mark1Neighborhood.size()*mark2Neighborhood.size());
	return energy;
}



Feature::MarkerList Frame::seekNeighborMarks(Feature::DotMarker marker, Feature::MarkerList markList1, double radius)
{
	Feature::MarkerList markNeighborhood;
	double x1 = marker.undistortX;
	double y1 = marker.undistortY;

	for (Feature::MarkerList::iterator itr = markList1.begin(); itr != markList1.end(); itr++)
	{
		Feature::DotMarker marker2 = *itr;
		double x2 = marker2.undistortX;
		double y2 = marker2.undistortY;

		double dist = powf((x2 - x1), 2) + powf((y2 - y1), 2);
		dist = sqrtf(dist);
		if (dist<radius)
		{
			markNeighborhood.emplace_back(marker2);
		}
	}

	return markNeighborhood;
}



double Frame::calcMatchedRMSE()
{
	std::unique_lock<std::mutex> lock(Frame::_matchedMutex);

	double RMSE = 0;
	if (isBad())
		return -1.;
	if (_matchedMptMap.empty())
		return 0.;
	cv::Mat x3Dw;
	cv::Mat x3Dc;
	for (auto mItr = _matchedMptMap.begin(); mItr != _matchedMptMap.end(); mItr++)
	{
		if (mItr->first->isBad())
			continue;//跳过地图坏点
		if (_outLiers.count(mItr->first))
			continue;
		x3Dw = cv::Mat(*mItr->first);
		x3Dc = _Rcw*x3Dw +_tcw;
		cv::Point3d* p3D = mItr->second.first->_matchedP3D;
		double errX = x3Dc.at<double>(0) - p3D->x;
		double errY = x3Dc.at<double>(1) - p3D->y;
		double errZ = x3Dc.at<double>(2) - p3D->z;

		double p3DError = errX*errX + errY*errY + errZ*errZ;
		RMSE += p3DError;
	}
	return sqrt(RMSE / _matchedMptMap.size());
}



bool Frame::checkMatchedExtend()
{
#ifdef STAGE
	int quadrantThres = 2;
	double areaThres = 0.1;
#else
	int quadrantThres = 3;
	double areaThres = 0.4;
#endif

	double xMin(DBL_MAX), xMax(DBL_MIN), yMin(DBL_MAX), yMax(DBL_MIN);
	std::vector<cv::Point> markersList;
	for (auto itr = _p3DMap.begin(); itr != _p3DMap.end(); itr++)
	{
		KeyPoint* kptLeft = itr->second.first;
		if (kptLeft->x < xMin)
			xMin = kptLeft->x;
		if (kptLeft->y < yMin)
			yMin = kptLeft->y;
		if (kptLeft->x > xMax)
			xMax = kptLeft->x;
		if (kptLeft->y > yMax)
			yMax = kptLeft->y;
		markersList.emplace_back(dynamic_cast<cv::Point2d&>(*kptLeft));
	}
	std::vector<cv::Point> visConvexHull;
	cv::convexHull(cv::Mat(markersList), visConvexHull, false);
	double visArea = cv::contourArea(visConvexHull);

	int OriX = (xMin + xMax) / 2;
	int OriY = (yMin + yMax) / 2;

	xMin = DBL_MAX, xMax = DBL_MIN, yMin = DBL_MAX, yMax = DBL_MIN;
	std::vector<cv::Point> markerMatchedList;
	for (auto itr = _matchedMptMap.begin(); itr != _matchedMptMap.end(); itr++)
	{
		KeyPoint* kptLeft = itr->second.first;
		if (kptLeft->x < xMin)
			xMin = kptLeft->x;
		if (kptLeft->y < yMin)
			yMin = kptLeft->y;
		if (kptLeft->x > xMax)
			xMax = kptLeft->x;
		if (kptLeft->y > yMax)
			yMax = kptLeft->y;
		markerMatchedList.emplace_back(dynamic_cast<cv::Point2d&>(*kptLeft));
	}
	std::vector<cv::Point> matchedConvexHull;
	cv::convexHull(cv::Mat(markerMatchedList), matchedConvexHull, false);
	double matchedArea = cv::contourArea(matchedConvexHull);

	if ((ceil(xMin) <OriX) + (ceil(yMin) < OriY) + (floor(xMax) > OriX) + (floor(yMax) > OriY) >= quadrantThres &&
		matchedArea / visArea>areaThres)
		return true;

	return false;
}



}