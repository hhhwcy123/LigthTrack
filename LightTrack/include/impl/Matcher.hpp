#include "Matcher.h"


namespace SLAM
{


//3d-3d三角形匹配
#ifdef STAGE 
MatcherTemplate
const double Matcher_<T1, T2, Tp>::SITE_DIFF_THRES = 0.5;
MatcherTemplate
const double Matcher_<T1, T2, Tp>::P3D_MATCH_THRES = 10.;
MatcherTemplate
const double Matcher_<T1, T2, Tp>::MAX_PTBIAS_ERROR = 0.30;
MatcherTemplate
const double Matcher_<T1, T2, Tp>::TRIANGLEBIAS_THRES = 2;
#else
MatcherTemplate
const double Matcher_<T1, T2, Tp>::SITE_DIFF_THRES = 0.1;
MatcherTemplate
const double Matcher_<T1, T2, Tp>::P3D_MATCH_THRES = 10.;
MatcherTemplate
const double Matcher_<T1, T2, Tp>::MAX_PTBIAS_ERROR = 0.10;
MatcherTemplate
const double Matcher_<T1, T2, Tp>::TRIANGLEBIAS_THRES = 2;
#endif


MatcherTemplate
std::mutex Matcher_<T1, T2, Tp>::_matchedMutex;

MatcherTemplate
const typename Matcher_<T1, T2, Tp>::OrientFunc Matcher_<T1, T2, Tp>::_orientFuncs[] = { absoluteOrientation };

MatcherTemplate
const typename Matcher_<T1, T2, Tp>::CalcSim3Func Matcher_<T1, T2, Tp>::_calcSim3Funcs[] = { calcSim3Successive ,calcSim3RANSAC };

MatcherTemplate
const typename Matcher_<T1, T2, Tp>::MatchedKeysFunc Matcher_<T1, T2, Tp>::_matchedKeysFuncs[] = { findMatchKeysByProj ,findMatchKeysByUnProj };



MatcherTemplate
bool Matcher_<T1, T2, Tp>::calcP3DDistList(const Frame& frame, T1DistMap& distMap)
{
	const T1KeyPairMap& p3DMap = frame._p3DMap;
	T1List p3DList;
	for (T1KeyPairMap::const_iterator itr = p3DMap.begin(); itr != p3DMap.end(); itr++)
	{
		p3DList.emplace_back(itr->first);
	}

	for (T1List::iterator itr = p3DList.begin(); itr != p3DList.end(); itr++)
	{
		for (T1List::iterator itrNext = itr + 1; itrNext != p3DList.end(); itrNext++)
		{
			T1<Tp> vec = **itr - **itrNext;
			double dist = cv::norm(vec);
			distMap.insert(std::make_pair(dist, std::make_pair(*itr, *itrNext)));
			//distList[dist]=std::make_pair(*itr, *itrNext);
		}
	}
	return true;
}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::calcP3DDistList(const T1List& p3DList, T1DistMap& distList, double th)
{
	for (T1List::const_iterator itr = p3DList.begin(); itr != p3DList.end(); itr++)
	{
		for (T1List::const_iterator itrNext = itr + 1; itrNext != p3DList.end(); itrNext++)
		{
			T1<Tp> vec = **itr - **itrNext;
			double dist = cv::norm(vec);
			if (dist < th)
				distList.insert(std::make_pair(dist, std::make_pair(*itr, *itrNext)));
		}
	}
	return true;
}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::findMatchedP3DSite(T2PairList& condidateObjSiteList, T2DistMap& objSiteLenMap,
											  const T1Pair& srcSite, double th)
{
	double matchedLen = cv::norm(*srcSite.first - *srcSite.second);

	T2DistMap::iterator startItr = objSiteLenMap.lower_bound(matchedLen - th);
	T2DistMap::iterator endItr = objSiteLenMap.upper_bound(matchedLen + th);

	if (startItr == objSiteLenMap.end() || endItr == objSiteLenMap.begin())
		return false;

	for (T2DistMap::iterator itr = startItr; itr != endItr; itr++)
	{
		condidateObjSiteList.emplace_back(itr->second);
	}

	if (condidateObjSiteList.empty())
		return false;
	return true;
}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::findMatchedP3DSite(T2Pair& objSite, T2DistMap& objSiteLenMap,
											  const T1Pair& srcSite, double th)
{
	double matchedLen = cv::norm(*srcSite.first - *srcSite.second);

	double minDiff = DBL_MAX;
	bool bFound = false;
	std::pair<double, T2Pair>  bestSite;

	for each(std::pair<double, T2Pair> tmpSite in objSiteLenMap)
	{
		double tmpDiff = abs(tmpSite.first - matchedLen);

		if (tmpDiff < minDiff)
		{
			minDiff = tmpDiff;
			bestSite = tmpSite;
		}

	}
	if (minDiff < th)
	{
		objSite = bestSite.second;
		return true;
	}
	return false;
}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::matchP3DByFrame(Frame& prevFrame, Frame& curFrame,
										   T1T2PairList& matchedP3DList, cv::Mat& R, cv::Mat& t,
										   const bool& bDispErrInfo, OrientFunc orientFunc,
										   const double maxP3dDist, const double matchedRatio)
{
	T1List p3dListCur;
	T2List p3dListPrev;

	std::unique_lock<std::mutex> lockMatched(_matchedMutex);//当前帧多线程跟踪锁
	T1KeyPairMap& p3dMapCur = curFrame._p3DMap;
	for (T1KeyPairMap::iterator itrCur = p3dMapCur.begin(); itrCur != p3dMapCur.end(); itrCur++)
	{
		p3dListCur.emplace_back(itrCur->first);
	}
	lockMatched.unlock();

	T2KeyPairMap& p3dMapPrev = prevFrame._p3DMap;
	for (T2KeyPairMap::iterator itrPrev = p3dMapPrev.begin(); itrPrev != p3dMapPrev.end(); itrPrev++)
	{
		MapPoint_<Tp>* matchedMpt = dynamic_cast<MapPoint_<Tp>*>(itrPrev->second.first->getMatchedMpt());
		//assert(matchedMpt);
		if (matchedMpt)
			if (matchedMpt->isBad())
				continue;//跳过暂时未删除的地图坏点
		p3dListPrev.emplace_back(itrPrev->first);
	}
	T2DistMap siteLenMapPrev;
	calcP3DDistList(p3dListPrev, siteLenMapPrev, maxP3dDist);

	for (T1List::iterator itrCur1 = p3dListCur.begin(); itrCur1 != p3dListCur.end(); itrCur1++)
	{
		for (T1List::iterator itrCur2 = itrCur1 + 1; itrCur2 != p3dListCur.end(); itrCur2++)
		{
			T2PairList p3dCondidateSiteList;
			T1Pair p3dSiteCur = std::make_pair(*itrCur1, *itrCur2);
			if (!findMatchedP3DSite(p3dCondidateSiteList, siteLenMapPrev, p3dSiteCur, SITE_DIFF_THRES))
				continue;

			for (T1List::iterator itrCur3 = itrCur2 + 1; itrCur3 != p3dListCur.end(); itrCur3++)
			{
				int nMatchedMax = INT_MIN;
				bool bMatched = false;
				for (T2PairList::iterator prevSiteItr = p3dCondidateSiteList.begin(); prevSiteItr != p3dCondidateSiteList.end(); prevSiteItr++)
				{
					T1T2PairList p3DMatchedList;
					if (!findMatchedTriVertexByFrame(*itrCur1, *itrCur2, *itrCur3,
						*prevSiteItr, siteLenMapPrev,
						p3DMatchedList, Matcher_::SITE_DIFF_THRES))
						continue;

					std::set<T1<Tp>*> curP3DTriSet;
					std::set<T2<Tp>*> prevP3DTriSet;
					cv::Mat curMat = cv::Mat(3, 3, CV_64F);
					cv::Mat prevMat = cv::Mat(3, 3, CV_64F);
					int c = 0;
					for (T1T2PairList::iterator matchedItr = p3DMatchedList.begin(); matchedItr != p3DMatchedList.end(); matchedItr++, c++)
					{
						curP3DTriSet.insert(matchedItr->first);
						prevP3DTriSet.insert(matchedItr->second);
						assert(dynamic_cast<cv::Point3_<Tp>*>(matchedItr->first) && dynamic_cast<cv::Point3_<Tp>*>(matchedItr->second));
						cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(*matchedItr->first)).copyTo(curMat.col(c));
						cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(*matchedItr->second)).copyTo(prevMat.col(c));
					}

					cv::Mat RTmp, tTmp;
					double scale = 1.0;
					orientFunc(curMat, prevMat, RTmp, tTmp, scale, false);//根据当前空间匹配三角形计算帧-帧相对位置

					T1T2PairList matchedP3DListTmp;

					if (evaluateRt(p3dListCur, p3dListPrev, curP3DTriSet, prevP3DTriSet, RTmp, tTmp, matchedP3DListTmp, matchedRatio))//根据当前匹配三角形寻找更多3d匹配对
					{
						///////////////////////根据当前三角形所得最优匹配点集计算R，t/////////////////
						matchedP3DListTmp.insert(matchedP3DListTmp.end(), p3DMatchedList.begin(), p3DMatchedList.end());
						const int& nMatched = matchedP3DList.size();
						if (nMatched > nMatchedMax)
						{
							nMatchedMax = nMatched;
							bMatched = true;
							R = RTmp;
							t = tTmp;
							matchedP3DList = matchedP3DListTmp;
						}
					}
				}
				if (bMatched)
					return true;
			}
		}
	}
	return false;
}


MatcherTemplate
bool Matcher_<T1, T2, Tp>::matchP3DByGlobalMap(GlobalMap* globalMap, Frame& curFrame,
											   T1T2PairList& p3dMptPairList,
											   cv::Mat& R, cv::Mat& t, OrientFunc orientFunc,
											   const bool& bDispErrInfo, const double matchedRatio)
{
	std::unique_lock<std::mutex> lockMatched(_matchedMutex);//当前帧多线程跟踪锁

	T1List p3dListCur;
	T1KeyPairMap& p3dMapCur = curFrame._p3DMap;
	for (T1KeyPairMap::iterator itrCur = p3dMapCur.begin(); itrCur != p3dMapCur.end(); itrCur++)
		p3dListCur.emplace_back(itrCur->first);

	for (T1List::iterator itrCur1 = p3dListCur.begin(); itrCur1 != p3dListCur.end(); itrCur1++)
	{
		for (T1List::iterator itrCur2 = itrCur1 + 1; itrCur2 != p3dListCur.end(); itrCur2++)
		{

			T2PairList mptCondidateSiteList;
			//MptPair mptSite;
			T1Pair p3dSiteCur = std::make_pair(*itrCur1, *itrCur2);
			if (!findMatchedP3DSite(mptCondidateSiteList, globalMap->getMptDistMap(), p3dSiteCur, 2.0*SITE_DIFF_THRES))
				continue;
			for (T1List::iterator itrCur3 = itrCur2 + 1; itrCur3 != p3dListCur.end(); itrCur3++)
			{
				int nMatchedMax = INT_MIN;
				bool bMatched = false;
				for (T2PairList::iterator mptSiteItr = mptCondidateSiteList.begin(); mptSiteItr != mptCondidateSiteList.end(); mptSiteItr++)
				{//mptSiteItr：与p3dSiteCur匹配的全局地图边
					if (mptSiteItr->first->isBad())
						continue;
					if (mptSiteItr->second->isBad())
						continue;
					T1T2MatchedMap tmpP3dMptPairList;
					if (!findMatchedTriVertexByGlobalMap(*itrCur1, *itrCur2, *itrCur3, *mptSiteItr, tmpP3dMptPairList, 2.0*SITE_DIFF_THRES))
						continue;

					std::vector<T1<Tp>> curP3DTriList, globalP3DTriList;
					for (T1T2MatchedMap::iterator matchedItr = tmpP3dMptPairList.begin(); matchedItr != tmpP3dMptPairList.end(); matchedItr++)
					{
						cv::Point3_<Tp>* p3DCur = dynamic_cast<cv::Point3_<Tp>*>(matchedItr->first);
						cv::Point3_<Tp>* p3DGlobal = dynamic_cast<cv::Point3_<Tp>*>(matchedItr->second);
						assert(p3DCur&&p3DGlobal);
						globalP3DTriList.emplace_back(*p3DGlobal);
						curP3DTriList.emplace_back(*p3DCur);
					}

					cv::Mat curMat = cv::Mat(curP3DTriList).reshape(1, 3).t();
					cv::Mat refMat = cv::Mat(globalP3DTriList).reshape(1, 3).t();
					cv::Mat RTmp, tTmp;
					double scale = 1.0;
					orientFunc(curMat, refMat, RTmp, tTmp, scale, false);

					T1T2PairList matchedListTmp;
					if (evaluateRt(p3dListCur, globalMap->getMapPointList(), RTmp, tTmp, matchedListTmp))
					{
						const int& nMatched = matchedListTmp.size();
						if (nMatched > nMatchedMax)
						{
							nMatchedMax = nMatched;
							bMatched = true;
							R = RTmp;
							t = tTmp;
							p3dMptPairList = matchedListTmp;
						}
					}
				}
				if (bMatched)
				{
					return true;
				}
			}
		}
	}
	return false;
}


MatcherTemplate
bool Matcher_<T1, T2, Tp>::calcSim3Successive(T1T2PairList& matchedP3DList, T1T2PairList& outLiers,
											  cv::Mat& R, cv::Mat& t,
											  OrientFunc orientfunc, const bool& bDispErrorInfo)
{
	int nMatched = matchedP3DList.size();
	if (nMatched < 4)
		return false;
	cv::Mat curMat = cv::Mat(3, nMatched, CV_64F);
	cv::Mat prevMat = cv::Mat(3, nMatched, CV_64F);

	int c = 0;
	for (T1T2PairList::iterator itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++, c++)
	{
		assert(dynamic_cast<cv::Point3_<Tp>*>(itr->first) && dynamic_cast<cv::Point3_<Tp>*>(itr->second));
		cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(*itr->first)).copyTo(curMat.col(c));
		cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(*itr->second)).copyTo(prevMat.col(c));
	}
	if (checkCollinear(curMat))
		return false;
	double error2 = 0;
	cv::Mat errorMat;
	double RMSE = 0;
	double errorSum2 = 0;
	double errorMax = DBL_MIN;
	int worstIndex = -1;
	double scale = 1.0;

	while (nMatched > 3)
	{
		error2 = 0;
		RMSE = 0;
		errorSum2 = 0;
		errorMax = DBL_MIN;
		worstIndex = -1;

		orientfunc(curMat, prevMat, R, t, scale, false);

		for (int i = 0; i < nMatched; i++)
		{
			errorMat = curMat.col(i) - (R*prevMat.col(i) + t);
			error2 = errorMat.dot(errorMat);
			if (error2 > errorMax)
			{
				errorMax = error2;
				worstIndex = i;
			}
			if (bDispErrorInfo)
				std::cout << "Matched P3D " << i << " Error: " << sqrt(error2) << std::endl;
			errorSum2 += error2;
		}
		RMSE = sqrt(errorSum2 / nMatched);
		if (bDispErrorInfo)
			std::cout << "Matched RMSE: " << RMSE << std::endl;
		if (RMSE > MAX_PTBIAS_ERROR)
		{
			nMatched--;
			cv::Mat curMatTmp = cv::Mat(3, nMatched, CV_64F);
			cv::Mat prevMatTmp = cv::Mat(3, nMatched, CV_64F);
			for (int i = 0, j = 0; i < curMat.cols; i++)
			{
				if (i == worstIndex)
					continue;
				curMat.col(i).copyTo(curMatTmp.col(j));
				prevMat.col(i).copyTo(prevMatTmp.col(j));
				j++;
			}
			if (checkCollinear(curMatTmp))
				return false;//如果剩余3d点共线，返回false

			curMat = curMatTmp.clone();
			prevMat = prevMatTmp.clone();

			outLiers.emplace_back(*(matchedP3DList.begin() + worstIndex));
			//matchedP3DList.erase(matchedP3DList.begin() + worstIndex);

			std::cout << "Remove matched P3D: " << worstIndex
				<< ", Compute Sim3 again..." << std::endl;
		}
		else
			return true;
	}

	return false;
}




/******************************************
RANSAC匹配对求解帧-帧相对运动
******************************************/
MatcherTemplate
bool Matcher_<T1, T2, Tp>::calcSim3RANSAC(T1T2PairList& matchedP3DList, T1T2PairList& outLiers,
										  cv::Mat& R, cv::Mat& t,
										  OrientFunc orientfunc, const bool& bDispErrorInfo)
{
	int nMatched = matchedP3DList.size();
	if (nMatched < 4)
		return false;

	cv::Mat curMat(3, nMatched, CV_64F);
	cv::Mat prevMat(3, nMatched, CV_64F);
	cv::Mat curMatTmp(3, 3, CV_64F);
	cv::Mat prevMatTmp(3, 3, CV_64F);

	int c = 0;

	double RMSE = 0.;
	double error2 = 0.;
	cv::Mat errorMat;
	double totalError2 = 0.;
	double scale = 1.0;
	int nInLiers = 0.;
	std::set<int> inlierIndices;
	int nIterations = 0;


	///////////////////初始定向///////////////////
	for (auto itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++, c++)
	{
		cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(*itr->first)).copyTo(curMat.col(c));
		cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(*itr->second)).copyTo(prevMat.col(c));
	}
	if (checkCollinear(curMat))
		return false;
	orientfunc(curMat, prevMat, R, t, scale, false);
	for (int i = 0; i < nMatched; i++)
	{
		errorMat = curMat.col(i) - (R*prevMat.col(i) + t);
		error2 = errorMat.dot(errorMat);

		if (bDispErrorInfo)
			std::cout << "Matched P3D " << i << " Error: " << sqrt(error2) << std::endl;
		totalError2 += error2;
	}
	RMSE = sqrt(totalError2 / nMatched);
	if (bDispErrorInfo)
		std::cout << "Matched Prev P3D RMSE: " << RMSE << std::endl;
	if (RMSE < MAX_PTBIAS_ERROR)//RMSE过大可能因为存在误匹配对，通过RANSAC筛除
		return true;
	else if (nMatched == 4)
		return false;
	//////////RANSAC匹配定向//////////////
	std::vector<size_t> matchedIndices;
	matchedIndices.reserve(nMatched);
	for (int i = 0; i < nMatched; i++)
		matchedIndices.emplace_back(i);
	//int maxIterations = nMatched == 3 ? 1 : 4;
	int maxIterations = 4;

	while (nIterations < maxIterations)
	{
		nIterations++;
		std::cout << "RANSAC Sim3 ... " << std::endl;

		std::vector<size_t> availableIndices = matchedIndices;
		for (short i = 0; i < 3; ++i)
		{
			int randi = RandomInt(0, availableIndices.size() - 1);
			int idx = availableIndices[randi];

			curMat.col(idx).copyTo(curMatTmp.col(i));
			prevMat.col(idx).copyTo(prevMatTmp.col(i));

			availableIndices[randi] = availableIndices.back();
			availableIndices.pop_back();
		}
		if (checkCollinear(curMatTmp))
			continue;//如果随机匹配点共线	
		orientfunc(curMatTmp, prevMatTmp, R, t, scale, false);

		error2 = 0.;
		totalError2 = 0.;
		inlierIndices.clear();
		for (int i = 0; i < nMatched; i++)
		{
			errorMat = curMat.col(i) - (R*prevMat.col(i) + t);
			error2 = errorMat.dot(errorMat);
			if (bDispErrorInfo)
				std::cout << "Matched P3D " << i << " Error: " << sqrt(error2) << std::endl;
			if (sqrt(error2) < MAX_PTBIAS_ERROR)
			{
				inlierIndices.insert(i);
				totalError2 += error2;
			}
		}
		nInLiers = inlierIndices.size();
		if (nInLiers < 4)
			continue;
		RMSE = sqrt(totalError2 / nInLiers);

		if (nInLiers > 3 && RMSE < MAX_PTBIAS_ERROR)
		{
			cv::Mat curInliers(3, nInLiers, CV_64F);
			cv::Mat prevInliers(3, nInLiers, CV_64F);
			int c = 0;
			for (auto indItr = inlierIndices.begin(); indItr != inlierIndices.end(); indItr++, c++)
			{
				if (bDispErrorInfo)
					std::cout << "Inliers index: " << *indItr << std::endl;
				curMat.col(*indItr).copyTo(curInliers.col(c));
				prevMat.col(*indItr).copyTo(prevInliers.col(c));
			}
			/*std::vector<cv::Point3_<Tp>> p3dListCur;
			for (int i = 0, j = 0; i < curInliers.cols; i++)
			{
			p3dListCur.emplace_back(cv::Point3_<Tp>(curInliers.at<double>(0, i), curInliers.at<double>(1, i), curInliers.at<double>(2, i)));
			}
			if (!checkCollinear(p3dListCur))
			{
			orientfunc(curInliers, prevInliers, R, t, scale, false);
			return true;
			}*/

			orientfunc(curInliers, prevInliers, R, t, scale, false);

			int count = 0;
			for (int i = 0; i < nMatched; i++)
			{
				if (!inlierIndices.count(i))
				{
					outLiers.emplace_back(*(matchedP3DList.begin() + (i - count)));
					matchedP3DList.erase(matchedP3DList.begin() + (i - count++));
				}
			}
			return true;
		}
		/*if (nInLiers <= 4 || nInliersMax >= 3)
		return true;*/
	}
	return false;
}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::checkCollinear(cv::Mat& p3dMat)
{
	if (p3dMat.cols < 3)
		return false;

	T1<Tp> A = T1<Tp>(p3dMat.at<double>(0, 0), p3dMat.at<double>(1, 0), p3dMat.at<double>(2, 0));
	T1<Tp> B = T1<Tp>(p3dMat.at<double>(0, 1), p3dMat.at<double>(1, 1), p3dMat.at<double>(2, 1));

	for (int i = 2; i < p3dMat.cols; i++)
	{
		T1<Tp> C = T1<Tp>(p3dMat.at<double>(0, i), p3dMat.at<double>(1, i), p3dMat.at<double>(2, i));
		T1<Tp> CA = A - C;
		T1<Tp> CB = B - C;
		T1<Tp> AB = B - A;
		double cosACB = CA.dot(CB) / (cv::norm(CA)*cv::norm(CB));
		if (1 - abs(cosACB) > 0.1)
			return false;
		double cosCAB = CA.dot(AB) / (cv::norm(CA)*cv::norm(AB));
		if (1 - abs(cosCAB) > 0.1)
			return false;
		double cosCBA = CB.dot(AB) / (cv::norm(CB)*cv::norm(AB));
		if (1 - abs(cosCBA) > 0.1)
			return false;
	}
	std::cout << p3dMat.cols << " Markers are collinear" << std::endl;
	return true;
}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::checkCollinear(std::vector<T1<Tp>>& p3dList)
{
	if (p3dList.size() < 3)
		return false;
	T1<Tp> A = p3dList[0];
	T1<Tp> B = p3dList[1];
	int size = p3dList.size();

	for (int i = 2; i < size; i++)
	{
		T1<Tp> C = p3dList[i];
		T1<Tp> CA = A - C;
		T1<Tp> CB = B - C;
		T1<Tp> AB = B - A;
		double cosACB = CA.dot(CB) / (cv::norm(CA)*cv::norm(CB));
		if (1 - abs(cosACB) > 0.1)
			return false;
		double cosCAB = CA.dot(AB) / (cv::norm(CA)*cv::norm(AB));
		if (1 - abs(cosCAB) > 0.1)
			return false;
		double cosCBA = CB.dot(AB) / (cv::norm(CB)*cv::norm(AB));
		if (1 - abs(cosCBA) > 0.1)
			return false;
	}
	std::cout << size << " marks are collinear" << std::endl;
	return true;
}






/*************************************************
功能：计算全局地图点与当前帧三角形匹配的顶点匹配对
p3DCur1：当前帧三角形顶点1
p3DCur2：当前帧三角形顶点2
p3DCur3：当前帧三角形顶点3
mptSite：全局地图与当前帧三角形已匹配上的边
mptDistMap:全局地图的所有边，进行红黑树搜索
p3DPairList：返回值，全局地图点与当前帧三角形3d点匹配对集合
*****************************************************/
MatcherTemplate
bool Matcher_<T1, T2, Tp>::findMatchedTriVertexByGlobalMap(T1<Tp>* p3DCur1, T1<Tp>* p3DCur2, T1<Tp>* p3DCur3,
														   T2Pair& mptSite,
														   T1T2MatchedMap& p3dMptPairList, double th)
{
	T2<Tp>* objMpt1 = nullptr;
	T2<Tp>* objMpt2 = nullptr;
	T2<Tp>* objMpt3 = nullptr;

	if (p3DCur3 == p3DCur1 || p3DCur3 == p3DCur2)
		return false;

	double len13 = cv::norm(*p3DCur1 - *p3DCur3);

	for (int i = 0; i < 2; i++)
	{
		if (i == 0)
		{
			objMpt1 = mptSite.first;
			objMpt2 = mptSite.second;
			objMpt3 = nullptr;
		}
		else if (i == 1)
		{
			objMpt1 = mptSite.second;
			objMpt2 = mptSite.first;
			objMpt3 = nullptr;
		}

		assert(dynamic_cast<MapPoint_<Tp>*>(objMpt1) && dynamic_cast<MapPoint_<Tp>*>(objMpt2));

		T2DistMap::iterator startItr = objMpt1->getConnSiteMap().lower_bound(len13 - th);
		T2DistMap::iterator endItr = objMpt1->getConnSiteMap().upper_bound(len13 + th);

		if (startItr == objMpt1->getConnSiteMap().end() || endItr == objMpt1->getConnSiteMap().begin())
			return false;


		bool bMatchedOK = false;
		double minDiff = DBL_MAX;
		for (T2DistMap::iterator connItr = startItr; connItr != endItr; connItr++)
		{//遍历地图点objMpt1的连接边
			if (connItr->second == mptSite)
				continue;
			double tmpDiff = abs(connItr->first - len13);
			if (tmpDiff < minDiff)
			{
				objMpt3 = connItr->second.second;
				if (objMpt3->isBad())
					continue;
				if (abs(cv::norm(*objMpt3 - *objMpt2) - cv::norm(*p3DCur3 - *p3DCur2)) < th)
					bMatchedOK = true;
			}
		}
		if (bMatchedOK)
		{
			p3dMptPairList[p3DCur1] = objMpt1;
			p3dMptPairList[p3DCur2] = objMpt2;
			p3dMptPairList[p3DCur3] = objMpt3;
			return true;
		}
	}
	return false;
}


/*************************************************
功能：返回前帧中与当前帧指定三角形匹配的三角形顶点匹配对
p3DCur1：当前帧三角形顶点1
p3DCur2：当前帧三角形顶点2
p3DCur3：当前帧三角形顶点3
p3dSitePrev：前帧与当前帧三角形已匹配上的边(对应顶点1、顶点2)
siteLenMapPrev：前帧3d点集中所有边的集合，供搜索
p3DPairList：返回值，前帧与当前帧三角形顶点匹配对集合
*****************************************************/
MatcherTemplate
bool Matcher_<T1, T2, Tp>::findMatchedTriVertexByFrame(T1<Tp>* p3DCur1, T1<Tp>* p3DCur2, T1<Tp>* p3DCur3,
													   T2Pair& p3dSitePrev12, T2DistMap& siteLenMapPrev,
													   T1T2PairList& p3DMptPairList, double th)
{
	T2<Tp>* p3DPrev1 = p3dSitePrev12.first;
	T2<Tp>* p3DPrev2 = p3dSitePrev12.second;
	T2<Tp>* p3DPrev3 = nullptr;

	if (p3DCur3 == p3DCur1 || p3DCur3 == p3DCur2)
		return false;

	T2Pair p3dSitePrev13;
	double lenCur13 = cv::norm(*p3DCur1 - *p3DCur3);
	T2DistMap::iterator startItr = siteLenMapPrev.lower_bound(lenCur13 - th);
	T2DistMap::iterator endItr = siteLenMapPrev.upper_bound(lenCur13 + th);

	if (startItr == siteLenMapPrev.end() || endItr == siteLenMapPrev.begin())
		return false;

	bool bMatchedOK = false;
	double minDiff13 = DBL_MAX;//前帧1、3顶点与当前帧1、3顶点距离差
	for (T2DistMap::iterator itr13 = startItr; itr13 != endItr; itr13++)
	{
		p3dSitePrev13 = itr13->second;
		if (p3dSitePrev13 == p3dSitePrev12)
			continue;

		double tmpDiff13 = abs(itr13->first - lenCur13);
		if (tmpDiff13 < minDiff13)
		{
			p3DPrev1 = findVertex(p3dSitePrev13, p3dSitePrev12);
			if (!p3DPrev1)
			{
				p3DPrev1 = p3dSitePrev12.first;
				continue;
			}
			minDiff13 = tmpDiff13;
			if (p3DPrev1 == p3dSitePrev12.first)
				p3DPrev2 = p3dSitePrev12.second;
			else
				p3DPrev2 = p3dSitePrev12.first;

			if (p3DPrev1 == p3dSitePrev13.first)
				p3DPrev3 = p3dSitePrev13.second;
			else
				p3DPrev3 = p3dSitePrev13.first;
			double diff23 = abs(cv::norm(*p3DPrev2 - *p3DPrev3) - cv::norm(*p3DCur2 - *p3DCur3));
			if (diff23 < th)
			{//diff23：前帧2、3顶点与当前帧2、3顶点距离差
				bMatchedOK = true;
			}

		}
	}
	if (bMatchedOK)
	{
		p3DMptPairList.emplace_back(p3DCur1, p3DPrev1);
		p3DMptPairList.emplace_back(p3DCur2, p3DPrev2);
		p3DMptPairList.emplace_back(p3DCur3, p3DPrev3);
		return true;
	}
	return false;

}




MatcherTemplate
T1<Tp>* Matcher_<T1, T2, Tp>::findVertex(const T1Pair& p3DPair1, const T1Pair& p3DPair2)
{
	if (p3DPair1.first == p3DPair2.first || p3DPair1.first == p3DPair2.second)
	{
		return p3DPair1.first;
	}
	else if (p3DPair1.second == p3DPair2.first || p3DPair1.second == p3DPair2.second)
	{
		return p3DPair1.second;
	}
	else
	{
		return nullptr;
	}
}



//void Matcher_::absoluteOrientation(cv::Mat P1, cv::Mat P2, cv::Mat& R, cv::Mat& t, double& scale, bool bScale)
//{
//	cv::Mat meanVec1(P1.rows, 1, CV_64FC1, cv::Scalar(0));
//	cv::reduce(P1, meanVec1, 1, CV_REDUCE_AVG);
//
//	cv::Mat meanVec2(P2.rows, 1, CV_64FC1, cv::Scalar(0));
//	cv::reduce(P2, meanVec2, 1, CV_REDUCE_AVG);
//
//
//	cv::Mat mean1(3, P1.cols, CV_64F, .0), mean2(3, P2.cols, CV_64F, .0);
//	for (int c = 0; c < P1.cols; c++)
//		meanVec1.copyTo(mean1.col(c));
//	for (int c = 0; c < P2.cols; c++)
//		meanVec2.copyTo(mean2.col(c));
//
//
//
//	cv::Mat Pr1 = P1 - mean1;
//	cv::Mat Pr2 = P2 - mean2;
//
//	double a1, a2, a3, a4;
//	double b1, b2, b3, b4;
//	cv::Mat M(4, 4, CV_64F, cv::Scalar(0));
//	for (int c = 0; c < Pr1.cols; c++)
//	{
//		a1 = 0;
//		a2 = Pr1.at<double>(0, c);
//		a3 = Pr1.at<double>(1, c);
//		a4 = Pr1.at<double>(2, c);
//		b1 = 0;
//		b2 = Pr2.at<double>(0, c);
//		b3 = Pr2.at<double>(1, c);
//		b4 = Pr2.at<double>(2, c);
//
//		cv::Mat M1 = (cv::Mat_<double>(4, 4) << a1, -a2, -a3, -a4, a2, a1, a4, -a3, a3, -a4, a1, a2, a4, a3, -a2, a1);
//		cv::Mat M2 = (cv::Mat_<double>(4, 4) << b1, -b2, -b3, -b4, b2, b1, -b4, b3, b3, b4, b1, -b2, b4, -b3, b2, b1);
//		M += M1.t() *M2;
//	}
//	cv::Mat valueMat, vectorMat;
//	cv::eigen(M, valueMat, vectorMat);
//
//	double e1 = vectorMat.ptr<double>(0)[0];
//	double e2 = vectorMat.ptr<double>(0)[1];
//	double e3 = vectorMat.ptr<double>(0)[2];
//	double e4 = vectorMat.ptr<double>(0)[3];
//
//	cv::Mat M1 = (cv::Mat_<double>(4, 4) << e1, -e2, -e3, -e4, e2, e1, e4, -e3, e3, -e4, e1, e2, e4, e3, -e2, e1);
//	cv::Mat M2 = (cv::Mat_<double>(4, 4) << e1, -e2, -e3, -e4, e2, e1, -e4, e3, e3, e4, e1, -e2, e4, -e3, e2, e1);
//	R = M1.t()*M2;
//	R = R.rowRange(1, 4).colRange(1, 4);
//
//	double s = .0;
//	if (bScale)
//	{
//		double a(0), b(0);
//		for (int c = 0; c < P1.cols; c++)
//		{
//			a += Pr2.col(c).dot(R*P1.col(c));
//			b += Pr2.col(c).dot(Pr2.col(c));
//		}
//		s = b / a;
//	}
//	else
//		s = 1.0;
//
//	t = meanVec2 - s*R*meanVec1;
//
//}



MatcherTemplate
void Matcher_<T1, T2, Tp>::absoluteOrientation(cv::Mat &P1, cv::Mat &P2, cv::Mat& R12, cv::Mat& t12, double& scale, bool bScale)
{
	// Custom implementation of:
	// Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

	// Step 1: Centroid and relative coordinates

	cv::Mat Pr1(P1.size(), P1.type()); // Relative coordinates to centroid (set 1)
	cv::Mat Pr2(P2.size(), P2.type()); // Relative coordinates to centroid (set 2)
	cv::Mat O1(3, 1, Pr1.type()); // Centroid of P1
	cv::Mat O2(3, 1, Pr2.type()); // Centroid of P2

	computeCentroid(P1, Pr1, O1);
	computeCentroid(P2, Pr2, O2);

	// Step 2: Compute M matrix

	cv::Mat M = Pr2 * Pr1.t();

	// Step 3: Compute N matrix

	double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

	cv::Mat N(4, 4, P1.type());

	N11 = M.at<double>(0, 0) + M.at<double>(1, 1) + M.at<double>(2, 2);
	N12 = M.at<double>(1, 2) - M.at<double>(2, 1);
	N13 = M.at<double>(2, 0) - M.at<double>(0, 2);
	N14 = M.at<double>(0, 1) - M.at<double>(1, 0);
	N22 = M.at<double>(0, 0) - M.at<double>(1, 1) - M.at<double>(2, 2);
	N23 = M.at<double>(0, 1) + M.at<double>(1, 0);
	N24 = M.at<double>(2, 0) + M.at<double>(0, 2);
	N33 = -M.at<double>(0, 0) + M.at<double>(1, 1) - M.at<double>(2, 2);
	N34 = M.at<double>(1, 2) + M.at<double>(2, 1);
	N44 = -M.at<double>(0, 0) - M.at<double>(1, 1) + M.at<double>(2, 2);

	N = (cv::Mat_<double>(4, 4) << N11, N12, N13, N14,
		N12, N22, N23, N24,
		N13, N23, N33, N34,
		N14, N24, N34, N44);


	// Step 4: Eigenvector of the highest eigenvalue
	cv::Mat eval, evec;
	cv::eigen(N, eval, evec); //evec[0] is the quaternion of the desired rotation

	cv::Mat vec(1, 3, evec.type());
	(evec.row(0).colRange(1, 4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

												// Rotation angle. sin is the norm of the imaginary part, cos is the real part
	double ang = atan2(norm(vec), evec.at<double>(0, 0));

	vec = 2 * ang*vec / norm(vec); //Angle-axis representation. quaternion angle is the half

	R12.create(3, 3, P1.type());

	cv::Rodrigues(vec, R12); // computes the rotation matrix from angle-axis

								// Step 5: Rotate set 2
	cv::Mat P3 = R12 * Pr2;

	// Step 6: Scale
	if (bScale)
	{
		double nom = Pr1.dot(P3);
		cv::Mat aux_P3(P3.size(), P3.type());
		aux_P3 = P3;
		cv::pow(P3, 2, aux_P3);
		double den = 0;

		for (int i = 0; i < aux_P3.rows; i++)
		{
			for (int j = 0; j < aux_P3.cols; j++)
			{
				den += aux_P3.at<double>(i, j);
			}
		}
		scale = nom / den;
	}
	else
		scale = 1.0f;

	// Step 7: Translation
	t12.create(1, 3, P1.type());
	t12 = O1 - scale * R12*O2;
}



/*************************
根据已计算出的R，t在当前帧和前帧中匹配更多的3d点
p3dListCur：当前帧3d点集
p3dListPrev：前帧3d点集
R，t：当前帧相对于前帧的位姿R，t
matchedP3DList：两帧匹配的3d点对集合
*************************/
MatcherTemplate
bool Matcher_<T1, T2, Tp>::evaluateRt(const T1List& p3dListCur, const T2List& p3dListPrev,
									  std::set<T1<Tp>*>& curP3DTriSet, std::set<T2<Tp>*>& prevP3DTriSet,
									  cv::Mat& R, cv::Mat& t,
									  T1T2PairList& matchedP3DList, const double matchedRatio)
{
	double P3DBias = 0;
	double biasOri = 0;
	bool bMatched = false;
	for (T1List::const_iterator itrCur = p3dListCur.begin(); itrCur != p3dListCur.end(); itrCur++)
	{
		if (curP3DTriSet.count(*itrCur))
			continue;
		for (T2List::const_iterator itrPrev = p3dListPrev.begin(); itrPrev != p3dListPrev.end(); itrPrev++)
		{
			if (prevP3DTriSet.count(*itrPrev))
				continue;
			assert(dynamic_cast<cv::Point3_<Tp>*>(*itrCur) && dynamic_cast<cv::Point3_<Tp>*>(*itrPrev));
			cv::Mat curMat = cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(**itrCur));
			cv::Mat prevMat = cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(**itrPrev));
			cv::Mat P3DBiasMat = curMat - (R*prevMat + t);
			P3DBias = cv::norm(P3DBiasMat);

			if (P3DBias < P3D_MATCH_THRES)
			{
				T1T2PairList::iterator matchedItr = matchedP3DList.end();
				for (auto itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++)
				{
					if (itr->first == *itrCur)
					{
						biasOri = cv::norm(*itr->first - *itr->second);
						if (P3DBias < biasOri)
						{
							bMatched = true;
							matchedItr = itr;
						}
					}
				}
				if (bMatched)
				{
					matchedP3DList.erase(matchedItr);
					matchedP3DList.emplace_back(*itrCur, *itrPrev);
					bMatched = false;
				}
				else
					matchedP3DList.emplace_back(*itrCur, *itrPrev);

				break;
			}
		}
	}
	if (matchedP3DList.size() < 1)
		return false;
	/*if (matchedP3DList.size()==3)
	{
	int i = 0;
	cv::Point3_<Tp>* p3dList[3];
	for (P3DMatchedMap::iterator itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++, i++)
	p3dList[i] = itr->first;
	if (crossProductAngle3D(*p3dList[0], *p3dList[1], *p3dList[2]) < 0.1)
	return false;
	}*/
	if (matchedRatio == 0)
		return true;

	int maxP3DMatched = p3dListPrev.size() < p3dListCur.size() ? p3dListPrev.size() : p3dListCur.size();
	if ((double)matchedP3DList.size() / (double)maxP3DMatched >= matchedRatio)
		return true;
	else
		return false;


}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::evaluateRt(const T1List& p3dListCur, const T2List& p3dListPrev,
									  cv::Mat& R, cv::Mat& t,
									  T1T2PairList& matchedP3DList, const double matchedRatio)
{
	double P3DBias = 0;
	double biasOri = 0;
	bool bMatched = false;
	for (T1List::const_iterator itrCur = p3dListCur.begin(); itrCur != p3dListCur.end(); itrCur++)
	{
		for (T2List::const_iterator itrPrev = p3dListPrev.begin(); itrPrev != p3dListPrev.end(); itrPrev++)
		{
			assert(dynamic_cast<cv::Point3_<Tp>*>(*itrCur) && dynamic_cast<cv::Point3_<Tp>*>(*itrPrev));
			cv::Mat curMat = cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(**itrCur));
			cv::Mat prevMat = cv::Mat(dynamic_cast<cv::Point3_<Tp>&>(**itrPrev));
			cv::Mat P3DBiasMat = curMat - (R*prevMat + t);
			P3DBias = cv::norm(P3DBiasMat);

			if (P3DBias < P3D_MATCH_THRES)
			{
				T1T2PairList::iterator matchedItr = matchedP3DList.end();
				for (auto itr = matchedP3DList.begin(); itr != matchedP3DList.end(); itr++)
				{
					if (itr->first == *itrCur)
					{
						biasOri = cv::norm(*itr->first - *itr->second);
						if (P3DBias < biasOri)
						{
							bMatched = true;
							matchedItr = itr;
						}
					}
				}
				if (bMatched)
				{
					matchedP3DList.erase(matchedItr);
					matchedP3DList.emplace_back(*itrCur, *itrPrev);
					bMatched = false;
				}
				else
					matchedP3DList.emplace_back(*itrCur, *itrPrev);

				break;
			}
		}
	}
	if (matchedP3DList.size() < 4)
		return false;
	if (matchedRatio == 0)
		return true;

	int maxP3DMatched = p3dListPrev.size() < p3dListCur.size() ? p3dListPrev.size() : p3dListCur.size();
	if ((double)matchedP3DList.size() / (double)maxP3DMatched >= matchedRatio)
		return true;
	else
		return false;


}


MatcherTemplate
int Matcher_<T1, T2, Tp>::searchMptMatchedByFrame(Frame& refFrame, Frame& curFrame,
										     	  T1PairList& p3DMatchedList, T1T2PairList& p3DMptMatchedList)
{
	T1<Tp>* prevP3D = nullptr;
	T1<Tp>* currentP3D = nullptr;
	KeyPoint* prevKeyLeft = nullptr;
	KeyPoint* currentKeyLeft = nullptr;
	KeyPoint* currentKeyRight = nullptr;
	T2<Tp>* refMapPoint = nullptr;
	int nMatched = 0;
	for (T1PairList::const_iterator itr = p3DMatchedList.begin(); itr != p3DMatchedList.end(); itr++)
	{
		currentP3D = itr->first;
		prevP3D = itr->second;

		auto itrCur = curFrame._p3DMap.find(currentP3D);
		auto itrPrev = refFrame._p3DMap.find(prevP3D);

		if (itrCur != curFrame._p3DMap.end() && itrPrev != refFrame._p3DMap.end())
		{
			currentKeyLeft = itrCur->second.first;
			prevKeyLeft = itrPrev->second.first;
		}
		else
			return false;

		currentKeyRight = curFrame.getKeyPointRight(currentKeyLeft);
		if (!currentKeyRight)
			return false;


		refMapPoint = dynamic_cast<T2<Tp>*>(prevKeyLeft->getMatchedMpt());
		assert(refMapPoint);
		if (refMapPoint)
		{
			if (refMapPoint->isBad())
				continue;

			p3DMptMatchedList.emplace_back(currentP3D, refMapPoint);
			nMatched++;
		}
		/*else
		return false;*/
	}
	return nMatched;
}





MatcherTemplate
int Matcher_<T1, T2, Tp>::findMatchKeysByUnProj(const T2List& mapPointList, Frame& curFrame,
												T1T2PairList& p3DMptMatchedList, const double th3D)
{
	int nMatches = 0;

	for (auto p3DItr = curFrame._p3DMap.begin(); p3DItr != curFrame._p3DMap.end(); p3DItr++)
	{
		T1<Tp>* p3D = p3DItr->first;
		KeyPoint* keyPointLeft = p3DItr->second.first;
		KeyPoint* keyPointRight = p3DItr->second.second;

		if (keyPointLeft->getMatchedMpt())
			continue;//跳过匹配地图点的3d点


		cv::Mat& x3Dc = cv::Mat(*p3D);
		cv::Mat x3Dw = curFrame._Rwc*x3Dc + curFrame._Ow;

		double minDist2 = DBL_MAX;
		double tmpDist2 = 0;
		double minDist = 0;
		T2List::const_iterator bestMptItr = mapPointList.end();
		for (T2List::const_iterator mptItr = mapPointList.begin(); mptItr != mapPointList.end(); mptItr++)
		{
			if ((*mptItr)->isBad())
				continue;//跳过地图坏点
			if ((*mptItr)->getObservations().count(&curFrame))
				continue;//跳过已匹配地图点
			double errX = x3Dw.at<double>(0) - (*mptItr)->x;
			double errY = x3Dw.at<double>(1) - (*mptItr)->y;
			double errZ = x3Dw.at<double>(2) - (*mptItr)->z;

			double tmpDist2 = errX * errX + errY * errY + errZ * errZ;

			if (tmpDist2 < minDist2)
			{
				minDist2 = tmpDist2;
				bestMptItr = mptItr;
			}
		}
		minDist = sqrt(minDist2);
		if (minDist < th3D)
		{
			curFrame.insertMatchedMPt(*bestMptItr, keyPointLeft, keyPointRight);
			(*bestMptItr)->addObservation(&curFrame);
			KeyFrame* pKF = dynamic_cast<KeyFrame*>(&curFrame);
			if (pKF)
				(*bestMptItr)->addKFObservation(pKF);

			p3DMptMatchedList.emplace_back(p3D, *bestMptItr);

			nMatches++;
		}

	}
	return nMatches;
}


//MatcherTemplate
//int Matcher_<T1, T2, Tp>::findMatchKeysByProj(Frame& prevFrame, Frame& curFrame, 
//											  T1T2PairList& p3DMptMatchedList, const double th2D)
//{
//	int nMatched = 0;
//
//	const cv::Mat RcwCurrent = curFrame._Tcw.rowRange(0, 3).colRange(0, 3);
//	const cv::Mat tcwCurrent = curFrame._Tcw.rowRange(0, 3).col(3);
//
//	const cv::Mat twcCurrent = -RcwCurrent.t()*tcwCurrent;
//
//
//	double fx = curFrame._camera->Kl().at<double>(0, 0);
//	double fy = curFrame._camera->Kl().at<double>(1, 1);
//	double cx = curFrame._camera->Kl().at<double>(0, 2);
//	double cy = curFrame._camera->Kl().at<double>(1, 2);
//
//
//	T2KeyPairMap& matchedMptListPrev = prevFrame._matchedMptMap;
//	T2KeyPairMap& matchedMptListCur = curFrame._matchedMptMap;
//	for (T2KeyPairMap::iterator prevItr = matchedMptListPrev.begin(); prevItr != matchedMptListPrev.end(); prevItr++)
//	{
//		T2<Tp>* mapPoint = prevItr->first;
//		if (!mapPoint->isBad())
//		{
//			cv::Mat x3Dw = (cv::Mat_<double>(1, 3) << mapPoint->x, mapPoint->y, mapPoint->z);
//			cv::Mat x3Dc = RcwCurrent*x3Dw + tcwCurrent;
//
//			const double xc = x3Dc.at<double>(0);
//			const double yc = x3Dc.at<double>(1);
//			const double invzc = 1.0 / x3Dc.at<double>(2);
//
//			if (invzc<0)
//				continue;
//
//
//			double u = fx*xc*invzc + cx;
//			double v = fy*yc*invzc + cy;
//
//			if (u<curFrame._minX || u>curFrame._maxX)
//				continue;
//			if (v<curFrame._minY || v>curFrame._maxY)
//				continue;
//
//			double minDist2 = DBL_MAX;
//			double tmpDist2;
//			KeyPoint* bestKeyPoint = nullptr;
//			KeyPoint* bestMatchedKeyLeft = nullptr;
//			for (MptKeyPairMap::iterator curItr = matchedMptListCur.begin(); curItr != matchedMptListCur.end(); curItr++)
//			{
//				KeyPoint* keyPointLeft = curItr->second.first;
//
//				double errX1 = u - keyPointLeft->_undistortX;
//				double errY1 = v - keyPointLeft->_undistortY;
//				tmpDist2 = errX1*errX1 + errY1*errY1;
//				if (tmpDist2 < minDist2)
//				{
//					minDist2 = tmpDist2;
//					bestKeyPoint = keyPointLeft;
//				}
//			}
//			if (bestKeyPoint)
//			{
//				if (sqrt(minDist2)<th2D)
//					bestMatchedKeyLeft = bestKeyPoint;
//
//				if (bestMatchedKeyLeft->_matchedMpt)
//					continue;
//				else
//				{
//					KeyPoint* keyPointRight = curFrame.getKeyPointRight(bestMatchedKeyLeft);
//					curFrame.insertMatchedMPt(mapPoint, bestMatchedKeyLeft, keyPointRight);
//					p3DMptMatchedList.emplace_back(bestMatchedKeyLeft->getMatchedP3D(), mapPoint);
//					mapPoint->addObservation(&curFrame);
//					nMatched++;
//				}
//			}
//		}
//	}
//	return nMatched;
//}



MatcherTemplate
int Matcher_<T1, T2, Tp>::findMatchKeysByProj(const T2List& mapPointList, Frame& curFrame,
											  T1T2PairList& p3DMptMatchedList, const double th2D)
{
	int nMatches = 0;

	//外参
	const cv::Mat Rcw = curFrame._Tcw.rowRange(0, 3).colRange(0, 3);
	const cv::Mat tcw = curFrame._Tcw.rowRange(0, 3).col(3);
	const cv::Mat twc = -Rcw.t()*tcw;

	//内参
	double fx = curFrame._camera->Kl().at<double>(0, 0);
	double fy = curFrame._camera->Kl().at<double>(1, 1);
	double cx = curFrame._camera->Kl().at<double>(0, 2);
	double cy = curFrame._camera->Kl().at<double>(1, 2);

	KeyPoint* bestMatchedKeyLeft = nullptr;
	KeyPoint* bestMatchedKeyRight = nullptr;
	for (KeyPointList::iterator keyItr = curFrame._keyPointsLeft.begin(); keyItr != curFrame._keyPointsLeft.end(); keyItr++)
	{
		if ((*keyItr)->getMatchedMpt())
			continue;

		double minDist2 = DBL_MAX;
		double tmpDist2;
		T2<Tp>* bestMapPoint = nullptr;

		cv::Mat x3Dw;
		cv::Mat x3Dc;
		for (T2List::const_iterator mptItr = mapPointList.begin(); mptItr != mapPointList.end(); mptItr++)
		{
			T2<Tp>* mapPoint = *mptItr;

			x3Dw = (cv::Mat_<double>(3, 1) << mapPoint->x, mapPoint->y, mapPoint->z);
			x3Dc = Rcw * x3Dw + tcw;

			const double xc = x3Dc.at<double>(0);
			const double yc = x3Dc.at<double>(1);
			const double invzc = 1.0 / x3Dc.at<double>(2);

			if (invzc < 0)
				continue;

			double u = fx * xc*invzc + cx;
			double v = fy * yc*invzc + cy;

			if (u<curFrame._minX || u>curFrame._maxX)
				continue;
			if (v<curFrame._minY || v>curFrame._maxY)
				continue;


			double errX = u - (*keyItr)->getUndistortX();
			double errY = v - (*keyItr)->getUndistortY();
			tmpDist2 = errX * errX + errY * errY;
			if (tmpDist2 < minDist2)
			{
				minDist2 = tmpDist2;
				bestMapPoint = mapPoint;
			}
		}
		if (sqrt(minDist2) < th2D)
		{
			KeyPoint* keyPointRight = curFrame.getKeyPointRight(*keyItr);
			curFrame.insertMatchedMPt(bestMapPoint, *keyItr, keyPointRight);
			bestMapPoint->addObservation(&curFrame);

			p3DMptMatchedList.emplace_back((*keyItr)->getMatchedP3D(), bestMapPoint);
			nMatches++;

			/*
				double errX = (*keyItr)->_matchedP3D->x - x3Dc.at<double>(0);
				double errY = (*keyItr)->_matchedP3D->y - x3Dc.at<double>(1);
				double errZ = (*keyItr)->_matchedP3D->z - x3Dc.at<double>(2);
				if (sqrt(errX*errX + errY*errY + errZ*errZ) < th3D)
				{
					KeyPoint* keyPointRight = curFrame.getKeyPointRight(*keyItr);
					curFrame.insertMatchedMPt(bestMapPoint, *keyItr, keyPointRight);
					bestMapPoint->addObservation(&curFrame);

					p3DMptMatchedList.emplace_back((*keyItr)->getMatchedP3D(), bestMapPoint);
					nMatches++;
				}*/
		}

	}
	return nMatches;

}


MatcherTemplate
bool Matcher_<T1, T2, Tp>::matchP3DByDist(Frame& prevFrame, Frame& curFrame,
										  T1T2MatchedMap& matchedP3DList, cv::Mat& R, cv::Mat& t,
										  OrientFunc orientFunc)
{
	T2DistMap prevDistList;
	T1DistMap currentDistList;
	T2DistMap condidatePrevList;
	T1DistMap condidateCurList;
	if (!calcP3DDistList(prevFrame, prevDistList))
	{
		std::cout << "Failed calculating 3DPoint distance list in Frame" << prevFrame.getID() << std::endl;

		return false;
	}
	if (!calcP3DDistList(curFrame, currentDistList))
	{
		std::cout << "Failed calculating 3DPoint distance list in Frame" << curFrame.getID() << std::endl;

		return false;
	}

	//MatchedCountList condidateMatched;
	//typedef std::map<KeyPoint*, int> PrevKey;
	//typedef std::map<KeyPoint*, PrevKey> MatchedMap;
	//MatchedMap condidateMatched;

	T2DistMap::iterator bestPrevItr;
	T1DistMap::iterator bestCurrentItr;


	for (T1DistMap::iterator currentItr = currentDistList.begin(); currentItr != currentDistList.end(); currentItr++)
	{
		double minDiff = FLT_MAX;

		for (T2DistMap::iterator prevItr = prevDistList.begin(); prevItr != prevDistList.end(); prevItr++)
		{
			double distDiff = abs(prevItr->first - currentItr->first);

			if (distDiff < minDiff)
			{
				minDiff = distDiff;
				bestPrevItr = prevItr;
				bestCurrentItr = currentItr;
			}

		}
		if (minDiff < SITE_DIFF_THRES)
		{
			condidateCurList.insert(*bestCurrentItr);
			condidatePrevList.insert(*bestPrevItr);
		}

		/*
		if (minDiff < 10)
		{
		if (currentDistList.size() < prevDistList.size())
		{
		condidateMatched[bestCurrentItr->second.first][bestPrevItr->second.first]++;
		condidateMatched[bestCurrentItr->second.first][bestPrevItr->second.second]++;
		condidateMatched[bestCurrentItr->second.second][bestPrevItr->second.first]++;
		condidateMatched[bestCurrentItr->second.second][bestPrevItr->second.second]++;
		}
		else
		{
		condidateMatched[bestPrevItr->second.first][bestCurrentItr->second.first]++;
		condidateMatched[bestPrevItr->second.first][bestCurrentItr->second.second]++;
		condidateMatched[bestPrevItr->second.second][bestCurrentItr->second.first]++;
		condidateMatched[bestPrevItr->second.second][bestCurrentItr->second.second]++;

		}
		}
		*/
	}
	TrianglePairList trianglePairList;
	if (!generateTriList(condidateCurList, condidatePrevList, trianglePairList))
	{
		std::cout << "Failed generating triangle list from key point in frame!" << std::endl;
		return false;
	}

	TrianglePairMap triPairMap;
	if (!findCondidateMatchedList(trianglePairList, triPairMap))
	{
		std::cout << "Failed find matched keys from matched triangle list in frame!" << std::endl;
		return false;
	}

	if (!evaluateMatch(triPairMap, R, t, matchedP3DList, orientFunc))
	{
		std::cout << "No valid R,t suit the matched triangle pairs, prevFrame id: " << prevFrame.getID() << std::endl;
		return false;
	}

	/*
	KeyPoint* firstMatchedKey;

	PrevKey::iterator bestSecondMatchedItr;
	int maxCount;
	for (MatchedMap::iterator matchedItr = condidateMatched.begin(); matchedItr != condidateMatched.end(); matchedItr++)
	{
	maxCount = -INT_MAX;
	KeyPointList bestKeyPointList;

	firstMatchedKey = matchedItr->first;
	PrevKey secondMatchedList = matchedItr->second;
	for (PrevKey::iterator secondMatchedItr = secondMatchedList.begin(); secondMatchedItr != secondMatchedList.end(); secondMatchedItr++)
	{
	int matchedCount = secondMatchedItr->second;
	if (matchedCount > maxCount)
	{
	bestKeyPointList.clear();
	bestKeyPointList.emplace_back(secondMatchedItr->first);

	maxCount = matchedCount;
	bestSecondMatchedItr = secondMatchedItr;
	}
	else if (matchedCount == maxCount)
	{
	bestKeyPointList.emplace_back(secondMatchedItr->first);
	}
	}

	if (bestKeyPointList.size() ==1)
	{//如果最优匹配特征点只有一个
	if (currentDistList.size() < prevDistList.size())
	{
	matchedKeysList[firstMatchedKey] = bestSecondMatchedItr->first;
	fprintf(stdout, "(%d,%d)\n", firstMatchedKey->_id, bestSecondMatchedItr->first->_id);
	}
	else
	{
	matchedKeysList[bestSecondMatchedItr->first] = firstMatchedKey;
	fprintf(stdout, "(%d,%d)\n", bestSecondMatchedItr->first->_id, firstMatchedKey->_id);
	}
	}
	else
	{//如果最优匹配特征点大于一个，选择距离最近的特征点
	double minDist = DBL_MAX;
	double dist;
	KeyPointList::iterator bestItr;
	for (KeyPointList::iterator condidateItr = bestKeyPointList.begin(); condidateItr != bestKeyPointList.end(); condidateItr++)
	{
	dist = firstMatchedKey->distWithKeyPoint(*condidateItr);
	if (dist < minDist)
	{
	minDist = dist;
	bestItr = condidateItr;
	}
	}
	if (maxCount >= 1)
	{
	if (currentDistList.size() < prevDistList.size())
	{
	matchedKeysList[firstMatchedKey] = *bestItr;
	fprintf(stdout, "(%d,%d)\n", firstMatchedKey->_id, (*bestItr)->_id);
	}
	else
	{
	matchedKeysList[*bestItr] = firstMatchedKey;
	fprintf(stdout, "(%d,%d)\n", (*bestItr)->_id, firstMatchedKey->_id);
	}

	}
	}

	}*/


	return true;
}


MatcherTemplate
bool Matcher_<T1, T2, Tp>::generateTriList(T1DistMap& distListCur, T2DistMap& distListPrev, TrianglePairList& trianglePairList)
{
	T1PairList triSiteListCur;
	T2PairList triSiteListPrev;
	T1PairList siteListCur;
	T2PairList siteListPrev;

	T2DistMap::iterator itrPrev = distListPrev.begin();
	for (T1DistMap::iterator itrCur = distListCur.begin(); itrCur != distListCur.end(); itrCur++, itrPrev++)
	{
		siteListPrev.emplace_back(itrPrev->second);
		siteListCur.emplace_back(itrCur->second);
	}

	T2PairList::iterator itrPrev1 = siteListPrev.begin();
	for (T1PairList::iterator itrCur1 = siteListCur.begin(); itrCur1 != siteListCur.end(); itrCur1++, itrPrev1++)
	{
		T2PairList::iterator itrPrev2 = itrPrev1 + 1;
		for (T1PairList::iterator itrCur2 = itrCur1 + 1; itrCur2 != siteListCur.end(); itrCur2++, itrPrev2++)
		{
			T2PairList::iterator itrPrev3 = itrPrev2 + 1;
			for (P3DPairList::iterator itrCur3 = itrCur2 + 1; itrCur3 != siteListCur.end(); itrCur3++, itrPrev3++)
			{
				triSiteListCur.emplace_back(*itrCur1);
				triSiteListCur.emplace_back(*itrCur2);
				triSiteListCur.emplace_back(*itrCur3);

				triSiteListPrev.emplace_back(*itrPrev1);
				triSiteListPrev.emplace_back(*itrPrev2);
				triSiteListPrev.emplace_back(*itrPrev3);

				if (checkTriangulate(triSiteListCur) && checkTriangulate(triSiteListPrev))
				{
					trianglePairList.emplace_back(triSiteListCur, triSiteListPrev);
				}
				triSiteListCur.clear();
				triSiteListPrev.clear();

			}
		}
	}
	if (trianglePairList.empty())
	{
		return false;
	}
	return true;
}


MatcherTemplate
bool Matcher_<T1, T2, Tp>::checkTriangulate(const T1PairList& p3DPairList)
{
	std::multiset<T1<Tp>*> p3DSet;
	for (T1PairList::const_iterator itr = p3DPairList.begin(); itr != p3DPairList.end(); itr++)
	{
		p3DSet.insert(itr->first);
		p3DSet.insert(itr->second);
	}
	for (T1PairList::const_iterator itr = p3DPairList.begin(); itr != p3DPairList.end(); itr++)
	{
		if (p3DSet.count(itr->first) != 2 || p3DSet.count(itr->second) != 2)
			return false;
	}

	return true;
}


MatcherTemplate
bool Matcher_<T1, T2, Tp>::findCondidateMatchedList(const TrianglePairList& trianglePairList, TrianglePairMap& triPairMap)
{
	for (TrianglePairList::const_iterator itr = trianglePairList.begin(); itr != trianglePairList.end(); itr++)
	{
		const T1PairList& curTriangle = itr->first;
		const T2PairList& prevTriangle = itr->second;

		T1<Tp>* curP3D = nullptr;
		T2<Tp>* prevP3D = nullptr;
		T1T2PairList p3DPairList;

		T2PairList::const_iterator prevItr1 = prevTriangle.begin();
		for (T1PairList::const_iterator curItr1 = curTriangle.begin(); curItr1 != curTriangle.end(); curItr1++, prevItr1++)
		{
			T2PairList::const_iterator prevItr2 = prevItr1 + 1;
			for (T1PairList::const_iterator curItr2 = curItr1 + 1; curItr2 != curTriangle.end(); curItr2++, prevItr2++)
			{
				curP3D = findVertex(*curItr1, *curItr2);
				prevP3D = findVertex(*prevItr1, *prevItr2);
				if (curP3D&&prevP3D)
				{
					p3DPairList.emplace_back(curP3D, prevP3D);
				}
			}
		}
		triPairMap.insert(std::make_pair(*itr, p3DPairList));
	}
	if (triPairMap.empty())
	{
		return false;
	}
	return true;
}



MatcherTemplate
bool Matcher_<T1, T2, Tp>::evaluateMatch(TrianglePairMap& triPairMap,
										 cv::Mat& RBest, cv::Mat& tBest,
										 T1T2MatchedMap& bestMatchedP3DList, OrientFunc orientFunc)
{
	P3DMatchedMap matchedP3DList;
	double bias = .0;
	double minBias = DBL_MAX;

	for (TrianglePairMap::iterator itr = triPairMap.begin(); itr != triPairMap.end(); itr++)
	{
		std::vector<cv::Point3_<Tp>> curP3DList,
			std::vector<cv::Point3_<Tp>> prevP3DList;

		T1T2PairList& p3DPairList = itr->second;
		for (T1T2PairList::iterator pairItr = p3DPairList.begin(); pairItr != p3DPairList.end(); pairItr++)
		{
			assert(dynamic_cast<cv::Point3_<Tp>*>(pairItr->first) && dynamic_cast<cv::Point3_<Tp>*>(pairItr->second));
			cv::Point3_<Tp> curPoint3D = dynamic_cast<cv::Point3_<Tp>&>(*pairItr->first);
			cv::Point3_<Tp> prevPoint3D = dynamic_cast<cv::Point3_<Tp>&>(*pairItr->second);
			curP3DList.emplace_back(curPoint3D);
			prevP3DList.emplace_back(prevPoint3D);
		}

		cv::Mat curMat = cv::Mat((curP3DList)).reshape(1, 3).t();
		cv::Mat prevMat = cv::Mat(prevP3DList).reshape(1, 3).t();
		cv::Mat R, t;
		double scale = 1.0;
		orientFunc(prevMat, curMat, R, t, scale, true);
		if (evaluateRt(triPairMap, R, t, bestMatchedP3DList))
		{
			break;
		}
		bestMatchedP3DList.clear();
		/*
		bias=evaluateRt(triPairMap, R, t, matchedP3DList);

		if (bias!=-1 && bias < minBias)
		{
		minBias = bias;

		RBest = R;
		tBest = t;
		bestMatchedP3DList = matchedP3DList;
		}*/
	}
	if (bestMatchedP3DList.empty())
	{
		//std::cout << "No valid R,t suit the matched triangle pairs" << std::endl;
		return false;
	}


	///////////////////////根据最优匹配点集合计算最终R，t/////////////////
	std::vector<T1<Tp>> curP3DList, prevP3DList;
	for (T1T2MatchedMap::iterator itr = bestMatchedP3DList.begin(); itr != bestMatchedP3DList.end(); itr++)
	{
		assert(dynamic_cast<cv::Point3_<Tp>*>(itr->first) && dynamic_cast<cv::Point3_<Tp>*>(itr->second));
		cv::Point3_<Tp> curPoint3D = dynamic_cast<cv::Point3_<Tp>&>(*itr->first);
		cv::Point3_<Tp> prevPoint3D = dynamic_cast<cv::Point3_<Tp>&>(*itr->second);
		curP3DList.emplace_back(curPoint3D);
		prevP3DList.emplace_back(prevPoint3D);
	}
	cv::Mat curMat = cv::Mat((curP3DList)).reshape(1, curP3DList.size()).t();
	cv::Mat prevMat = cv::Mat(prevP3DList).reshape(1, prevP3DList.size()).t();
	double scale = 1.0;
	orientFunc(prevMat, curMat, RBest, tBest, scale, false);

	return true;
}


MatcherTemplate
bool Matcher_<T1, T2, Tp>::evaluateRt(TrianglePairMap& triPairMap, cv::Mat& R, cv::Mat& t,
									  T1T2MatchedMap& matchedP3DList, const double matchedRatio)
{
	double matchedBias = .0;
	double triBias = .0;

	T1PairSet matchedP3DPairSet;
	for (TrianglePairMap::iterator itr = triPairMap.begin(); itr != triPairMap.end(); itr++)
	{//遍历匹配三角形构造匹配3D点对

		T1T2PairList& p3DPairList = itr->second;
		for (P3DPairList::iterator pairItr = p3DPairList.begin(); pairItr != p3DPairList.end(); pairItr++)
		{
			assert(dynamic_cast<cv::Point3_<Tp>*>(pairItr->first) && dynamic_cast<cv::Point3_<Tp>*>(pairItr->second));
			cv::Point3_<Tp>* curPoint3D = dynamic_cast<cv::Point3_<Tp>*>(pairItr->first);
			cv::Point3_<Tp>* prevPoint3D = dynamic_cast<cv::Point3_<Tp>*>(pairItr->second);
			matchedP3DPairSet.insert(std::make_pair(curPoint3D, prevPoint3D));
		}
	}
	for (P3DPairSet::iterator itr = matchedP3DPairSet.begin(); itr != matchedP3DPairSet.end(); itr++)
	{//遍历匹配3D点对，验证R,t得到低误差的匹配3D点对
		cv::Mat curMat = cv::Mat(*itr->first);
		cv::Mat prevMat = cv::Mat(*itr->second);
		cv::Mat P3DBiasMat = curMat - (R*prevMat + t);
		double P3DBias = cv::norm(P3DBiasMat);
		if (P3DBias < P3D_MATCH_THRES)
		{
			matchedP3DList.insert(std::make_pair(itr->first, itr->second));
		}
	}

	if ((double)matchedP3DList.size() / (double)matchedP3DPairSet.size() >= matchedRatio)
		return true;
	return false;
}
}

