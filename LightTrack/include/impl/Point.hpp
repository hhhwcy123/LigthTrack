#ifndef POINT_H
#define POINT_H

#include "Point.h"

namespace SLAM
{


template<typename T>
std::mutex MapPoint_<T>::_obsMutex;

template<typename T>
MapPoint_<T>::MapPoint_(cv::Point3_<T>& point3D) :
	_id(GlobalMap::mptCount()++),
	_refKF(nullptr),
	_outLierCount(0),
	//_fOutLier(false),
	_fBad(false),
	_localForKF(-1),
	_coordOri(cv::Point3_<T>(x, y, z)),
	_coordLast(cv::Point3_<T>(x, y, z)),
	_globalMap(nullptr),
	cv::Point3_<T>(point3D)
{}

template<typename T>
MapPoint_<T>::MapPoint_(double x, double y, double z) :
	_id(GlobalMap::mptCount()++),
	_refKF(nullptr),
	//_fOutLier(false), 
	_fBad(false),
	_localForKF(-1),
	_coordOri(cv::Point3_<T>(x, y, z)),
	_coordLast(cv::Point3_<T>(x, y, z)),
	_globalMap(nullptr),
	cv::Point3_<T>(x, y, z)
{}


template<typename T>
void MapPoint_<T>::addKFObservation(KeyFrame* kF)
{
	std::unique_lock<std::mutex> lockMptObs(_obsMutex);
	if (_observationKFs.count(kF))
		return;
	_observationKFs.insert(kF);

}


template<typename T>
void MapPoint_<T>::addObservation(Frame* frame)
{
	std::unique_lock<std::mutex> lockMptObs(_obsMutex);
	if (_observations.count(frame))
		return;
	_observations.insert(frame);

}


template<typename T>
void MapPoint_<T>::addConnSite(MptPair& site, double len)
{
	if (len == 0)
		len = cv::norm(site.first - site.second);
	_connSiteMap[len] = site;
}


template<typename T>
void MapPoint_<T>::eraseObservation(Frame* frame)
{
	std::unique_lock<std::mutex> lockMptObs(_obsMutex);
	if (_observations.count(frame))
	{
		_observations.erase(frame);

		if (_observations.empty())
		{
			setBad(true);//如果观察帧数为0，在LocalMapping子线程中删除
			_globalMap->insertBadMpt(this);
		}
		else if (!isBad())
			if (obs() > Tracker::OBSERVE_THRES)
				if ((double)_outLierCount / (double)obs() > 0.5)
				{
					setBad(true);
					_globalMap->insertBadMpt(this);
				}
	}

}


template<typename T>
void MapPoint_<T>::eraseKFObservation(KeyFrame* pKF)
{
	std::unique_lock<std::mutex> lockMptObs(_obsMutex);
	if (_observationKFs.count(pKF))
	{
		_observationKFs.erase(pKF);
		if (_observations.count(pKF))
			_observations.erase(pKF);

		if (!_observationKFs.empty())
		{//删除pKF后地图点仍有观察关键帧，修改该地图点的参考（起始）观察关键帧
			if (_refKF == pKF)
			{
				auto pItr = std::find(pKF->_newMptList.begin(), pKF->_newMptList.end(), this);
				pKF->_newMptList.erase(pItr);

				auto minItr = std::min_element(_observationKFs.begin(), _observationKFs.end(), KeyFrame::KFComparator());
				_refKF = *minItr;
				_refKF->_newMptList.emplace_back(this);
			}
		}
		else
		{//如果地图点没有观察关键帧
			auto pItr = std::find(pKF->_newMptList.begin(), pKF->_newMptList.end(), this);
			pKF->_newMptList.erase(pItr);
			_refKF = nullptr;
			setBad(true);//如果观察关键帧数为0，在LocalMapping子线程中删除
			_globalMap->insertBadMpt(this);
		}

		if (!isBad())
			if (obs() > Tracker::OBSERVE_THRES)
				if ((double)_outLierCount / (double)obs() > 0.5)
				{
					setBad(true);
					_globalMap->insertBadMpt(this);
				}
	}
}



template<typename T>
bool MapPoint_<T>::outputCoord(std::ofstream& ofs)
{
	if (!ofs.is_open())
	{
		std::cout << "Failed Opening source file" << std::endl;
		return false;
	}
	ofs << _id << ": " << x << " " << y << " " << z << std::endl;
	return true;
}


}//namespace SLAM


#endif // !POINT_H