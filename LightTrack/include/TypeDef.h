#ifndef  TYPEDEF_H
#define  TYPEDEF_H

#include <map>
#include <set>
#include <vector>
#include <opencv2/core/core.hpp>

namespace SLAM
{

	class Frame;
	class KeyFrame;
	class KeyPoint;
	template<typename Tp>
	class MapPoint_;

	typedef MapPoint_<double> MapPoint3d;
	typedef MapPoint_<float> MapPoint3f;
	typedef MapPoint_<int> MapPoint3i;


	typedef std::pair<KeyPoint*, KeyPoint*> KeyPointPair;
	typedef std::vector<KeyPoint*> KeyPointList;
	typedef std::map<cv::Point3d*, KeyPointPair> P3DKeyPairMap;
	typedef std::map<MapPoint3d*, KeyPointPair> MptKeyPairMap;

	typedef std::pair<cv::Point3d*, cv::Point3d*> P3DPair;
	typedef std::vector<cv::Point3d*> P3DList;
	typedef std::set<P3DPair> P3DPairSet;
	typedef std::map<cv::Point3d*, std::pair<KeyPoint*, KeyPoint*>> P3DKeyPairMap;

	typedef std::multimap<double, P3DPair> P3DDistMap;
	typedef std::map<cv::Point3d*, cv::Point3d*> P3DMatchedMap;
	typedef std::map<cv::Point3d*, MapPoint3d*> P3DMptMatchedMap;
	typedef std::vector<P3DPair> P3DPairList;
	typedef std::vector<std::pair<cv::Point3d*, MapPoint3d*>> P3DMptPairList;

	typedef std::pair<P3DPairList, P3DPairList> TrianglePair;
	typedef std::vector<P3DPairList> TriangleList;
	typedef std::vector<TrianglePair> TrianglePairList;
	typedef std::multimap<TrianglePair, P3DPairList> TrianglePairMap;

	typedef std::vector<MapPoint3d*> MptList;
	typedef std::set<MapPoint3d*> MptSet;
	typedef std::pair<MapPoint3d*, MapPoint3d*> MptPair;
	typedef std::map<double, MptPair> MptDistMap;
	typedef std::vector<MptPair> MptPairList;

	typedef std::map<Frame*, int> FrameWeight;
	typedef std::set<Frame*> FrameSet;
	typedef std::map<Frame*, int> FrameMap;
	typedef std::vector<Frame*> FrameList;

	typedef std::vector<KeyFrame*> KeyFrameList;
	typedef std::map<int, KeyFrame*> KFMap;
	typedef std::set<KeyFrame*> KFSet;

}


#endif // ! TYPEDEF_H