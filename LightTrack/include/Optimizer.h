#ifndef OPTIMIZER_H
#define OPTIMIZER_H


#include "GlobalMap.h"
#include "LoopClosing.h"

#include <vector>
#include <list>
#include <iostream>


#include <Eigen/Geometry>
#include <Eigen/StdVector>



namespace SLAM
{


class Optimizer
{
public:
	static double MAX_EDGE_RMSE;
	static double MAX_EDGE_ERROR;
	static double MAX_BA_EDGE_ERROR;


	class VertexSim3ExpmapRGBD;

	class EdgeSim3ProjectXYZRGBD;

	class EdgeInverseSim3ProjectXYZRGBD;

	class EdgeProjectXYZRGBDOnlyPose;

	class  EdgeProjectXYZRGBD;

	class EdgeSE3ProjectXYZOnlyPose;

	class EdgeSE3ProjectXYZ;



public:
	


	static int optimizeRGBDSim3(Frame *pF1, Frame *pF2,
								P3DPairList& matchedP3DList, cv::Mat& S12,
		                        const int& iterCount,const bool& bFixScale,const bool& bDispErrorStats);

	static int poseOptimization2D(Frame* pFrame, const int& iterCount, const bool& bDispErrorStats);

	static int poseOptimization3D(Frame* pFrame, const int& iterCount, const bool& bDispErrorStats);

	static void localKFBundleAdjustment2D(KeyFrame* keyFrame, GlobalMap* pMap, bool& bAbort);
		   
	static void localKFBundleAdjustment3D(KeyFrame* keyFrame, GlobalMap* pMap, bool& bAbort);

	static void localBundleAdjustment2D(Frame* currentFrame, GlobalMap* pMap, FrameList& localFrameList, MptList& localMptList);
		   
	static void localBundleAdjustment3D(Frame* currentFrame, GlobalMap* pMap, FrameList& localFrameList, MptList& localMptList);

	static void optimizeEssentialGraph(GlobalMap* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
									   const LoopClosing::KeyFrameAndPose &nonCorrectedSim3,
									   const LoopClosing::KeyFrameAndPose &correctedSim3,
									   const std::map<KeyFrame *, std::set<KeyFrame *>> &loopConnections,
									   const bool& bFixScale);

	static void globalBundleAdjustment(KeyFrameList& KFList,MptList& mptList, 
									   KeyFrame* loopKF,KeyFrame::KFCompSet& backKFSet,
									   const int& nIterations);

};

}

#endif // OPTIMIZER_H