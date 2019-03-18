// LightTrack.cpp : 定义控制台应用程序的入口点。

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <map>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include "System.h"

#define HISTO_LENGTH 10



#ifdef _STAGE 

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::cout << "Not enough params!" << std::endl;
		return EXIT_FAILURE;
	}
	if ((_access(argv[1], 0)) == -1)
	{
		std::cout << "Input Dir wrong!" << std::endl;
		return EXIT_FAILURE;
	}

	SLAM::System system;//SLAM工作数据集对象

	////////////////中间结果保存设置/////////////////////
	std::string workSpace = argv[1];
	if (workSpace.back() != '\\')workSpace += '\\';
	SLAM::System::instance()->setWorkSpace(workSpace);//设置slam工作目录

	SLAM::System::instance()->setOutputTrackImgFlag(false);//是否输出帧-帧跟踪匹配图像
	SLAM::System::instance()->setTrackImgOutDir(workSpace + "outputTrack\\");//帧-帧标志点匹配图像输出目录

	SLAM::System::instance()->setOutputStereoImgFlag(true);//是否输出单帧左右相机标志点匹配图像
	SLAM::System::instance()->setStereoImgOutDir(workSpace + "output2D\\");//左右相机匹配图像输出目录

	SLAM::System::instance()->setOutputPosFlag(true);//是否输出每帧跟踪位姿到文件
	SLAM::System::instance()->setPosOutDir(workSpace);
	SLAM::System::instance()->setPosFileName("posList.txt");

	SLAM::System::instance()->setOutputP3dFlag(false);//是否输出每帧3d点坐标到文件
	SLAM::System::instance()->setP3dOutPath(workSpace + "p3dList.txt");

	//////////////////优化方式选择///////////////
	SLAM::System::instance()->setOptimize3DFlag(true);//采用3d-3d优化
	SLAM::System::instance()->setLocalBundleAdjustmentFlag(true);//开启子线程线程局部优化
	SLAM::System::instance()->setDispOptimizeStats(true);

	std::string calibFile = workSpace + "Final_cen.rt";
	SLAM::System::instance()->loadStageCamera(calibFile);

	///////////////////////初始化//////////////////////
	if (!SLAM::System::instance()->initiate())
	{
		std::cout << "Failed Initiating" << std::endl;
		return EXIT_FAILURE;
	}

	//////////////从图像目录载入帧图像路径表///////////////////
	std::string imgDir, imgLeftDir, imgRightDir;
	//imgDir = workSpace + "1\\Dev1\\";
	imgDir = workSpace;
	SLAM::ReadWriter::FilePathMap dirMap;
	SLAM::ReadWriter::getDirListFromDir(imgDir, dirMap);
	SLAM::ReadWriter::FilePathMap leftImgPathMap;
	SLAM::ReadWriter::FilePathMap rightImgPathMap;
	int i = 0;
	int frameCount = dirMap.size();
	for (SLAM::ReadWriter::FilePathMap::iterator itr = dirMap.begin(); itr != dirMap.end(); itr++, i++)
	{
		char indexStr[5];
		itoa(itr->first, indexStr, 10);
		std::string imgLeftPath = itr->second + "\\Dev1\\" + indexStr + "\\L_Cam\\8.png";
		std::string imgRightPath = itr->second + "\\Dev1\\" + indexStr + "\\R_Cam\\8.png";
		//std::string imgLeftPath = itr->second + "\\L_Cam\\8.png";
		//std::string imgRightPath = itr->second + "\\R_Cam\\8.png";
		leftImgPathMap.insert(std::make_pair(i, imgLeftPath));
		rightImgPathMap.insert(std::make_pair(i, imgRightPath));
	}

	if (leftImgPathMap.size() != leftImgPathMap.size())
	{
		std::cout << "Wrong input image files sequence" << std::endl;
		return EXIT_FAILURE;
	}
	//cv::Mat M1l, M2l, M1r, M2r;
	//cv::initUndistortRectifyMap(camera.Kl(), camera.Dl(), camera.Rl(), camera.Pl().rowRange(0, 3).colRange(0, 3), cv::Size(camera.width(), camera.height()), CV_32F, M1l, M2l);
	//cv::initUndistortRectifyMap(camera.Kr(), camera.Dr(), camera.Rr(), camera.Pr().rowRange(0, 3).colRange(0, 3), cv::Size(camera.width(), camera.height()), CV_32F, M1r, M2r);
	SLAM::ReadWriter::StrList leftImgPathList;
	SLAM::ReadWriter::StrList rightImgPathList;
	for (SLAM::ReadWriter::FilePathMap::iterator fItr = leftImgPathMap.begin(); fItr != leftImgPathMap.end(); fItr++)
		leftImgPathList.emplace_back(fItr->second);
	for (SLAM::ReadWriter::FilePathMap::iterator fItr = rightImgPathMap.begin(); fItr != rightImgPathMap.end(); fItr++)
		rightImgPathList.emplace_back(fItr->second);

	//////////////////////////开始SLAM/////////////////////////
	unsigned int imgCount = leftImgPathList.size();
	cv::Mat imgLeft, imgRight, imgLeftRect, imgRightRect;
	SLAM::System::instance()->setLocalBundleAdjustmentFlag(false);
	for (int i = 0; i < imgCount; i++)
	{
		imgLeft = cv::imread(leftImgPathList[i], CV_LOAD_IMAGE_UNCHANGED);
		imgRight = cv::imread(rightImgPathList[i], CV_LOAD_IMAGE_UNCHANGED);
		//cv::Mat R, t;
		//int index = 0;

		if (imgLeft.empty())
		{
			return false;
		}
		if (imgRight.empty())
		{
			return false;
		}

		//if (SLAM::Frame* newFrame=createFrame(imgLeftRect, imgRightRect))
		if (SLAM::Frame* newFrame = SLAM::System::instance()->createFrame(imgLeft, imgRight))
		{
			if (SLAM::System::instance()->getTracker().tracking(*newFrame))
			{
				fprintf(stdout, "Tracking frame %d succeeded!\n\n", newFrame->getID());
			}
		}
		else
		{
			std::cout << "Failed creating frame!" << std::endl << std::endl;
		}
	}


	///////////////////输出所有帧位姿//////////////////
	std::ofstream ofsPos = std::ofstream(workSpace + "posListBA.txt", std::ios::trunc | std::ios::in);
	SLAM::FrameList& frameList = SLAM::System::instance()->getGlobalMap().getFrameList();
	if (!SLAM::System::instance()->outputRt(frameList, ofsPos))
	{
		std::cout << "Failed outputing R,t into file" << std::endl;
		return EXIT_FAILURE;
	}
	////////////////控制台输出所有地图点优化前后坐标变化距离差/////////////////////////////
	SLAM::GlobalMap::MptList& mapPointList = SLAM::System::instance()->getGlobalMap().getMapPointList();
	std::cout << "Map point count: " << mapPointList.size() << std::endl;
	for (SLAM::GlobalMap::MptList::iterator itr = mapPointList.begin(); itr != mapPointList.end(); itr++)
	{
		std::cout << (*itr)->getID() << ":" << cv::norm((*itr)->getLastCoord() - (*itr)->getOriCoord()) << " "
			<< (*itr)->obs() << " " << (*itr)->obsKF()
			<< std::endl;
	}

	std::string mapSavPath = workSpace + "map.txt";
	SLAM::System::instance()->saveGlobalMap(mapSavPath);

	system("pause");

	SLAM::System::instance()->shutDown();

}


#else
int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::cout << "Not enough params!" << std::endl;
		return EXIT_FAILURE;
	}
	if ((_access(argv[1], 0)) == -1)
	{
		std::cout << "Input Dir wrong!" << std::endl;
		return EXIT_FAILURE;
	}

	////////////////中间结果保存设置/////////////////////
	std::string workSpace = argv[1];
	if (workSpace.back() != '\\')workSpace += '\\';
	SLAM::System::instance()->setWorkSpace(workSpace);//设置slam工作目录

	SLAM::System::instance()->setOutputTrackImgFlag(false);//是否输出帧-帧跟踪匹配图像
	SLAM::System::instance()->setTrackImgOutDir(workSpace + "outputTrack\\");//帧-帧标志点匹配图像输出目录
							 
	SLAM::System::instance()->setOutputStereoImgFlag(false);//是否输出单帧左右相机标志点匹配图像
	SLAM::System::instance()->setStereoImgOutDir(workSpace + "output2D\\");//左右相机匹配图像输出目录
							 
	SLAM::System::instance()->setOutputPosFlag(false);//是否输出每帧位姿到文件
	SLAM::System::instance()->setPosOutDir(workSpace);
	SLAM::System::instance()->setPosFileName("posList.txt");
							 
	SLAM::System::instance()->setOutputP3dFlag(false);//是否输出每帧3d点坐标到文件
	SLAM::System::instance()->setP3dOutPath(workSpace + "p3dList.txt");

	SLAM::System::instance()->setDisplay3DModelFlag(true);//是否实时显示3d模型

	//////////////////优化方式选择///////////////
	SLAM::System::instance()->setOptimize3DFlag(true);//是否采用3d-3d优化
	SLAM::System::instance()->setLocalBundleAdjustmentFlag(true);//是否开启子线程局部优化
	SLAM::System::instance()->setDispErrorStats(false);//控制台是否输出每帧位姿、局部优化误差信息
	SLAM::System::instance()->setLoopClosingFlag(true);//是否开启回环检测

	//////////////关键环节实现方法选择///////////////
	SLAM::System::instance()->setCalcMatchedType(SLAM::Frame::NORM);//设置双目极线匹配方法
	SLAM::System::instance()->setExtractMarkType(SLAM::Feature::SIMPLE);//设置图像标志点提取方法
	SLAM::System::instance()->setOrientType(SLAM::Matcher_<>::UNIT_QUAT);//设置帧-帧基于3d点匹配的绝对定向算法
	SLAM::System::instance()->setCalcSim3Type(SLAM::Matcher_<>::SUCCESSIVE);//设置帧-帧相对Sim3的计算方法
	SLAM::System::instance()->setMatchedKeysType(SLAM::Matcher_<>::UNPROJ);//设置单帧2d点-全局地图点的匹配方法

	std::string calibFile = workSpace + "External.rt";
	SLAM::System::instance()->loadHandHeldCamera(calibFile);
	
	
	///////////////////////初始化//////////////////////
	if (!SLAM::System::instance()->initiate())
	{
		std::cout << "Failed Initiating" << std::endl;
		return EXIT_FAILURE;
	}

	//////////////从图像目录载入帧图像路径列表///////////////////
	SLAM::ReadWriter::FilePathMap leftImg1PathMap, rightImg1PathMap,leftImg2PathMap, rightImg2PathMap;
	/*SLAM::ReadWriter::StrList leftImgPathList,rightImgPathList;
	std::string imgInputDir = workSpace + "TEST\\";
	if (!SLAM::ReadWriter::loadImageList(imgInputDir, leftImgPathList, rightImgPathList))
	{
	std::cout << "Failed Loading image list" << std::endl;
	return EXIT_FAILURE;
	}*/

	std::string imgInputDir = workSpace;
	if (!SLAM::ReadWriter::loadHandHeldImage1List(imgInputDir, leftImg1PathMap, rightImg1PathMap))
	{
		std::cout << "Failed Loading image list" << std::endl;
		return EXIT_FAILURE;
	}
	if (!SLAM::ReadWriter::loadHandHeldImage2List(imgInputDir, leftImg2PathMap, rightImg2PathMap))
	{
		std::cout << "Failed Loading image list" << std::endl;
		return EXIT_FAILURE;
	}

	////////////////////加载全局地图文件(续扫功能使用)///////////////////
	std::string mapSavPath = workSpace + "map.txt";
	SLAM::System::instance()->getViewer().requestStop();
	while (!SLAM::System::instance()->getViewer().isStopped())
		SLAM::usleep(500);
	//SLAM::System::instance()->loadGlobalMap(mapSavPath);
	SLAM::System::instance()->getViewer().release();

	//////////////////////////开始SLAM/////////////////////////
	cv::Mat img1Left, img1Right, img2Left, img2Right;
	cv::Mat img1RectLeft, img1RectRight;
	cv::Mat img2RectLeft, img2RectRight;
	/*int* eccHist = new int[HISTO_LENGTH];
	int* areaHist = new int[HISTO_LENGTH];
	memset(eccHist, 0, HISTO_LENGTH*sizeof(int));
	memset(areaHist, 0, HISTO_LENGTH*sizeof(int));
	double maxEcc = 1.0;
	double maxArea = 2000;
	SLAM::System::Statistic eccStats;
	SLAM::System::Statistic areaStats;*/
	clock_t start, end;
	int index = 0;
	for (SLAM::ReadWriter::FilePathMap::iterator itr = leftImg1PathMap.begin(); itr != leftImg1PathMap.end();itr++)
	{	
		index = itr->first;

		img1Left = cv::imread(leftImg1PathMap[index], CV_LOAD_IMAGE_UNCHANGED);
		img1Right = cv::imread(rightImg1PathMap[index], CV_LOAD_IMAGE_UNCHANGED);

		img2Left = cv::imread(leftImg2PathMap[index], CV_LOAD_IMAGE_UNCHANGED);
		img2Right = cv::imread(rightImg2PathMap[index], CV_LOAD_IMAGE_UNCHANGED);

		if (img1Left.empty())
		{
			std::cerr << std::endl << "Failed to load image at: " << std::string(leftImg1PathMap[index]) << std::endl;
			continue;
		}
		if (img1Right.empty())
		{
			std::cerr << std::endl << "Failed to load image at: " << std::string(rightImg1PathMap[index]) << std::endl;
			continue;
		}
		if (img2Left.empty())
		{
			std::cerr << std::endl << "Failed to load image at: " << std::string(leftImg2PathMap[index]) << std::endl;
			continue;
		}
		if (img2Right.empty())
		{
			std::cerr << std::endl << "Failed to load image at: " << std::string(rightImg2PathMap[index]) << std::endl;
			continue;
		}

		SLAM::System::instance()->undistortImage(img1Left, img1Right, img1RectLeft, img1RectRight);
		start = clock();
		if (SLAM::Frame* newFrame1 = SLAM::System::instance()->createFrame(img1RectLeft, img1RectRight, true))
		//if (SLAM::Frame* newFrame1 = SLAM::System::instance()->createFrame(img1Left, img1Right, false))
		{	
			end = clock();
			std::cout << "Frame time: " << (double)(end - start)/CLOCKS_PER_SEC << std::endl;

			start = clock();
			if (SLAM::System::instance()->getTracker().tracking(*newFrame1))
			{
				/*if (newFrame1->getID() > 100)
					break;*/
				fprintf(stdout, "Tracking frame %d succeeded!\n\n", newFrame1->getID());	
			}	
			else
				fprintf(stdout, "Tracking frame %d Failed!\n\n", newFrame1->getID());
			end = clock();
			std::cout << "Traking time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl << std::endl << std::endl;
		}
		else
			fprintf(stdout, "Failed creating frame!\n\n");

		SLAM::System::instance()->undistortImage(img2Left, img2Right, img2RectLeft, img2RectRight); 
		start = clock();
		if (SLAM::Frame* newFrame2 = SLAM::System::instance()->createFrame(img2RectLeft, img2RectRight, true))
		//if (SLAM::Frame* newFrame2 = SLAM::System::instance()->createFrame(img2Left, img2Right, false))
		{
			end = clock();
			std::cout << "Frame time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl;

			start = clock();
			if (SLAM::System::instance()->getTracker().tracking(*newFrame2))
			{
				fprintf(stdout, "Tracking frame %d succeeded!\n\n", newFrame2->getID());
			}
			else
				fprintf(stdout, "Tracking frame %d Failed!\n\n", newFrame2->getID());
			end = clock();
			std::cout << "Traking time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl << std::endl << std::endl;
		}
		else
			fprintf(stdout, "Failed creating frame!\n\n");
					
	}
	
	/*
	//SLAM::System::instance()->setImgInputDir("F:\\Data\\TEST");//图像目录(运行slamRun()需要设置)
	//////////////////////////开始slam///////////////////////
	if (!SLAM::System::instance()->slamRun())
	{
	std::cout << "Failed SLAMING" << std::endl;
	return EXIT_FAILURE;
	}*/


	SLAM::System::instance()->getLoopCloser().runGlobalBundleAdjustment();
	SLAM::System::instance()->clearBadFrames();
	SLAM::System::instance()->setStopped(true);

	/////////////////保存全局地图到文件////////////////
	SLAM::System::instance()->saveGlobalMap(mapSavPath);

	SLAM::GlobalMap* globalMap = &SLAM::System::instance()->getGlobalMap();
	
	///////////////////输出优化后的所有帧位姿到文件//////////////////
	std::ofstream ofsPos = std::ofstream(workSpace + "posListBA.txt", std::ios::trunc | std::ios::in);
	SLAM::FrameList& frameList = globalMap->getFrameList();
	if (!SLAM::System::instance()->outputRt(frameList, ofsPos))
	{
		std::cout << "Failed outputing R,t into file" << std::endl;
		return EXIT_FAILURE;
	}
	std::map<double, SLAM::Frame*, std::greater<double>> RMSESet1, RMSESet2;
	for (auto itr = globalMap->getFrameList().begin(); itr != globalMap->getFrameList().end(); itr++)
	{
		RMSESet1.insert(std::make_pair((*itr)->calcMatchedRMSE(), *itr));
	}
	

	///////////////输出全局地图点距离列表到文件////////////////////
	std::ofstream ofsDist = std::ofstream(workSpace + "distList.txt", std::ios::trunc | std::ios::in);
	SLAM::MptDistMap& mptDistMap= globalMap->getMptDistMap();
	if (!SLAM::System::instance()->outputMptDistList(mptDistMap, ofsDist))
	{
		std::cout << "Failed outputing map points' distance list into file" << std::endl;
		return EXIT_FAILURE;
	}

	////////////////控制台输出所有地图点优化前后坐标变化距离差/////////////////////////////
	SLAM::MptList& mapPointList = SLAM::System::instance()->getGlobalMap().getMapPointList();
	std::cout << "Map point count: " << mapPointList.size() << std::endl;
	for (SLAM::MptList::iterator itr = mapPointList.begin(); itr != mapPointList.end(); itr++)
	{
		std::cout << (*itr)->getID() << ":" << cv::norm((*itr)->getLastCoord() - (*itr)->getOriCoord()) << " "
			<< (*itr)->obs() << " " << (*itr)->obsKF()
			<< std::endl;
	}
	std::cout << "Keyframes size is: "
			  << SLAM::System::instance()->getGlobalMap().getKeyFrameList().size()
			  << std::endl;


	double shiftRMSE1 = 0;
	double shiftRMSE2 = 0;
	double rotateRMSE = 0;
	SLAM::System::instance()->calcFramesRefShiftRMSE(frameList, **frameList.begin(), shiftRMSE1);
	SLAM::System::instance()->calcFramesShiftRMSE(frameList, shiftRMSE2);
	SLAM::System::instance()->calcFramesRotateRMSE(frameList, rotateRMSE);
	std::cout << "Frames shift RMSE from frame " << (*frameList.begin())->getID()
			  << " is: " << shiftRMSE1
			  << std::endl;
	std::cout << "Frames shift RMSE  "
			  << "is: " << shiftRMSE2
			  << std::endl;
	std::cout << "Frames rotate RMSE  "
			  << "is: " << rotateRMSE
			  << std::endl;
	///////////////输出全局地图点点云文件////////////////////
	//std::string mptCloudPath = workSpace + "mapPointCloud.txt";
	//std::ofstream ofsMpt = std::ofstream(mptCloudPath, std::ios::trunc | std::ios::in);
	//for (SLAM::GlobalMap::MptList::iterator itr = mapPointList.begin(); itr != mapPointList.end(); itr++)
	//{
	//	ofsMpt << std::setprecision(8) << (*itr)->x << " " << (*itr)->y << " " << (*itr)->z << " " << std::endl;
	//}


	/////////////////求地图点云平面拟合误差并输出到文件///////////////////
	//SLAM::System::instance()->outPutPlaneFittingError(mptCloudPath,workSpace + "planeFitting.txt");

	////////////////绘制统计直方图////////////////////
	/*cv::Mat eccHistImg, areaHistImg;
	SLAM::System::instance()->drawStatsHisto(eccHistImg, eccHist, HISTO_LENGTH, maxEcc, &eccStats);
	SLAM::System::instance()->drawStatsHisto(areaHistImg, areaHist, HISTO_LENGTH, maxArea, &areaStats);
	cv::imwrite(workSpace + "标志点离心率统计.bmp", eccHistImg);
	cv::imwrite(workSpace + "标志点面积统计.bmp", areaHistImg);
	cv::imshow("【标志点离心率直方图】", eccHistImg);
	cv::imshow("【标志点面积直方图】", areaHistImg);*/

	int* matchedRMSEHist = new int[HISTO_LENGTH];
	memset(matchedRMSEHist, 0, HISTO_LENGTH*sizeof(int));
	double maxRMSE = 0.2;
	SLAM::System::Statistic matchedRMSEStats;
	SLAM::System::instance()->calcFrameMatchedRMSE(&SLAM::System::instance()->getGlobalMap(), matchedRMSEHist, HISTO_LENGTH, maxRMSE, matchedRMSEStats);
	cv::Mat matchedRMSEHistImg;
	SLAM::System::instance()->drawStatsHisto(matchedRMSEHistImg, matchedRMSEHist, HISTO_LENGTH, maxRMSE, &matchedRMSEStats);
	cv::imwrite(workSpace + "帧匹配均方差统计.bmp", matchedRMSEHistImg);
	cv::imshow("【帧匹配均方差直方图】", matchedRMSEHistImg);
	

	int* matchedKFRMSEHist = new int[HISTO_LENGTH];
	memset(matchedKFRMSEHist, 0, HISTO_LENGTH*sizeof(int));
	double maxKFRMSE = 0.2;
	SLAM::System::Statistic matchedKFRMSEStats;
	SLAM::System::instance()->calcKFMatchedRMSE(&SLAM::System::instance()->getGlobalMap(), matchedKFRMSEHist, HISTO_LENGTH, maxKFRMSE, matchedKFRMSEStats);
	cv::Mat matchedKFRMSEHistImg;
	SLAM::System::instance()->drawStatsHisto(matchedKFRMSEHistImg, matchedKFRMSEHist, HISTO_LENGTH, maxKFRMSE, &matchedKFRMSEStats);
	cv::imwrite(workSpace + "关键帧匹配均方差统计.bmp", matchedKFRMSEHistImg);
	cv::imshow("【关键帧匹配均方差直方图】", matchedKFRMSEHistImg);
	cv::waitKey(0);
	//system("pause");
	
	/////////////////释放线程////////////////
	SLAM::System::instance()->shutDown();
	//////////////////释放内存////////////////
	SLAM::System::instance()->release();

	return EXIT_SUCCESS;
}
#endif