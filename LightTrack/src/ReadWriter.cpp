#include "ReadWriter.h"
#include "Matcher.h"
#include "impl\Matcher.hpp"
#include "Camera.h"
#include "System.h"

namespace SLAM
{


FILE* ReadWriter::_INFILE = nullptr;
FILE* ReadWriter::_DFILE = stderr;



FILE* ReadWriter::efopen(const char *file, char *mode)
{
	FILE* fp;
	if ((fp = fopen(file, mode)) != NULL) return fp;
	fprintf(_DFILE, "couldn't open file %s mode %s\n", file, mode);
	return NULL;
}




bool ReadWriter::readLine(FILE* infile, char* buffer, const int& len)
{
	fgets(buffer, len, infile);
	//跳过空格和注释
	while (isspace(*buffer))
	{
		if (*buffer == '#' || *buffer == '\0' || *buffer == '\n')
			return false;
		buffer++;
	}
	return true;
}


/*******************************
后续改为protbuf协议存取内存对象
*******************************/
bool ReadWriter::loadMapFile(const std::string filePath, System& system)
{
	GlobalMap& globalMap = system.getGlobalMap();
	Camera& camera = system.getCamera();

	typedef std::map<MapPoint3d*, std::vector<int>> MptVectorMap;

	if (!(_INFILE = efopen(filePath.c_str(), "r")))
		return false;
	int k = 0;
	static int lineCount(0);
	char buffer[10240];
	typedef std::map<MapPoint3d*, int> MptIntMap;
	MptVectorMap mptMathcedFrames, mptMathcedKFs;
	MptIntMap mptRefKFIDMap;
	int mptSize = 0;
	int frameSize = 0;
	int keyFrameSize = 0;

	globalMap.release();

	///////////////////////读取地图点///////////////////////
	//读取地图点全局数量
	{
		while (!readLine(_INFILE, buffer, 10239))
			lineCount++;
		lineCount++;
		if (sscanf(buffer, "%d", &mptSize) != 1)
		{
			fprintf(stderr, "Syntax error reading line %d in file %s, while reading map points size!\n", lineCount, filePath);
			fclose(_INFILE);
			return false;
		}
	}

	//遍历读取地图点
	MptList& mptList = globalMap.getMapPointList();
	for (int i = 0; i < mptSize; i++)
	{
		MapPoint3d* mpt = new MapPoint3d();
		mpt->setGlobalMap(&globalMap);

		//读取地图点基本信息
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;
			char* bufferp = buffer;

			int index = 0;
			cv::Mat normal = cv::Mat::zeros(3, 1, CV_64F);
			int refKFid = -1;
			if (sscanf(bufferp, "%d%lf%lf%lf%lf%lf%lf%d%d",
				&index,
				&mpt->x, &mpt->y, &mpt->z,
				&normal.at<double>(0), &normal.at<double>(1), &normal.at<double>(2),
				&mpt->outliersCount(),
				&refKFid) != 9)
			{
				fprintf(stderr, "Syntax error reading line %d in file %s, while reading map point's info!\n", lineCount, filePath);
				fclose(_INFILE);
				return false;
			}
			mptRefKFIDMap[mpt] = refKFid;
			mpt->setID(index);
			mpt->getOriCoord() = mpt->getLastCoord() = *mpt;//*dynamic_cast<cv::Point3d*>(mpt);
			mpt->setNormOri(normal);
			mpt->setNormLast(normal);
			mptList.emplace_back(mpt);
			globalMap.addMptDistList(mpt, Tracker::P3D_DIST_MAX);
		}
		
		//读取匹配帧数
		int obsSize = 0;
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &obsSize) == EOF)
			{
				fprintf(_DFILE, "Bad input line: %d, while reading observations size!\n", lineCount);
				return false;
			}
		}


		//读取匹配帧
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int obsIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//如果buf[k]字节不为空
				while (bufferp[k] && isspace(bufferp[k])) k++;//若果buf[k]有字节且该字节是‘/t‘空格，略过该空格
				if (bufferp[k])
				{//如果buf[k]有字节且该字节不是‘/t‘空格且输入参数j>=0
					if (sscanf(bufferp + k, "%d", &obsIndex) == EOF)
					{
						fprintf(_DFILE, "Bad input line: %d, while reading map points'observations!\n", lineCount);
						return false;
					}
					mptMathcedFrames[mpt].emplace_back(obsIndex);
				}
				while (bufferp[k] && !isspace(bufferp[k]))
				{//定位k为该数值后一个空格符号的索引
					k++;
				}
			}
			if (obsSize != mptMathcedFrames[mpt].size())
			{
				fprintf(_DFILE, "Observations size is not matched ,line: %d\n", lineCount);
				return false;
			}
		}


		//读取匹配关键帧数
		int obsKFSize = 0;
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &obsKFSize) == EOF)
			{
				fprintf(_DFILE, "Bad input line: %d\n", lineCount);
				return false;
			}
		}

		//读取匹配关键帧	
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int obsKFIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//如果buf[k]字节不为空
				while (bufferp[k] && isspace(bufferp[k])) k++;//若果buf[k]有字节且该字节是‘/t‘空格，略过该空格
				if (bufferp[k])
				{//如果buf[k]有字节且该字节不是‘/t‘空格且输入参数j>=0
					if (sscanf(bufferp + k, "%d", &obsKFIndex) == EOF)
					{
						fprintf(_DFILE, "Bad input line: %d\n", lineCount);
						return false;
					}
					mptMathcedKFs[mpt].emplace_back(obsKFIndex);
				}
				while (bufferp[k] && !isspace(bufferp[k]))
				{//定位k为该数值后一个空格符号的索引
					k++;
				}
			}
			if (mptMathcedKFs[mpt].size() != obsKFSize)
			{
				fprintf(_DFILE, "KeyFrame observations size is not matched ,line: %d\n", lineCount);
				return false;
			}
		}
	}

	///////////////////////读取全局帧///////////////////////

	typedef std::map<Frame*, std::vector<int>> FrameVectorMap;
	typedef std::map<KeyFrame*, std::vector<int>> KFVectorMap;
	typedef std::map<Frame*, int> FrameIntMap;
	typedef std::map<KeyFrame*, int> KFIntMap;
	FrameVectorMap frameCovisMap;
	KFVectorMap kFCovisMap;
	KFIntMap kFParentMap;
	FrameIntMap kFRefMap;
	//读取帧数
	{
		while (!readLine(_INFILE, buffer, 10239))
			lineCount++;
		lineCount++;
		int obsSize = 0;
		if (sscanf(buffer, "%d", &frameSize) == EOF)
		{
			fprintf(_DFILE, "Bad input line: %d, while reading observationgs size!\n", lineCount);
			return false;
		}
	}
	
	//遍历读取帧
	for (int i = 0; i < frameSize; i++)
	{
		Frame* pFrame = new Frame(-1, camera.width(),camera.height());
		pFrame->setCamera(&camera);
		pFrame->setGlobalMap(&globalMap);
		pFrame->setSystem(&system);

		int kFIndex = -1;
		int refKFIndex = -1;
		//读取帧基本信息
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;


			int index = 0;
			cv::Mat Rcw = cv::Mat::zeros(3, 3, CV_64F);
			cv::Mat tcw = cv::Mat::zeros(3, 1, CV_64F);
			char* bufferp = buffer;
			if (sscanf(bufferp, "%d%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
				&index,
				&kFIndex,
				&refKFIndex,
				&Rcw.at<double>(0, 0), &Rcw.at<double>(0, 1), &Rcw.at<double>(0, 2),
				&Rcw.at<double>(1, 0), &Rcw.at<double>(1, 1), &Rcw.at<double>(1, 2),
				&Rcw.at<double>(2, 0), &Rcw.at<double>(2, 1), &Rcw.at<double>(2, 2),
				&tcw.at<double>(0), &tcw.at<double>(1), &tcw.at<double>(2)) != 15)
			{
				fprintf(stderr, "Syntax error reading line %d in file %s, while reading map point's info!\n", lineCount, filePath);
				fclose(_INFILE);
				return false;
			}

			pFrame->setID(index);
			pFrame->setPose(Rcw, tcw);
			pFrame->updatePoseMatrices();
			kFRefMap[pFrame] = refKFIndex;
		}


		//读取帧匹配点数量（3d点，2d点，地图点）
		int p3DSize = 0;
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &p3DSize) == EOF)
			{
				fprintf(_DFILE, "Bad input line: %d, while reading frame's matched points size!\n", lineCount);
				return false;
			}
		}
		//遍历读取帧匹配3d点和地图点
		for (int i = 0; i < p3DSize; i++)
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;
			char* bufferp = buffer;

			cv::Point3d* p3D = new cv::Point3d;
			KeyPoint* kptLeft = new KeyPoint;
			KeyPoint* kptRight = new KeyPoint;
			int kptLeftIndex(0), kptRightIndex(0);
			double kptLeftUnX(0), kptLeftUnY(0), kptRightUnX, kptRightUnY(0);
			double kptLeftWeight(0), kptRightWeight(0);
			cv::Mat normLeft = cv::Mat::zeros(3, 1, CV_64F);
			cv::Mat normRight = cv::Mat::zeros(3, 1, CV_64F);
			cv::RotatedRect rectLeft, rectRight;
			int mptMatchedIndex = -1;
			if (sscanf(bufferp, "%lf%lf%lf%d%lf%lf%lf%lf%lf%f%f%f%f%f%lf%lf%lf%d%lf%lf%lf%lf%lf%f%f%f%f%f%lf%lf%lf%d",
				&p3D->x, &p3D->y, &p3D->z,
				&kptLeftIndex,
				&kptLeft->x, &kptLeft->y,
				&kptLeftUnX, &kptLeftUnY,
				&kptLeftWeight,
				&(rectLeft.angle), &(rectLeft.center.x), &rectLeft.center.y, &rectLeft.size.width, &rectLeft.size.height,
				&normLeft.at<double>(0), &normLeft.at<double>(1), &normLeft.at<double>(2),
				&kptRightIndex,
				&kptRight->x, &kptRight->y,
				&kptRightUnX, &kptRightUnY,
				&kptRightWeight,
				&rectRight.angle, &rectRight.center.x, &rectRight.center.y, &rectRight.size.width, &rectRight.size.height,
				&normRight.at<double>(0), &normRight.at<double>(1), &normRight.at<double>(2),
				&mptMatchedIndex) != 32)
			{
				fprintf(stderr, "Syntax error reading line %d in file %s, while reading frame's matched points!\n", lineCount, filePath);
				fclose(_INFILE);
				return false;
			}
			kptLeft->setID(kptLeftIndex);
			kptLeft->setUndistorX(kptLeftUnX);
			kptLeft->setUndistorY(kptLeftUnY);
			kptLeft->setWeight(kptLeftWeight);
			kptLeft->setRect(rectLeft);
			kptLeft->setBestNorm(normLeft);

			kptRight->setID(kptRightIndex);
			kptRight->setUndistorX(kptRightUnX);
			kptRight->setUndistorY(kptRightUnY);
			kptRight->setWeight(kptRightWeight);
			kptRight->setRect(rectRight);
			kptRight->setBestNorm(normRight);

			pFrame->addKeyPointsLeft(kptLeft);
			pFrame->addKeyPointsRight(kptRight);
			if (mptMatchedIndex != -1)
				for (auto mptItr = mptList.begin(); mptItr != mptList.end(); mptItr++)
				{
					if ((*mptItr)->getID() == mptMatchedIndex)
						pFrame->insertMatchedMPt(*mptItr, kptLeft, kptRight);
				}

			pFrame->insert3DPoints(p3D, kptLeft, kptRight);

		}

		//读取帧离散匹配点数量
		int outLierSize = 0;
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &outLierSize) == EOF)
			{
				fprintf(_DFILE, "Bad input line: %d, while reading frame's outlier matched size!\n", lineCount);
				return false;
			}
		}
		if (outLierSize > 0)
		{//读取帧离散匹配地图点
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int outlierIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//如果buf[k]字节不为空
				while (bufferp[k] && isspace(bufferp[k])) k++;//若果buf[k]有字节且该字节是‘/t‘空格，略过该空格
				if (bufferp[k])
				{//如果buf[k]有字节且该字节不是‘/t‘空格且输入参数j>=0
					if (sscanf(bufferp + k, "%d", &outlierIndex) == EOF)
					{
						fprintf(_DFILE, "Bad input line: %d, while reading frame's outlier matched!\n", lineCount);
						return false;
					}
					for (auto mptItr = mptList.begin(); mptItr != mptList.end(); mptItr++)
					{
						if ((*mptItr)->getID() == outlierIndex)
							pFrame->insertOutLier(*mptItr);
					}
				}
				while (bufferp[k] && !isspace(bufferp[k]))
				{//定位k为该数值后一个空格符号的索引
					k++;
				}
			}
			if (pFrame->getOutLiers().size() != outLierSize)
			{
				fprintf(_DFILE, "Bad input line: %d, Outliers size not matched!\n", lineCount);
				return false;
			}
		}


		//读取共点帧数量
		int covisSize = 0;
		{
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &covisSize) == EOF)
			{
				fprintf(_DFILE, "Bad input line: %d, while reading covisible frames size!\n", lineCount);
				return false;
			}
		}
		if (covisSize > 0)
		{//遍历读取共点帧
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int covisIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//如果buf[k]字节不为空
				while (bufferp[k] && isspace(bufferp[k])) k++;//若果buf[k]有字节且该字节是‘/t‘空格，略过该空格
				if (bufferp[k])
				{//如果buf[k]有字节且该字节不是‘/t‘空格且输入参数j>=0
					if (sscanf(bufferp + k, "%d", &covisIndex) == EOF)
					{
						fprintf(_DFILE, "Bad input line: %d, while reading covisible frames!\n", lineCount);
						return false;
					}

					frameCovisMap[pFrame].emplace_back(covisIndex);
				}
				while (bufferp[k] && !isspace(bufferp[k]))
				{//定位k为该数值后一个空格符号的索引
					k++;
				}
			}
			if (frameCovisMap[pFrame].size() != covisSize)
			{
				fprintf(_DFILE, "Bad input line: %d, Covisible frames size not matched!\n", lineCount);
				return false;
			}
		}

		if (kFIndex != -1)
		{//若是关键帧
			KeyFrame* pKF = new KeyFrame(*pFrame);
			pKF->setID(kFIndex);

			//读取父节点id
			while (!readLine(_INFILE, buffer, 10239))
				lineCount++;
			lineCount++;
			int parentKFid = -1;
			if (sscanf(buffer, "%d", &parentKFid) == EOF)
			{
				fprintf(_DFILE, "Bad input line: %d, while reading parent keyframe index!\n", lineCount);
				return false;
			}
			kFParentMap[pKF] = parentKFid;

			//读取共点关键帧数量
			int covisKFSize = 0;
			{
				while (!readLine(_INFILE, buffer, 10239))
					lineCount++;
				lineCount++;

				if (sscanf(buffer, "%d", &covisKFSize) == EOF)
				{
					fprintf(_DFILE, "Bad input line: %d, while reading covisible keyframes size!\n", lineCount);
					return false;
				}
			}
			if (covisKFSize != 0)
			{//遍历读取共点关键帧
				while (!readLine(_INFILE, buffer, 10239))
					lineCount++;
				lineCount++;
				k = 0;
				int covisKFIndex = 0;
				char* bufferp = buffer;
				while (bufferp[k])
				{//如果buf[k]字节不为空
					while (bufferp[k] && isspace(bufferp[k])) k++;//若果buf[k]有字节且该字节是‘/t‘空格，略过该空格
					if (bufferp[k])
					{//如果buf[k]有字节且该字节不是‘/t‘空格且输入参数j>=0
						if (sscanf(bufferp + k, "%d", &covisKFIndex) == EOF)
						{
							fprintf(_DFILE, "Bad input line: %d, while reading covisible keyframes!\n", lineCount);
							return false;
						}

						kFCovisMap[pKF].emplace_back(covisKFIndex);
					}
					while (bufferp[k] && !isspace(bufferp[k]))
					{//定位k为该数值后一个空格符号的索引
						k++;
					}
				}
				if (kFCovisMap[pKF].size() != covisKFSize)
				{
					fprintf(_DFILE, "Bad input line: %d, covisible keyFrames size not matched!\n", lineCount);
					return false;
				}
			}
			globalMap.addFrame(pKF);
			globalMap.addKeyFrame(pKF);
		}
		else
		{
			globalMap.addFrame(pFrame);
		}
	}


	for (auto itr = mptMathcedFrames.begin(); itr != mptMathcedFrames.end(); itr++)
	{
		MapPoint3d* mpt = itr->first;
		auto observationIDs = itr->second;
		for (auto oItr = observationIDs.begin(); oItr != observationIDs.end(); oItr++)
		{
			int index = *oItr;
			FrameList& frameList = globalMap.getFrameList();
			for (auto fItr = frameList.begin(); fItr != frameList.end(); fItr++)
			{
				if((*fItr)->getID()== index)
					mpt->addObservation(*fItr);
			}
			
		}
	}
	for (auto itr = mptMathcedKFs.begin(); itr != mptMathcedKFs.end(); itr++)
	{
		MapPoint3d* mpt = itr->first;
		auto observationKFIDs = itr->second;
		for (auto oItr = observationKFIDs.begin(); oItr != observationKFIDs.end(); oItr++)
		{
			int index = *oItr;
			KeyFrameList& kFList = globalMap.getKeyFrameList();
			for (auto fItr = kFList.begin(); fItr != kFList.end(); fItr++)
			{
				if ((*fItr)->getID() == index)
					mpt->addKFObservation(*fItr);
			}	
		}
	}
	for (auto itr = mptRefKFIDMap.begin(); itr != mptRefKFIDMap.end(); itr++)
	{//设置地图点参考（起始）关键帧
		KeyFrameList& kFList = globalMap.getKeyFrameList();
		for (auto fItr = kFList.begin(); fItr != kFList.end(); fItr++)
		{
			if ((*fItr)->getID() == itr->second)
			{
				itr->first->setRefKF(*fItr);
				(*fItr)->addNewMapPoint(itr->first);
			}
		}
	}
	for (auto itr = kFParentMap.begin(); itr != kFParentMap.end(); itr++)
	{
		KeyFrameList& kFList = globalMap.getKeyFrameList();
		for (auto kFItr = kFList.begin(); kFItr != kFList.end(); kFItr++)
		{
			if ((*kFItr)->getID() == itr->second)
			{
				itr->first->setParentKF(*kFItr);
				(*kFItr)->insertChildrenKF(itr->first);
			}
		}
	}
	for (auto itr = kFRefMap.begin(); itr != kFRefMap.end(); itr++)
	{
		KeyFrameList& kFList = globalMap.getKeyFrameList();
		for (auto kFItr = kFList.begin(); kFItr != kFList.end(); kFItr++)
		{
			if ((*kFItr)->getID() == itr->second)
			{
				itr->first->setRefKF(*kFItr);
				(*kFItr)->addLocalFrame(itr->first);
				itr->first->calcMotionModel(**kFItr);
			}
		}
	}
	KeyFrameList& kFList = globalMap.getKeyFrameList();
	for (auto kFItr = kFList.begin(); kFItr != kFList.end(); kFItr++)
		(*kFItr)->updateKFConnections();
	
	Frame* lastFrame = globalMap.getFrameList().back();
	KeyFrame* lastKF = dynamic_cast<KeyFrame*>(lastFrame);
	system.getTracker().setRefFrame(lastFrame);
	if(lastKF)
		system.getTracker().setRefKF(lastKF);
	else
		system.getTracker().setRefKF(lastFrame->_refKF);

	GlobalMap::setMptCount(globalMap.getMapPointList().back()->getID()+1);
	GlobalMap::setFrameCount(globalMap.getFrameList().back()->getID() + 1);
	GlobalMap::setKeyFrameCount(globalMap.getKeyFrameList().back()->getID() + 1);
	return true;
}


bool ReadWriter::saveMapFile(std::string filePath, GlobalMap& globalMap)
{
	std::ofstream ofs = std::ofstream(filePath, std::ios::trunc | std::ios::in);
	if (!ofs.is_open())
		return false;
	const MptList& mptList = globalMap.getMapPointList();
	if (mptList.empty())
		return true;
	ofs << mptList.size() << std::endl;
	for (MptList::const_iterator itr = mptList.begin(); itr != mptList.end(); itr++)
	{
		MapPoint3d* mpt = *itr;
		cv::Mat& normal = mpt->getNormLast();
		ofs << std::setprecision(8)
			<< mpt->getID() << " "
			<< mpt->x << " " << mpt->y << " " << mpt->z << " "
			<< normal.at<double>(0) << " " << normal.at<double>(1) << " " << normal.at<double>(2) << " "
			<< mpt->outliersCount() << " "
			<< mpt->getReferenceKeyFrame()->getID() << " "
			<< std::endl;
		
		const FrameSet& observations = mpt->getObservations();
		ofs << observations.size() << std::endl;
		for (auto oItr = observations.begin(); oItr != observations.end(); oItr++)
		{
			ofs << (*oItr)->getID() << " ";
		}
		ofs << std::endl;
		const KFSet& observationKFs = mpt->getObservationKFs();
		ofs << observationKFs.size() << std::endl;
		for (auto oKFItr = observationKFs.begin(); oKFItr != observationKFs.end(); oKFItr++)
		{ 
			ofs << (*oKFItr)->getID() << " ";
		}
		ofs << std::endl;
	}


	const FrameList& frameList = globalMap.getFrameList();
	ofs << frameList.size() << std::endl;
	for (FrameList::const_iterator fItr = frameList.begin(); fItr != frameList.end(); fItr++)
	{
		Frame* pFrame = *fItr;
		KeyFrame* pKF = dynamic_cast<KeyFrame*>(pFrame);
		const cv::Mat Rcw = pFrame->getRotation();
		const cv::Mat tcw = pFrame->getTranslation();

		ofs << std::setprecision(8)
			<< pFrame->getID() << " "
			<< (int)(pKF ? pKF->getID() : -1) << " "
			<< (int)(pFrame->getRefKF() ? pFrame->getRefKF()->getID() : -1) << " "
			<< Rcw.at<double>(0, 0) << " " << Rcw.at<double>(0, 1) << " " << Rcw.at<double>(0, 2) << " "
			<< Rcw.at<double>(1, 0) << " " << Rcw.at<double>(1, 1) << " " << Rcw.at<double>(1, 2) << " " 
			<< Rcw.at<double>(2, 0) << " " << Rcw.at<double>(2, 1) << " " << Rcw.at<double>(2, 2) << " "
			<< tcw.at<double>(0) << " " << tcw.at<double>(1) << " " << tcw.at<double>(2)
			<< std::endl;

		auto& p3DMap = pFrame->getP3DMap();
		ofs << p3DMap.size() << std::endl;
		for (auto pItr = p3DMap.begin(); pItr != p3DMap.end(); pItr++)
		{
			cv::Point3d* p3d = pItr->first;
			KeyPoint* kptLeft = pItr->second.first;
			KeyPoint* kptRight = pItr->second.second;
			const cv::Mat& normLeft = kptLeft->getBestNorm();
			const cv::Mat& normRight = kptRight->getBestNorm();
			MapPoint3d* mptMatched = kptLeft->getMatchedMpt();
			ofs << p3d->x << " " << p3d->y << " " << p3d->z << " "
				<< kptLeft->getID() << " "
				<< kptLeft->x << " " << kptLeft->y << " "
				<< kptLeft->getUndistortX() << " " << kptLeft->getUndistortY() << " "
				<< kptLeft->getWeight() << " "
				<< kptLeft->getRect().angle <<" "<< kptLeft->getRect().center.x<<" "<< kptLeft->getRect().center.y<<" "
				<< kptLeft->getRect().size.width <<" "<<kptLeft->getRect().size.height << " "
				<< normLeft.at<double>(0) << " " << normLeft.at<double>(1) << " " << normLeft.at<double>(2) << " "
				<< kptRight->getID() << " "
				<< kptRight->x << " " << kptRight->y << " "
				<< kptRight->getUndistortX() << " " << kptRight->getUndistortY() << " "
				<< kptRight->getWeight() << " "
				<< kptRight->getRect().angle << " " << kptRight->getRect().center.x <<" "<< kptLeft->getRect().center.y <<" "
				<< kptRight->getRect().size.width << " " << kptRight->getRect().size.height << " "
				<< normRight.at<double>(0) << " " << normRight.at<double>(1) << " " << normRight.at<double>(2) << " "
				<< (int)(mptMatched? mptMatched->getID():-1)
				<< std::endl;
				
		}

		auto& outliers = pFrame->getOutLiers();
		ofs << outliers.size() << std::endl;
		for (auto oItr = outliers.begin(); oItr != outliers.end(); oItr++)
		{
			ofs << (*oItr)->getID() << " ";
		}
		ofs << std::endl;

		auto& covisibles = pFrame->getCovisibles();
		ofs << covisibles.size() << std::endl;
		for (auto cItr = covisibles.begin(); cItr != covisibles.end(); cItr++)
		{
			ofs << (*cItr)->getID() << " ";
		}
		ofs << std::endl;
			
		if (pKF)
		{

			KeyFrame* parentKF = pKF->getParentKF();
			ofs << (int)(parentKF ? parentKF->getID():-1)  << std::endl;
	
			auto& covisibleKFs = pKF->getCovisibleKFs();
			ofs << covisibleKFs.size() << std::endl;
			for (auto cKFItr = covisibleKFs.begin(); cKFItr != covisibleKFs.end(); cKFItr++)
			{
				ofs << (*cKFItr)->getID() << " ";
			}
			ofs << std::endl;
		}		
	}
	return true;
}


bool ReadWriter::loadFrameWithP3DFile(const std::string filePath, SLAM::FrameList& frameList)
{
	unsigned int frameCount = 0;
	cv::Mat K_l, K_r, D_l, D_r, R_l, R_r, t_l, t_r;

	if (!(_INFILE = efopen(filePath.c_str(), "r")))
		return false;
	static int lineCount(0);
	char buffer[1024];
	int pointCount = 0;

	while (fgets(buffer, 1023, _INFILE))
	{
		lineCount++;//有效行数
					//跳过开头空格
		char* bufferp = buffer;

		while (isspace(*bufferp)) bufferp++;

		//跳过空格和注释
		if (*bufferp == '#' || *bufferp == '\0')
		{
			lineCount--;
			continue;
		}

		if (lineCount == 1)
		{
			cv::Mat rotateRec = cv::Mat(1, 3, CV_64F);
			K_l = cv::Mat::zeros(3, 3, CV_64F);
			D_l = cv::Mat::zeros(1, 5, CV_64F);
			R_l = cv::Mat::zeros(3, 3, CV_64F);
			t_l = cv::Mat::zeros(1, 3, CV_64F);
			//double a= K_l.at<double>(0, 0);
			//if (sscanf(bufferp, "%lf", &K_l.at<double>(0, 0)) != 15)

			if (sscanf(bufferp, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
				&K_l.at<double>(0, 0), &K_l.at<double>(1, 1), &K_l.at<double>(0, 2), &K_l.at<double>(1, 2),
				&D_l.at<double>(0, 0), &D_l.at<double>(0, 1), &D_l.at<double>(0, 2), &D_l.at<double>(0, 3), &D_l.at<double>(0, 4),
				&rotateRec.at<double>(0, 0), &rotateRec.at<double>(0, 1), &rotateRec.at<double>(0, 2),
				&t_l.at<double>(0, 0), &t_l.at<double>(0, 1), &t_l.at<double>(0, 2)) != 15)
			{
				fprintf(stderr, "Syntax error reading header on line %d in file %s\n", lineCount, filePath);
				fclose(_INFILE);
				return NULL;
			}
			K_l.at<double>(2, 2) = 1.f;
			cv::Rodrigues(rotateRec, R_l);
		}
		else if (lineCount == 2)
		{
			cv::Mat rotateRec = cv::Mat(1, 3, CV_64F);
			K_r = cv::Mat::zeros(3, 3, CV_64F);
			D_r = cv::Mat::zeros(1, 5, CV_64F);
			R_r = cv::Mat::zeros(3, 3, CV_64F);
			t_r = cv::Mat::zeros(1, 3, CV_64F);
			if (sscanf(bufferp, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
				&K_r.at<double>(0, 0), &K_r.at<double>(1, 1), &K_r.at<double>(0, 2), &K_r.at<double>(1, 2),
				&D_r.at<double>(0, 0), &D_r.at<double>(0, 1), &D_r.at<double>(0, 2), &D_r.at<double>(0, 3), &D_r.at<double>(0, 4),
				&rotateRec.at<double>(0, 0), &rotateRec.at<double>(0, 1), &rotateRec.at<double>(0, 2),
				&t_r.at<double>(0, 0), &t_r.at<double>(0, 1), &t_r.at<double>(0, 2)) != 15)
			{
				fprintf(stderr, "Syntax error reading header on line %d in file %s\n", lineCount, filePath);
				fclose(_INFILE);
				return NULL;
			}
			K_r.at<double>(2, 2) = 1.f;
			cv::Rodrigues(rotateRec, R_r);
		}

		else if (lineCount == 3)
		{
			sscanf(bufferp, "%d", &frameCount);
		}

		else if (lineCount<frameCount + 4)
		{
			sscanf(bufferp, "%d", &pointCount);

			int framID = lineCount - 4;
			SLAM::Frame* frame = new SLAM::Frame(framID, 1280, 1024);

			//frame以左相机帧为准
			K_l.copyTo(frame->getCamera()->Kl());
			K_r.copyTo(frame->getCamera()->Kr());
			D_l.copyTo(frame->getCamera()->Dl());
			D_r.copyTo(frame->getCamera()->Dr());
			if (frame)
			{
				for (int i = 0; i < pointCount; i++)
				{
					fgets(buffer, 1023, _INFILE);

					cv::Point3d* Point3D = new cv::Point3d;
					SLAM::KeyPoint* keyPointL = new SLAM::KeyPoint(i);
					SLAM::KeyPoint* keyPointR = new SLAM::KeyPoint(i);

					if (sscanf(buffer, "%lf%lf%lf%lf%lf%lf%lf",
						&Point3D->x, &Point3D->y, &Point3D->z,
						&keyPointL->x, &keyPointL->y,
						&keyPointR->x, &keyPointR->y) != 7)
					{
						fprintf(stderr, "Syntax error reading header on line %d in file %s\n", lineCount, filePath);
						fclose(_INFILE);
						return NULL;
					}
					//keyPointL->setMatchedP3D(Point3D);
					//keyPointR->setMatchedP3D(Point3D);


					frame->addKeyPointsLeft(keyPointL);
					frame->addKeyPointsRight(keyPointR);
					frame->insert3DPoints(Point3D, keyPointL, keyPointR);

					frame->undistortKeyPoint();

					//此处可添加图像分割功能，判断分割出的每一个瓦片内的特征点集合，便于后续特征点匹配时搜索

				}
			}

			//globalFrameSet.insert(std::make_pair(framID, frame));
			frameList.emplace_back(frame);
		}

	}
	return true;
}


bool ReadWriter::loadPosFile(const std::string filePath, MatMap& rotationMap, MatMap& transMap)
{
	if (!(_INFILE = efopen(filePath.c_str(), "r")))
		return false;
	static int lineCount(0);
	char buffer[1024];
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
	int frameId;

	while (fgets(buffer, 1023, _INFILE))
	{
		lineCount++;//有效行数
					//跳过开头空格
		char* bufferp = buffer;

		while (isspace(*bufferp)) bufferp++;

		//跳过空格和注释
		if (*bufferp == '#' || *bufferp == '\0')
		{
			lineCount--;
			continue;
		}
		if (sscanf(bufferp, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
			&frameId,
			&R.at<double>(0, 0), &R.at<double>(0, 1), &R.at<double>(0, 2),
			&R.at<double>(1, 0), &R.at<double>(1, 1), &R.at<double>(1, 2),
			&R.at<double>(2, 0), &R.at<double>(2, 1), &R.at<double>(2, 2),
			&t.at<double>(0), &t.at<double>(1), &t.at<double>(2)) != 13)
		{
			fprintf(stderr, "Syntax error reading line %d in file %s\n", lineCount, filePath);
			fclose(_INFILE);
			return false;
		}

		rotationMap[frameId]=R.clone();
		transMap[frameId]=t.clone();
	}
	return true;
}


bool ReadWriter::loadAscPtCloudFile(const std::string filePath, MatList& x3DList, MatList& normList, ScalarList& colorList)
{/*
 std::ifstream ifs(filePath);
 if (!ifs.is_open())
 return false;
 cv::Mat x3d = cv::Mat::zeros(1, 3, CV_64F);
 cv::Mat norm3d = cv::Mat::zeros(1, 3, CV_64F);
 cv::Scalar color = cv::Scalar();

 std::string lineStr;
 while (std::getline(ifs, lineStr))
 {
 std::istringstream istream(lineStr);
 std::string str;
 int i = 0;
 while (istream >> str)
 {
 if (i < 3)
 {
 x3d.at<double>(i++) = atof(str.data());
 }
 else if (i >= 3 && i < 6)
 {
 norm3d.at<double>(i-3) = atof(str.data());
 i++;
 }
 else if (i >= 6 && i < 9)
 {
 color.val[i - 3] = atof(str.data());
 i++;
 }
 else
 break;
 }
 x3DList.emplace_back(x3d);
 normList.emplace_back(norm3d);
 colorList.emplace_back(color);
 }
 return true;*/
	if (!(_INFILE = efopen(filePath.c_str(), "r")))
		return false;
	static int lineCount(0);
	char buffer[1024];
	cv::Mat x3d = cv::Mat::zeros(1, 3, CV_64F);
	cv::Mat norm3d = cv::Mat::zeros(1, 3, CV_64F);
	cv::Scalar color = cv::Scalar();

	while (fgets(buffer, 1023, _INFILE))
	{
		lineCount++;//有效行数
					//跳过开头空格
		char* bufferp = buffer;

		while (isspace(*bufferp)) bufferp++;

		//跳过空格和注释
		if (*bufferp == '#' || *bufferp == '\0')
		{
			lineCount--;
			continue;
		}
		/*if (sscanf(bufferp, "%lf%lf%lf%lf%lf%lf",
			&x3d.at<double>(0), &x3d.at<double>(1), &x3d.at<double>(2),
			color.val, &color.val[1], &color.val[2]) != 6)
		{
			fprintf(stderr, "Syntax error reading line %d in file %s\n", lineCount, filePath);
			fclose(_INFILE);
			return false;
		}*/
		
		 if (sscanf(bufferp, "%lf%lf%lf%lf%lf%lf%lf%lf%lf",
					&x3d.at<double>(0), &x3d.at<double>(1), &x3d.at<double>(2),
					&norm3d.at<double>(0), &norm3d.at<double>(1), &norm3d.at<double>(2),
					color.val, &color.val[1], &color.val[2]) != 9)
		 {
			fprintf(stderr, "Syntax error reading line %d in file %s\n", lineCount, filePath);
			fclose(_INFILE);
			return false;
		 }
		x3DList.emplace_back(x3d.clone());
		normList.emplace_back(norm3d.clone());
		colorList.emplace_back(color);
	}
	return true;
}


bool ReadWriter::saveAscPtCloudFile(const std::string filePath, const MatList& x3DList, const MatList& normList, const ScalarList& colorList)
{
	std::ofstream ofs = std::ofstream(filePath, std::ios::trunc | std::ios::in);
	if (!ofs.is_open())
	{
		std::cout << "Failed opening the asc file to write: " << filePath << std::endl;
		return false;
	}
	int ptSize = x3DList.size();


	for (int i = 0; i < ptSize; i++)
	{
	/*	ofs << std::setprecision(8) << x3DList[i].at<double>(0) << " " << x3DList[i].at<double>(1) << " " << x3DList[i].at<double>(2)
			<< " " << colorList[i][0] << " " << colorList[i][1] << " " << colorList[i][2] << std::endl;*/
		ofs << std::setprecision(8) << x3DList[i].at<double>(0) << " " << x3DList[i].at<double>(1) << " " << x3DList[i].at<double>(2)
			<< " " << normList[i].at<double>(0) << " " << normList[i].at<double>(1) << " " << normList[i].at<double>(2)
			<< " " << colorList[i][0] << " " << colorList[i][1] << " " << colorList[i][2] << std::endl;
	}
	return true;
}


bool ReadWriter::getFileListFromDir(const std::string& dir, FilePathStrMap& filePathList)
{
	char drive[_MAX_DRIVE];
	char secDir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];

	if (dir.empty())
		return false;
	char buffer[_MAX_PATH + 100] = { 0 };
	__finddata64_t fileinfo;

	std::string strFullName = dir;
	if (strFullName.back() != '\\') strFullName += '\\'; // 如果不是以'\\'结尾的文件路径,则补齐. 注意一个原则:URL以正斜杠分隔;文件名以反斜杠分隔
	strFullName += "*";

	intptr_t findRet = _findfirst64(strFullName.c_str(), &fileinfo);
	if (-1 != findRet)
	{
		do
		{
			// 跳过 . 文件
			if (stricmp(fileinfo.name, ".") == 0 || 0 == stricmp(fileinfo.name, ".."))
			{
				continue;
			}
			// 跳过系统文件和隐藏文件
			if (fileinfo.attrib & _A_SYSTEM || fileinfo.attrib & _A_HIDDEN)
			{
				continue;
			}

			// 输出子目录或者
			if (fileinfo.attrib & _A_SUBDIR)
			{
				continue;
			}
			else
			{
				// 普通文件,写入到一个缓冲的字符串string变量内,循环外再合并.这样,所有的目录都在前面,文件在后面
				_ctime64_s(buffer, _countof(buffer), &fileinfo.time_write);

				std::string fileName = fileinfo.name;

				_splitpath(fileName.c_str(), drive, secDir, fname, ext);
				//if(strcmp(ext, ".bmp"))
				//continue;

				std::string filePath;
				if (dir.back() != '\\')
					filePath = dir + '\\' + fileName;
				else
					filePath = dir + fileName;

				filePathList.insert(std::make_pair(fname, filePath));

			}
		} while (-1 != _findnext64(findRet, &fileinfo));

		_findclose(findRet);
	}
	return true;
}


bool ReadWriter::getFileListFromDir(const std::string& dir, FilePathMap& filePathList)
{
	char drive[_MAX_DRIVE];
	char secDir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];

	if (dir.empty())
		return false;
	char buffer[_MAX_PATH + 100] = { 0 };
	__finddata64_t fileinfo;

	std::string strFullName = dir;
	if (strFullName.back() != '\\') strFullName += '\\'; // 如果不是以'\\'结尾的文件路径,则补齐
	strFullName += "*";

	intptr_t findRet = _findfirst64(strFullName.c_str(), &fileinfo);
	if (-1 != findRet)
	{
		do
		{
			// 跳过 . 文件
			if (stricmp(fileinfo.name, ".") == 0 || 0 == stricmp(fileinfo.name, ".."))
			{
				continue;
			}
			// 跳过系统文件和隐藏文件
			if (fileinfo.attrib & _A_SYSTEM || fileinfo.attrib & _A_HIDDEN)
			{
				continue;
			}

			// 跳过子目录
			if (fileinfo.attrib & _A_SUBDIR)
			{
				continue;
			}
			else
			{
				// 普通文件,写入到一个缓冲的字符串string变量内,循环外再合并.这样,所有的目录都在前面,文件在后面
				_ctime64_s(buffer, _countof(buffer), &fileinfo.time_write);

				std::string fileName = fileinfo.name;

				_splitpath(fileName.c_str(), drive, secDir, fname, ext);
			/*	if(strcmp(ext, ".bmp")&& strcmp(ext, ".jpg")&& strcmp(ext, ".png"))
					continue;*/
				
				std::string filePath;
				if (dir.back() != '\\')
					filePath = dir + '\\' + fileName;
				else
					filePath = dir + fileName;

				int number = atoi(fname);
				filePathList.insert(std::make_pair(number, filePath));
	
			}
		} while (-1 != _findnext64(findRet, &fileinfo));

		_findclose(findRet);
	}
	return true;
}


int ReadWriter::getDirListFromDir(std::string& dir, FilePathMap& dirList)
{
	char drive[_MAX_DRIVE];
	char secDir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];

	if (dir.empty())
		return false;
	char buffer[_MAX_PATH + 100] = { 0 };
	__finddata64_t fileinfo;

	if (dir.back() != '\\') dir += '\\';
	std::string strFullName = dir;
	strFullName += "*";

	intptr_t findRet = _findfirst64(strFullName.c_str(), &fileinfo);
	if (-1 != findRet)
	{
		do
		{
			// 跳过 . 文件
			if (stricmp(fileinfo.name, ".") == 0 || 0 == stricmp(fileinfo.name, ".."))
			{
				continue;
			}
			// 跳过系统文件和隐藏文件
			if (fileinfo.attrib & _A_SYSTEM || fileinfo.attrib & _A_HIDDEN)
			{
				continue;
			}

			// 输出子目录或者
			if (fileinfo.attrib & _A_SUBDIR)
			{
				std::string fname = fileinfo.name;


				//if(strcmp(ext, ".bmp"))
				//continue;
				int number;
				if (fname == "0")
					number = 0;
				else
				{
					number = atoi(fileinfo.name);
					if (!number)
						continue;
				}

				std::string dirPath = dir + fileinfo.name;
				dirList.insert(std::make_pair(number, dirPath));
			}
			else
			{
				continue;
			}
		} while (-1 != _findnext64(findRet, &fileinfo));

		_findclose(findRet);
	}
	return true;
}


bool ReadWriter::loadImageList(std::string imgDir,ReadWriter::StrList& leftImgPathList, ReadWriter::StrList& rightImgPathList)
{
	std::string imgLeftDir, imgRightDir;

	if (imgDir.back() != '\\')imgDir += '\\';
	imgLeftDir = imgDir + "L";
	imgRightDir = imgDir + "R";

	SLAM::ReadWriter::FilePathStrMap leftImgPathMap;
	SLAM::ReadWriter::FilePathStrMap rightImgPathMap;
	SLAM::ReadWriter::getFileListFromDir(imgLeftDir, leftImgPathMap);
	SLAM::ReadWriter::getFileListFromDir(imgRightDir, rightImgPathMap);
	for (SLAM::ReadWriter::FilePathStrMap::iterator itr = leftImgPathMap.begin(); itr != leftImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = leftImgPathMap.erase(itr);
		else
			itr++;
	}
	for (SLAM::ReadWriter::FilePathStrMap::iterator itr = rightImgPathMap.begin(); itr != rightImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = rightImgPathMap.erase(itr);
		else
			itr++;
	}
	if (leftImgPathMap.size() != leftImgPathMap.size())
	{
		std::cout << "Wrong input image files sequence" << std::endl;
		return false;
	}

	//cv::Mat M1l, M2l, M1r, M2r;
	//cv::initUndistortRectifyMap(_camera.Kl(), _camera.Dl(), _camera.Rl(), _camera.Pl().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1l, M2l);
	//cv::initUndistortRectifyMap(_camera.Kr(), _camera.Dr(), _camera.Rr(), _camera.Pr().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1r, M2r);



	for (SLAM::ReadWriter::FilePathStrMap::iterator fItr = leftImgPathMap.begin(); fItr != leftImgPathMap.end(); fItr++)
		leftImgPathList.emplace_back(fItr->second);
	for (SLAM::ReadWriter::FilePathStrMap::iterator fItr = rightImgPathMap.begin(); fItr != rightImgPathMap.end(); fItr++)
		rightImgPathList.emplace_back(fItr->second);
	return true;
}


bool ReadWriter::loadHandHeldImageList(std::string imgDir, FilePathMap& leftImgPathMap, FilePathMap& rightImgPathMap)
{
	std::string imgLeftDir, imgRightDir;

	if (imgDir.back() != '\\')imgDir += '\\';
	imgLeftDir = imgDir + "1\\track\\";
	imgRightDir = imgDir + "2\\track\\";

	SLAM::ReadWriter::getFileListFromDir(imgLeftDir, leftImgPathMap);
	SLAM::ReadWriter::getFileListFromDir(imgRightDir, rightImgPathMap);
	for (SLAM::ReadWriter::FilePathMap::iterator itr = leftImgPathMap.begin(); itr != leftImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = leftImgPathMap.erase(itr);
		else
			itr++;
	}
	for (SLAM::ReadWriter::FilePathMap::iterator itr = rightImgPathMap.begin(); itr != rightImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = rightImgPathMap.erase(itr);
		else
			itr++;
	}
	if (leftImgPathMap.size() != leftImgPathMap.size())
	{
		std::cout << "Wrong input image files sequence" << std::endl;
		return false;
	}

	//cv::Mat M1l, M2l, M1r, M2r;
	//cv::initUndistortRectifyMap(_camera.Kl(), _camera.Dl(), _camera.Rl(), _camera.Pl().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1l, M2l);
	//cv::initUndistortRectifyMap(_camera.Kr(), _camera.Dr(), _camera.Rr(), _camera.Pr().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1r, M2r);
	return true;
}


bool ReadWriter::loadHandHeldImageList(std::string imgDir, StrList& leftImgPathList, StrList& rightImgPathList)
{
	std::string imgLeftDir, imgRightDir;

	if (imgDir.back() != '\\')imgDir += '\\';
	imgLeftDir = imgDir + "1\\track\\";
	imgRightDir = imgDir + "2\\track\\";

	SLAM::ReadWriter::FilePathMap leftImgPathMap;
	SLAM::ReadWriter::FilePathMap rightImgPathMap;
	SLAM::ReadWriter::getFileListFromDir(imgLeftDir, leftImgPathMap);
	SLAM::ReadWriter::getFileListFromDir(imgRightDir, rightImgPathMap);
	for (SLAM::ReadWriter::FilePathMap::iterator itr = leftImgPathMap.begin(); itr != leftImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = leftImgPathMap.erase(itr);
		else
			itr++;
	}
	for (SLAM::ReadWriter::FilePathMap::iterator itr = rightImgPathMap.begin(); itr != rightImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = rightImgPathMap.erase(itr);
		else
			itr++;
	}

	if (leftImgPathMap.size() != leftImgPathMap.size())
	{
		std::cout << "Wrong input image files sequence" << std::endl;
		return false;
	}

	//cv::Mat M1l, M2l, M1r, M2r;
	//cv::initUndistortRectifyMap(_camera.Kl(), _camera.Dl(), _camera.Rl(), _camera.Pl().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1l, M2l);
	//cv::initUndistortRectifyMap(_camera.Kr(), _camera.Dr(), _camera.Rr(), _camera.Pr().rowRange(0, 3).colRange(0, 3), cv::Size(_camera.width(), _camera.height()), CV_32F, M1r, M2r);



	for (SLAM::ReadWriter::FilePathMap::iterator fItr = leftImgPathMap.begin(); fItr != leftImgPathMap.end(); fItr++)
		leftImgPathList.emplace_back(fItr->second);
	for (SLAM::ReadWriter::FilePathMap::iterator fItr = rightImgPathMap.begin(); fItr != rightImgPathMap.end(); fItr++)
		rightImgPathList.emplace_back(fItr->second);
	return true;
}


bool ReadWriter::loadHandHeldImage1List(std::string imgDir, FilePathMap& leftImgPathMap, FilePathMap& rightImgPathMap)
{
	std::string imgLeftDir, imgRightDir;

	if (imgDir.back() != '\\')imgDir += '\\';
	imgLeftDir = imgDir + "1\\multi1\\";
	imgRightDir = imgDir + "2\\multi1\\";

	SLAM::ReadWriter::getFileListFromDir(imgLeftDir, leftImgPathMap);
	SLAM::ReadWriter::getFileListFromDir(imgRightDir, rightImgPathMap);

	for (SLAM::ReadWriter::FilePathMap::iterator itr = leftImgPathMap.begin(); itr != leftImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = leftImgPathMap.erase(itr);
		else
			itr++;
	}
	for (SLAM::ReadWriter::FilePathMap::iterator itr = rightImgPathMap.begin(); itr != rightImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = rightImgPathMap.erase(itr);
		else
			itr++;
	}
	if (leftImgPathMap.size() != leftImgPathMap.size())
	{
		std::cout << "Wrong input image files sequence" << std::endl;
		return false;
	}

	return true;
}


bool ReadWriter::loadHandHeldImage2List(std::string imgDir, FilePathMap& leftImgPathMap, FilePathMap& rightImgPathMap)
{
	std::string imgLeftDir, imgRightDir;

	if (imgDir.back() != '\\')imgDir += '\\';
	imgLeftDir = imgDir + "1\\multi2\\";
	imgRightDir = imgDir + "2\\multi2\\";

	SLAM::ReadWriter::getFileListFromDir(imgLeftDir, leftImgPathMap);
	SLAM::ReadWriter::getFileListFromDir(imgRightDir, rightImgPathMap);
	for (SLAM::ReadWriter::FilePathMap::iterator itr = leftImgPathMap.begin(); itr != leftImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = leftImgPathMap.erase(itr);
		else
			itr++;
	}
	for (SLAM::ReadWriter::FilePathMap::iterator itr = rightImgPathMap.begin(); itr != rightImgPathMap.end(); )
	{
		std::string filePath = itr->second;
		int dotIndex = filePath.rfind('.');
		std::string ext = filePath.substr(dotIndex, filePath.length() - dotIndex);
		if (ext != ".bmp" && ext != ".jpg" && ext != ".png")
			itr = rightImgPathMap.erase(itr);
		else
			itr++;
	}
	if (leftImgPathMap.size() != leftImgPathMap.size())
	{
		std::cout << "Wrong input image files sequence" << std::endl;
		return false;
	}

	return true;
}


bool ReadWriter::saveTrackImg(std::string outDir, Tracker* tracker)
{
	Frame& frameCur = *tracker->_currentFrame;
	Frame& framePrev = *tracker->_refKF;
	cv::Mat& canvasImg = tracker->_canvasImg;

	int wl = cvRound(frameCur._markImgLeft.cols);
	if (canvasImg.empty())
		return false;

	char indexStr1[5], indexStr2[5];
	itoa(frameCur.getID(), indexStr1, 10);
	itoa(framePrev.getID(), indexStr2, 10);

	putText(canvasImg, indexStr1, cv::Point(10, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);
	putText(canvasImg, indexStr2, cv::Point(10 + wl, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);
	if (outDir.back() != '\\') outDir += '\\';
	if (access(outDir.c_str(), 0) == -1)
	{
		if (_mkdir(outDir.c_str()))
		{
			std::cout << "Cannot create matched output dir!" << std::endl;
			return false;
		}
	}
	std::string filename;
	char fileName1[6], fileName2[6];
	itoa(frameCur.getID(), fileName1, 10);
	itoa(framePrev.getID(), fileName2, 10);
	filename = std::string(fileName1) + "-" + fileName2;
	if (!cv::imwrite(outDir + filename + ".bmp", canvasImg))
		return false;
	return true;
}


bool ReadWriter::saveMatchedMarkImg(std::string fileDir, Frame* frame)
{
	char fileName[6];
	itoa(frame->_id, fileName, 10);
	char index[5],matchedCount[3];
	itoa(frame->getID(), index, 10);
	itoa(frame->_p3DMap.size(), matchedCount, 10);

	std::string text1 = "Frame: " + std::string(index);
	std::string text2 = "Matched pts: " + std::string(matchedCount);
	putText(frame->_stereoImg, text1, cv::Point(10, 25), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);
	putText(frame->_stereoImg, text2, cv::Point(10, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);
	if (!cv::imwrite(fileDir + fileName + ".bmp", frame->_stereoImg))
		return false;
	return true;
}

}