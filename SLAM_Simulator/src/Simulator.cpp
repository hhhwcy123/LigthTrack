#include "Simulator.h"
#include "System.h"
#include "Converter.h"

#include <random>
#include <ctime>


bool Simulator::loadMapFile()
{
	const std::string& mapFilePath = _simOptions->getMapFilePath();
	typedef std::map<SLAM::MapPoint3d*, std::vector<int>> MptVectorMap;
	char* mode = "r";
	FILE* fp = fopen(mapFilePath.c_str(), mode);
	if (fp == NULL)
	{
		fprintf(stderr, "couldn't open file %s mode %s\n", mapFilePath, mode);
		return false;
	}

	int k = 0;
	static int lineCount(0);
	char buffer[10240];
	typedef std::map<SLAM::MapPoint3d*, int> MptIntMap;
	MptVectorMap mptMathcedFrames, mptMathcedKFs;
	MptIntMap mptRefKFIDMap;
	int mptSize = 0;
	int frameSize = 0;
	int keyFrameSize = 0;

	_realMap.release();

	///////////////////////读取地图点///////////////////////
	//读取地图点全局数量
	{
		while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
			lineCount++;
		lineCount++;
		if (sscanf(buffer, "%d", &mptSize) != 1)
		{
			fprintf(stderr, "Syntax error reading line %d in file %s, while reading map points size!\n", lineCount, mapFilePath);
			fclose(fp);
			return false;
		}
	}

	//遍历读取地图点
	SLAM::MptList& mptList = _realMap.getMapPointList();
	for (int i = 0; i < mptSize; i++)
	{
		SLAM::MapPoint3d* mpt = new SLAM::MapPoint3d();
		mpt->setGlobalMap(&_realMap);

		//读取地图点基本信息
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
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
				fprintf(stderr, "Syntax error reading line %d in file %s, while reading map point's info!\n", lineCount, mapFilePath);
				fclose(fp);
				return false;
			}
			mptRefKFIDMap[mpt] = refKFid;
			mpt->setID(index);
			mpt->getOriCoord() = mpt->getLastCoord() = *mpt;//*dynamic_cast<cv::Point3d*>(mpt);
			mpt->setNormOri(normal);
			mpt->setNormLast(normal);
			mptList.emplace_back(mpt);
			_realMap.addMptDistList(mpt, SLAM::Tracker::P3D_DIST_MAX);
		}

		//读取匹配帧数
		int obsSize = 0;
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &obsSize) == EOF)
			{
				fprintf(stderr, "Bad input line: %d, while reading observations size!\n", lineCount);
				return false;
			}
		}


		//读取匹配帧
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
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
						fprintf(stderr, "Bad input line: %d, while reading map points'observations!\n", lineCount);
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
				fprintf(stderr, "Observations size is not matched ,line: %d\n", lineCount);
				return false;
			}
		}


		//读取匹配关键帧数
		int obsKFSize = 0;
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &obsKFSize) == EOF)
			{
				fprintf(stderr, "Bad input line: %d\n", lineCount);
				return false;
			}
		}

		//读取匹配关键帧	
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
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
						fprintf(stderr, "Bad input line: %d\n", lineCount);
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
				fprintf(stderr, "SLAM::KeyFrame observations size is not matched ,line: %d\n", lineCount);
				return false;
			}
		}
	}

	///////////////////////读取全局帧///////////////////////

	typedef std::map<SLAM::Frame*, std::vector<int>> FrameVectorMap;
	typedef std::map<SLAM::KeyFrame*, std::vector<int>> KFVectorMap;
	typedef std::map<SLAM::Frame*, int> FrameIntMap;
	typedef std::map<SLAM::KeyFrame*, int> KFIntMap;
	FrameVectorMap frameCovisMap;
	KFVectorMap kFCovisMap;
	KFIntMap kFParentMap;
	FrameIntMap kFRefMap;
	//读取帧数
	{
		while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
			lineCount++;
		lineCount++;
		int obsSize = 0;
		if (sscanf(buffer, "%d", &frameSize) == EOF)
		{
			fprintf(stderr, "Bad input line: %d, while reading observationgs size!\n", lineCount);
			return false;
		}
	}

	//遍历读取帧
	for (int i = 0; i < frameSize; i++)
	{
		SLAM::Frame* pFrame = new SLAM::Frame(-1, _realCamera.width(), _realCamera.height());
		pFrame->setCamera(&_realCamera);
		pFrame->setGlobalMap(&_realMap);

		int kFIndex = -1;
		int refKFIndex = -1;
		//读取帧基本信息
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
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
				fprintf(stderr, "Syntax error reading line %d in file %s, while reading map point's info!\n", lineCount, mapFilePath);
				fclose(fp);
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
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &p3DSize) == EOF)
			{
				fprintf(stderr, "Bad input line: %d, while reading frame's matched points size!\n", lineCount);
				return false;
			}
		}
		//遍历读取帧匹配3d点和地图点
		for (int i = 0; i < p3DSize; i++)
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;
			char* bufferp = buffer;

			cv::Point3d* p3D = new cv::Point3d;
			SLAM::KeyPoint* kptLeft = new SLAM::KeyPoint;
			SLAM::KeyPoint* kptRight = new SLAM::KeyPoint;
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
				fprintf(stderr, "Syntax error reading line %d in file %s, while reading frame's matched points!\n", lineCount, mapFilePath);
				fclose(fp);
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
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &outLierSize) == EOF)
			{
				fprintf(stderr, "Bad input line: %d, while reading frame's outlier matched size!\n", lineCount);
				return false;
			}
		}
		if (outLierSize > 0)
		{//读取帧离散匹配地图点
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
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
						fprintf(stderr, "Bad input line: %d, while reading frame's outlier matched!\n", lineCount);
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
				fprintf(stderr, "Bad input line: %d, Outliers size not matched!\n", lineCount);
				return false;
			}
		}


		//读取共点帧数量
		int covisSize = 0;
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;

			if (sscanf(buffer, "%d", &covisSize) == EOF)
			{
				fprintf(stderr, "Bad input line: %d, while reading covisible frames size!\n", lineCount);
				return false;
			}
		}
		if (covisSize > 0)
		{//遍历读取共点帧
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
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
						fprintf(stderr, "Bad input line: %d, while reading covisible frames!\n", lineCount);
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
				fprintf(stderr, "Bad input line: %d, Covisible frames size not matched!\n", lineCount);
				return false;
			}
		}

		if (kFIndex != -1)
		{//若是关键帧
			SLAM::KeyFrame* pKF = new SLAM::KeyFrame(*pFrame);
			pKF->setID(kFIndex);

			//读取父节点id
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;
			int parentKFid = -1;
			if (sscanf(buffer, "%d", &parentKFid) == EOF)
			{
				fprintf(stderr, "Bad input line: %d, while reading parent keyframe index!\n", lineCount);
				return false;
			}
			kFParentMap[pKF] = parentKFid;

			//读取共点关键帧数量
			int covisKFSize = 0;
			{
				while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
					lineCount++;
				lineCount++;

				if (sscanf(buffer, "%d", &covisKFSize) == EOF)
				{
					fprintf(stderr, "Bad input line: %d, while reading covisible keyframes size!\n", lineCount);
					return false;
				}
			}
			if (covisKFSize != 0)
			{//遍历读取共点关键帧
				while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
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
							fprintf(stderr, "Bad input line: %d, while reading covisible keyframes!\n", lineCount);
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
					fprintf(stderr, "Bad input line: %d, covisible keyFrames size not matched!\n", lineCount);
					return false;
				}
			}
			_realMap.addFrame(pKF);
			_realMap.addKeyFrame(pKF);
		}
		else
		{
			_realMap.addFrame(pFrame);
		}
	}

	return true;
}


bool Simulator::processSimFrames()
{
	static std::default_random_engine e(time(0));
	static std::uniform_real_distribution<double> noise(_simOptions->getMinNoiseVal(), _simOptions->getMaxNoiseVal());
	const SimOptions::NoiseType& noiseType =  _simOptions->getNoiseType();
	if (noiseType < 0 || noiseType>1)
		return false;


	SLAM::FrameList& frameList = _realMap.getFrameList();
	for (auto fItr = frameList.begin(); fItr != frameList.end(); fItr++)
	{
		SLAM::Frame* realFrame = *fItr;
		SLAM::Frame* simFrame = new SLAM::Frame(realFrame->getID(), _realCamera.width(), _realCamera.height());
		if (!simFrame)
			return false;
		simFrame->setCamera(&_realCamera);
		SLAM::Feature::MarkerList simMarkerListLeft;
		SLAM::Feature::MarkerList simMarkerListRight;

		double noise3DPerFrame = 0;
		auto& mptMatchedMap = realFrame->getMatchedMptMap();
		for (auto mItr = mptMatchedMap.begin(); mItr != mptMatchedMap.end(); mItr++)
		{
			SLAM::KeyPoint* kptLeftReal = mItr->second.first;
			SLAM::KeyPoint* kptRightReal = mItr->second.second;
			cv::Point3d* mptReal = dynamic_cast<cv::Point3d*>(mItr->first);

			cv::Mat mptMat = cv::Mat(*mptReal);
			cv::Mat x3DRealMat = realFrame->Rcw()*mptMat + realFrame->tcw();
			if (noiseType == SimOptions::noise3D)
			{
				cv::Mat x3DRealMatOri = x3DRealMat.clone();
				for (int i = 0; i < x3DRealMat.rows; i++)
					*x3DRealMat.ptr<double>(i) += noise(e);
				noise3DPerFrame += cv::norm(x3DRealMatOri - x3DRealMat);
			}
				
			

			SLAM::Feature::DotMarker simMarkerLeft;
			SLAM::Feature::DotMarker simMarkerRight;
			cv::Mat x2DLeftMat, x2DRightMat;
			cv::projectPoints(x3DRealMat.reshape(3, 1), cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), _realCamera.Kl(), cv::Mat(), x2DLeftMat);
			cv::projectPoints(x3DRealMat.reshape(3, 1), _realCamera.R(), _realCamera.t(), _realCamera.Kr(), cv::Mat(), x2DRightMat);

			double& leftX = x2DLeftMat.at<cv::Vec2d>(0)[0];
			double& leftY = x2DLeftMat.at<cv::Vec2d>(0)[1];
			double& rightX = x2DRightMat.at<cv::Vec2d>(0)[0];
			double& rightY = x2DRightMat.at<cv::Vec2d>(0)[1];
			kptLeftReal->setUndistorX(leftX);
			kptLeftReal->setUndistorY(leftY);
			kptRightReal->setUndistorX(rightX);
			kptRightReal->setUndistorY(rightY);
			if (noiseType == SimOptions::noise2D)
			{
				simMarkerLeft = { leftX + noise(e) ,leftY + noise(e), kptLeftReal->getAreaSize(),kptLeftReal->getEccentricity() };
				simMarkerRight = { rightX + noise(e) ,rightY + noise(e), kptRightReal->getAreaSize(),kptRightReal->getEccentricity() };
			}
			else
			{
				simMarkerLeft = { leftX, leftY, kptLeftReal->getAreaSize(),kptLeftReal->getEccentricity() };
				simMarkerRight = { rightX, rightY, kptRightReal->getAreaSize(),kptRightReal->getEccentricity() };
			}
			
		
			simMarkerLeft.rect = kptLeftReal->getRect();
			simMarkerRight.rect = kptRightReal->getRect();

			simMarkerListLeft.emplace_back(simMarkerLeft);
			simMarkerListRight.emplace_back(simMarkerRight);
		}

		simFrame->setUndistorted(true);
		if (!simFrame->compute3Dpoints(simMarkerListLeft, simMarkerListRight, 
									   simFrame->calcMatchedFunc(SLAM::Frame::NORMAL),
									   SLAM::Feature::extractMarkFunc(SLAM::Feature::NPP_CUDA)))
			continue;

	
		_simFrameMap[simFrame] = realFrame;
		
	}
	return true;
}


bool Simulator::run()
{
	SLAM::System::instance()->setOutputTrackImgFlag(false);//是否输出帧-帧跟踪匹配图像	
	SLAM::System::instance()->setOutputStereoImgFlag(false);//是否输出单帧左右相机标志点匹配图像
	SLAM::System::instance()->setOutputPosFlag(false);//是否输出每帧位姿到文件
	SLAM::System::instance()->setOutputP3dFlag(false);//是否输出每帧3d点坐标到文件
	SLAM::System::instance()->setDisplay3DModelFlag(true);//是否实时显示3d模型														   //////////////////优化方式选择///////////////
	SLAM::System::instance()->setOptimize3DFlag(true);//是否采用3d-3d优化
	SLAM::System::instance()->setLocalBundleAdjustmentFlag(false);//是否开启子线程局部优化
	SLAM::System::instance()->setDispErrorStats(false);//控制台是否输出每帧位姿、局部优化误差信息
	SLAM::System::instance()->setLoopClosingFlag(true);//是否开启回环检测

	SLAM::System::instance()->release();
	SLAM::System::instance()->initiate();

	std::string& outDir = const_cast<std::string&>(_simOptions->getOutDir()) ;
	if (outDir.back() != '\\')outDir.back() += '\\';
	std::ofstream ofsPoseErr = std::ofstream(outDir +"PoseErr.txt", std::ios::trunc | std::ios::in);
	std::ofstream ofsPoseGBAErr = std::ofstream(outDir + "PoseGBAErr.txt", std::ios::trunc | std::ios::in);

	for (auto itr = _simFrameMap.begin(); itr != _simFrameMap.end();)
	{
		SLAM::Frame* simFrame = itr->first;
		simFrame->setGlobalMap(&SLAM::System::instance()->getGlobalMap());
		simFrame->setSystem(SLAM::System::instance());

		if (!SLAM::System::instance()->getTracker().tracking(*simFrame))
		{
			fprintf(stdout, "Tracking Sim frame %d failed!\n\n", simFrame->getID());
			itr = _simFrameMap.erase(itr);
			SLAM::usleep(20000);
		}
		else
		{
			SLAM::Frame* realFrame = itr->second;
			double shift = cv::norm(realFrame->getCameraCenter() - simFrame->getCameraCenter());

			cv::Mat& RwcReal = realFrame->getRotationInverse();
			Eigen::Matrix3d rotationMatrix = SLAM::Converter::toMatrix3d(RwcReal);
			Eigen::Vector3d eulerAnglesReal = rotationMatrix.eulerAngles(2, 1, 0).transpose();
			eulerAnglesReal = eulerAnglesReal / PI * 180.;
			for (int i = 0; i < eulerAnglesReal.size(); i++)
			{
				if (eulerAnglesReal[i] < 0.) eulerAnglesReal[i] = -eulerAnglesReal[i];
				eulerAnglesReal[i] = 180. - eulerAnglesReal[i]>90. ? eulerAnglesReal[i] : 180. - eulerAnglesReal[i];
			}

			cv::Mat& RwcSim = simFrame->getRotationInverse();
			rotationMatrix = SLAM::Converter::toMatrix3d(RwcSim);
			Eigen::Vector3d eulerAnglesSim = rotationMatrix.eulerAngles(2, 1, 0).transpose();
			eulerAnglesSim = eulerAnglesSim / PI * 180.;
			for (int i = 0; i < eulerAnglesSim.size(); i++)
			{
				if (eulerAnglesSim[i] < 0.) eulerAnglesSim[i] = -eulerAnglesSim[i];
				eulerAnglesSim[i] = 180. - eulerAnglesSim[i]>90. ? eulerAnglesSim[i] : 180. - eulerAnglesSim[i];
			}

			Eigen::Vector3d  angleMat = eulerAnglesSim - eulerAnglesReal;
			double angleErr = angleMat.dot(angleMat);

			ofsPoseErr << simFrame->getID() << " " << shift << " " << angleErr << std::endl;

			fprintf(stdout, "Tracking Sim frame %d succeeded!\n\n", simFrame->getID());
			itr++;
			SLAM::usleep(20000);
		}	
	}

	//全局优化
	SLAM::System::instance()->getLoopCloser().runGlobalBundleAdjustment();

	SLAM::FrameList& realFrameList = _realMap.getFrameList();
	SLAM::FrameList& simFrameList = SLAM::System::instance()->getGlobalFrameList();
	for (auto simItr = simFrameList.begin(); simItr != simFrameList.end(); simItr++)
	{
		SLAM::Frame* simFrame = *simItr;
		SLAM::Frame* realFrame = nullptr;
		for (auto realItr = realFrameList.begin(); realItr != realFrameList.end(); realItr++)
			if ((*realItr)->getID() == simFrame->getID())
				realFrame = *realItr;

		double shift = cv::norm(realFrame->getCameraCenter() - simFrame->getCameraCenter());

		cv::Mat& RwcReal = realFrame->getRotationInverse();
		Eigen::Matrix3d rotationMatrix = SLAM::Converter::toMatrix3d(RwcReal);
		Eigen::Vector3d eulerAnglesReal = rotationMatrix.eulerAngles(2, 1, 0).transpose();
		eulerAnglesReal = eulerAnglesReal / PI * 180.;
		for (int i = 0; i < eulerAnglesReal.size(); i++)
		{
			if (eulerAnglesReal[i] < 0.) eulerAnglesReal[i] = -eulerAnglesReal[i];
			eulerAnglesReal[i] = 180. - eulerAnglesReal[i]>90. ? eulerAnglesReal[i] : 180. - eulerAnglesReal[i];
		}

		cv::Mat& RwcSim = simFrame->getRotationInverse();
		rotationMatrix = SLAM::Converter::toMatrix3d(RwcSim);
		Eigen::Vector3d eulerAnglesSim = rotationMatrix.eulerAngles(2, 1, 0).transpose();
		eulerAnglesSim = eulerAnglesSim / PI * 180.;
		for (int i = 0; i < eulerAnglesSim.size(); i++)
		{
			if (eulerAnglesSim[i] < 0.) eulerAnglesSim[i] = -eulerAnglesSim[i];
			eulerAnglesSim[i] = 180. - eulerAnglesSim[i]>90. ? eulerAnglesSim[i] : 180. - eulerAnglesSim[i];
		}

		Eigen::Vector3d  angleMat = eulerAnglesSim - eulerAnglesReal;
		double angleErr = angleMat.dot(angleMat);

		ofsPoseGBAErr << simFrame->getID() << " " << shift << " " << angleErr << std::endl;
	}
	
	int* matchedRMSEHist = new int[HISTO_LENGTH];
	memset(matchedRMSEHist, 0, HISTO_LENGTH*sizeof(int));
	double maxRMSE = 0.2;
	SLAM::System::Statistic matchedRMSEStats;
	SLAM::System::instance()->calcFrameMatchedRMSE(&SLAM::System::instance()->getGlobalMap(), matchedRMSEHist, HISTO_LENGTH, maxRMSE, matchedRMSEStats);
	cv::Mat matchedRMSEHistImg;
	SLAM::System::instance()->drawStatsHisto(matchedRMSEHistImg, matchedRMSEHist, HISTO_LENGTH, maxRMSE, &matchedRMSEStats);
	cv::imwrite(outDir + "仿真-帧匹配均方差统计.bmp", matchedRMSEHistImg);
	cv::imshow("【仿真-帧匹配均方差直方图】", matchedRMSEHistImg);


	int* matchedKFRMSEHist = new int[HISTO_LENGTH];
	memset(matchedKFRMSEHist, 0, HISTO_LENGTH*sizeof(int));
	double maxKFRMSE = 0.2;
	SLAM::System::Statistic matchedKFRMSEStats;
	SLAM::System::instance()->calcKFMatchedRMSE(&SLAM::System::instance()->getGlobalMap(), matchedKFRMSEHist, HISTO_LENGTH, maxKFRMSE, matchedKFRMSEStats);
	cv::Mat matchedKFRMSEHistImg;
	SLAM::System::instance()->drawStatsHisto(matchedKFRMSEHistImg, matchedKFRMSEHist, HISTO_LENGTH, maxKFRMSE, &matchedKFRMSEStats);
	cv::imwrite(outDir + "仿真-关键帧匹配均方差统计.bmp", matchedKFRMSEHistImg);
	cv::imshow("【仿真-关键帧匹配均方差直方图】", matchedKFRMSEHistImg);
	cv::waitKey(0);

	SLAM::System::instance()->shutDown();
	return true;
}