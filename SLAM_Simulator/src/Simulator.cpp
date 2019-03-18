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

	///////////////////////��ȡ��ͼ��///////////////////////
	//��ȡ��ͼ��ȫ������
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

	//������ȡ��ͼ��
	SLAM::MptList& mptList = _realMap.getMapPointList();
	for (int i = 0; i < mptSize; i++)
	{
		SLAM::MapPoint3d* mpt = new SLAM::MapPoint3d();
		mpt->setGlobalMap(&_realMap);

		//��ȡ��ͼ�������Ϣ
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

		//��ȡƥ��֡��
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


		//��ȡƥ��֡
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int obsIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//���buf[k]�ֽڲ�Ϊ��
				while (bufferp[k] && isspace(bufferp[k])) k++;//����buf[k]���ֽ��Ҹ��ֽ��ǡ�/t���ո��Թ��ÿո�
				if (bufferp[k])
				{//���buf[k]���ֽ��Ҹ��ֽڲ��ǡ�/t���ո����������j>=0
					if (sscanf(bufferp + k, "%d", &obsIndex) == EOF)
					{
						fprintf(stderr, "Bad input line: %d, while reading map points'observations!\n", lineCount);
						return false;
					}
					mptMathcedFrames[mpt].emplace_back(obsIndex);
				}
				while (bufferp[k] && !isspace(bufferp[k]))
				{//��λkΪ����ֵ��һ���ո���ŵ�����
					k++;
				}
			}
			if (obsSize != mptMathcedFrames[mpt].size())
			{
				fprintf(stderr, "Observations size is not matched ,line: %d\n", lineCount);
				return false;
			}
		}


		//��ȡƥ��ؼ�֡��
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

		//��ȡƥ��ؼ�֡	
		{
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int obsKFIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//���buf[k]�ֽڲ�Ϊ��
				while (bufferp[k] && isspace(bufferp[k])) k++;//����buf[k]���ֽ��Ҹ��ֽ��ǡ�/t���ո��Թ��ÿո�
				if (bufferp[k])
				{//���buf[k]���ֽ��Ҹ��ֽڲ��ǡ�/t���ո����������j>=0
					if (sscanf(bufferp + k, "%d", &obsKFIndex) == EOF)
					{
						fprintf(stderr, "Bad input line: %d\n", lineCount);
						return false;
					}
					mptMathcedKFs[mpt].emplace_back(obsKFIndex);
				}
				while (bufferp[k] && !isspace(bufferp[k]))
				{//��λkΪ����ֵ��һ���ո���ŵ�����
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

	///////////////////////��ȡȫ��֡///////////////////////

	typedef std::map<SLAM::Frame*, std::vector<int>> FrameVectorMap;
	typedef std::map<SLAM::KeyFrame*, std::vector<int>> KFVectorMap;
	typedef std::map<SLAM::Frame*, int> FrameIntMap;
	typedef std::map<SLAM::KeyFrame*, int> KFIntMap;
	FrameVectorMap frameCovisMap;
	KFVectorMap kFCovisMap;
	KFIntMap kFParentMap;
	FrameIntMap kFRefMap;
	//��ȡ֡��
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

	//������ȡ֡
	for (int i = 0; i < frameSize; i++)
	{
		SLAM::Frame* pFrame = new SLAM::Frame(-1, _realCamera.width(), _realCamera.height());
		pFrame->setCamera(&_realCamera);
		pFrame->setGlobalMap(&_realMap);

		int kFIndex = -1;
		int refKFIndex = -1;
		//��ȡ֡������Ϣ
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


		//��ȡ֡ƥ���������3d�㣬2d�㣬��ͼ�㣩
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
		//������ȡ֡ƥ��3d��͵�ͼ��
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

		//��ȡ֡��ɢƥ�������
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
		{//��ȡ֡��ɢƥ���ͼ��
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int outlierIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//���buf[k]�ֽڲ�Ϊ��
				while (bufferp[k] && isspace(bufferp[k])) k++;//����buf[k]���ֽ��Ҹ��ֽ��ǡ�/t���ո��Թ��ÿո�
				if (bufferp[k])
				{//���buf[k]���ֽ��Ҹ��ֽڲ��ǡ�/t���ո����������j>=0
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
				{//��λkΪ����ֵ��һ���ո���ŵ�����
					k++;
				}
			}
			if (pFrame->getOutLiers().size() != outLierSize)
			{
				fprintf(stderr, "Bad input line: %d, Outliers size not matched!\n", lineCount);
				return false;
			}
		}


		//��ȡ����֡����
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
		{//������ȡ����֡
			while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
				lineCount++;
			lineCount++;
			k = 0;
			int covisIndex = 0;
			char* bufferp = buffer;
			while (bufferp[k])
			{//���buf[k]�ֽڲ�Ϊ��
				while (bufferp[k] && isspace(bufferp[k])) k++;//����buf[k]���ֽ��Ҹ��ֽ��ǡ�/t���ո��Թ��ÿո�
				if (bufferp[k])
				{//���buf[k]���ֽ��Ҹ��ֽڲ��ǡ�/t���ո����������j>=0
					if (sscanf(bufferp + k, "%d", &covisIndex) == EOF)
					{
						fprintf(stderr, "Bad input line: %d, while reading covisible frames!\n", lineCount);
						return false;
					}

					frameCovisMap[pFrame].emplace_back(covisIndex);
				}
				while (bufferp[k] && !isspace(bufferp[k]))
				{//��λkΪ����ֵ��һ���ո���ŵ�����
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
		{//���ǹؼ�֡
			SLAM::KeyFrame* pKF = new SLAM::KeyFrame(*pFrame);
			pKF->setID(kFIndex);

			//��ȡ���ڵ�id
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

			//��ȡ����ؼ�֡����
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
			{//������ȡ����ؼ�֡
				while (!SLAM::ReadWriter::readLine(fp, buffer, 10239))
					lineCount++;
				lineCount++;
				k = 0;
				int covisKFIndex = 0;
				char* bufferp = buffer;
				while (bufferp[k])
				{//���buf[k]�ֽڲ�Ϊ��
					while (bufferp[k] && isspace(bufferp[k])) k++;//����buf[k]���ֽ��Ҹ��ֽ��ǡ�/t���ո��Թ��ÿո�
					if (bufferp[k])
					{//���buf[k]���ֽ��Ҹ��ֽڲ��ǡ�/t���ո����������j>=0
						if (sscanf(bufferp + k, "%d", &covisKFIndex) == EOF)
						{
							fprintf(stderr, "Bad input line: %d, while reading covisible keyframes!\n", lineCount);
							return false;
						}

						kFCovisMap[pKF].emplace_back(covisKFIndex);
					}
					while (bufferp[k] && !isspace(bufferp[k]))
					{//��λkΪ����ֵ��һ���ո���ŵ�����
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
	SLAM::System::instance()->setOutputTrackImgFlag(false);//�Ƿ����֡-֡����ƥ��ͼ��	
	SLAM::System::instance()->setOutputStereoImgFlag(false);//�Ƿ������֡���������־��ƥ��ͼ��
	SLAM::System::instance()->setOutputPosFlag(false);//�Ƿ����ÿ֡λ�˵��ļ�
	SLAM::System::instance()->setOutputP3dFlag(false);//�Ƿ����ÿ֡3d�����굽�ļ�
	SLAM::System::instance()->setDisplay3DModelFlag(true);//�Ƿ�ʵʱ��ʾ3dģ��														   //////////////////�Ż���ʽѡ��///////////////
	SLAM::System::instance()->setOptimize3DFlag(true);//�Ƿ����3d-3d�Ż�
	SLAM::System::instance()->setLocalBundleAdjustmentFlag(false);//�Ƿ������ֲ߳̾��Ż�
	SLAM::System::instance()->setDispErrorStats(false);//����̨�Ƿ����ÿ֡λ�ˡ��ֲ��Ż������Ϣ
	SLAM::System::instance()->setLoopClosingFlag(true);//�Ƿ����ػ����

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

	//ȫ���Ż�
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
	cv::imwrite(outDir + "����-֡ƥ�������ͳ��.bmp", matchedRMSEHistImg);
	cv::imshow("������-֡ƥ�������ֱ��ͼ��", matchedRMSEHistImg);


	int* matchedKFRMSEHist = new int[HISTO_LENGTH];
	memset(matchedKFRMSEHist, 0, HISTO_LENGTH*sizeof(int));
	double maxKFRMSE = 0.2;
	SLAM::System::Statistic matchedKFRMSEStats;
	SLAM::System::instance()->calcKFMatchedRMSE(&SLAM::System::instance()->getGlobalMap(), matchedKFRMSEHist, HISTO_LENGTH, maxKFRMSE, matchedKFRMSEStats);
	cv::Mat matchedKFRMSEHistImg;
	SLAM::System::instance()->drawStatsHisto(matchedKFRMSEHistImg, matchedKFRMSEHist, HISTO_LENGTH, maxKFRMSE, &matchedKFRMSEStats);
	cv::imwrite(outDir + "����-�ؼ�֡ƥ�������ͳ��.bmp", matchedKFRMSEHistImg);
	cv::imshow("������-�ؼ�֡ƥ�������ֱ��ͼ��", matchedKFRMSEHistImg);
	cv::waitKey(0);

	SLAM::System::instance()->shutDown();
	return true;
}