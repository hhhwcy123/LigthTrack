#include "Camera.h"

namespace SLAM
{


bool Camera::calibrateStereo(const std::string& chessImgDirLeft, const std::string& chessImgDirRight,
								cv::Size boardSize, float squareSize, 
								std::string outputDir,
								bool displayCorners , bool showRectified)
{
	ReadWriter::FilePathMap pathList1;
	ReadWriter::FilePathMap pathList2;
	

	ReadWriter::getFileListFromDir(chessImgDirLeft, pathList1);
	ReadWriter::getFileListFromDir(chessImgDirRight, pathList2);


	if (pathList1.size() != pathList2.size())
	{
		std::cout << "Wrong number of input chessboard images for stereo camera" << std::endl;
		return false;
	}

	int nImages = pathList1.size();
	if (nImages < 2)
	{
		std::cout << "Error: too little pairs to run the calibration" << std::endl;
		return false;
	}

	const int maxScale = 2;

	ReadWriter::StrList imgPathList;
	imgPathList.resize(2 * nImages);
	int p = 0;
	for ( ReadWriter::FilePathMap::iterator fItr1 = pathList1.begin(); fItr1 != pathList1.end(); fItr1++, p++)
		imgPathList[2* p] = fItr1->second;
	p = 0;
	for (ReadWriter::FilePathMap::iterator fItr2 = pathList2.begin(); fItr2 != pathList2.end(); fItr2++, p++)
		imgPathList[2 * p +1] = fItr2->second;



	/////////////////////生成imagePoints//////////////////////
	std::vector<std::vector<cv::Point2f> > imagePoints[2];
	std::vector<std::vector<cv::Point3f> > objectPoints;
	cv::Size imageSize;

	imagePoints[0].resize(nImages);
	imagePoints[1].resize(nImages);
	std::vector<std::string> goodImageList;

	int i, j, k;
	for (i = j = 0; i < nImages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const std::string& filename = imgPathList[i * 2 + k];
			cv::Mat img = cv::imread(filename, 0);
			if (img.empty())
				break;
			if (imageSize == cv::Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			std::vector<cv::Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				cv::Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, cv::Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						cv::Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			if (displayCorners)
			{
				std::cout << "Loaded file: "<<filename << std::endl;
				cv::Mat cimg, cimg1;
				cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, cv::Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)cv::waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
						 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
						30, 0.01));
	
		}
		if (k == 2)
		{
			goodImageList.emplace_back(imgPathList[i * 2]);
			goodImageList.emplace_back(imgPathList[i * 2 + 1]);
			j++;
		}
	}

	//////////////////////生成objectPoints//////////////////
	objectPoints.resize(nImages);
	for (i = 0; i < nImages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].emplace_back(cv::Point3f(k*squareSize, j*squareSize, 0));
	}

	///////////////////////开始标定/////////////////////////////
	std::cout << "Running stereo calibration ..."<<std::endl;
	cv::Mat K[2],D[2],rvec[2],tvec[2];
	cv::calibrateCamera(objectPoints, imagePoints[0], imageSize, K[0], D[0], rvec[0], tvec[0]);
	cv::calibrateCamera(objectPoints, imagePoints[1], imageSize, K[1], D[1], rvec[1], tvec[1]);

	
#if CV_MAJOR_VERSION < 3
	double rms = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
									 K[0], D[0],
									 K[1], D[1],
									 imageSize, _R, _t, _E, _F,
									 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6),
									 cv::CALIB_USE_INTRINSIC_GUESS +
									 cv::CALIB_SAME_FOCAL_LENGTH//+
								   // cv::CALIB_RATIONAL_MODEL
									);
#else
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
								 K[0], D[0],
								 K[1], D[1],
								 imageSize, _R, _t, _E, _F,
								 cv::CALIB_USE_INTRINSIC_GUESS +
								 cv::CALIB_SAME_FOCAL_LENGTH,//+
								 							// cv::CALIB_RATIONAL_MODEL ,
								 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));
#endif
	std::cout << "done with RMSE error=" << rms << std::endl;


	//////////////////////////保存内参//////////////////////////
	_baseLen = cv::norm(-_R.t()*_t);
	_Kl = K[0];
	_Kr = K[1];
	_Dl = D[0];
	_Dr = D[1];
	_imgWidth = imageSize.width;
	_imgHeight = imageSize.height;
	if (outputDir.back() != '\\') 
		outputDir += '\\';
	cv::FileStorage fs(outputDir+"intrinsics.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "K1" << _Kl << "D1" << _Dl
		   << "K2" << _Kr << "D2" << _Dr
		   << "width" << _imgWidth << "height" << _imgHeight
		   << "baseLen" << _baseLen;
		fs.release();
	}
	else
	{
		std::cout << "Error: can not save the intrinsic parameters" << std::endl;
		return false;
	}
		
	////////////////////////畸变矫正/////////////////////////

	cv::Rect validRoi[2];

	stereoRectify(K[0], D[0],K[1], D[1],
				  imageSize, _R, _t, _Rl, _Rr, _Pl, _Pr, _Q,
				  0, 1, imageSize, &validRoi[0], &validRoi[1]);



	////////////////////////保存外参/////////////////////////
	fs.open(outputDir+"extrinsics.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << _R << "t" << _t << "R1" << _Rl << "R2" << _Rr << "P1" << _Pl << "P2" << _Pr << "Q" << _Q << "E" << _E << "F" << _F;
		fs.release();
	}
	else
	{
		std::cout << "Error: can not save the extrinsic parameters" << std::endl;
		return false;
	}
		
	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(_Pl.at<double>(1, 3)) > fabs(_Pr.at<double>(0, 3));

	if (!showRectified)
		return true;//如果不显示畸变矫正图像则退出


	/////////////////////////显示畸变矫正后的图像////////////////////

	//////////////计算畸变矫正矩阵rmap/////////////
	cv::Mat rmap[2][2];
	initUndistortRectifyMap(K[0], D[0], _Rl, _Pl, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(K[1], D[1], _Rr, _Pr, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < nImages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			cv::Mat img = cv::imread(goodImageList[i * 2 + k], 0), rimg, cimg;
			cv::remap(img, rimg, rmap[k][0], rmap[k][1], cv::INTER_LINEAR);//图像畸变矫正
			cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
			cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
			
			cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
				cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
			cv::rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);
			
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		char c = (char)cv::waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
	return true;

}


bool Camera::loadStereoExtrinsicsFile(const std::string& filePath)
{
	cv::FileStorage fsSettings(filePath, cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		std::cerr << "ERROR: Wrong path to extrinsic settings" << std::endl;
		return -1;
	}

	fsSettings["R"] >> _R;
	fsSettings["t"] >> _t;

	fsSettings["P1"] >> _Pl;
	fsSettings["P2"] >> _Pr;

	fsSettings["R1"] >> _Rl;
	fsSettings["R2"] >> _Rr;
	fsSettings["Q"] >> _Q;
	fsSettings["E"] >> _E;
	fsSettings["F"] >> _F;

	if (_Pl.empty() || _Pr.empty() || _Rl.empty() || _Rr.empty())
	{
		std::cerr << "ERROR: Calibration extrinsic parameters to rectify stereo are missing!" << std::endl;
		return -1;
	}

}



bool Camera::loadStereoIntrinsicsFile(const std::string& filePath)
{
	cv::FileStorage fsSettings(filePath, cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		std::cerr << "ERROR: Wrong path to intrinsic settings" << std::endl;
		return -1;
	}


	fsSettings["K1"] >> _Kl;
	fsSettings["K2"] >> _Kr;

	fsSettings["D1"] >> _Dl;
	fsSettings["D2"] >> _Dr;


	_imgHeight = fsSettings["height"];
	_imgWidth = fsSettings["width"];
	_baseLen = fsSettings["baseLen"];


	if (_Kl.empty() || _Kr.empty() || _Dl.empty() || _Dr.empty())
	{
		std::cerr << "ERROR: Calibration intrinsic parameters to rectify stereo are missing!" << std::endl;
		return -1;
	}

}







}