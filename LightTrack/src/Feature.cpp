#include "Feature.h"
#include <Cuda\helper_cuda.h>
#include <Cuda\Exceptions.h>
#include <Cuda\Cuda_Wrapper.h>

#include "opencv2\cudafilters.hpp"
#include "opencv2\cudaimgproc.hpp"

namespace SLAM
{

#ifdef STAGE
int  Feature::MinArea = 50;        //轮廓最小面积
#else
int  Feature::MinArea = 200;        //轮廓最大面积
#endif//_STAGE
int  Feature::MaxArea = 10000;        //轮廓最大面积
double  Feature::MinCircularity = 0.8; //最小圆度限制
double  Feature::MaxCircularity = 1.2; //最大圆度限制
double  Feature::MaxConvexity = 1.0;   //最大凸度限制
double  Feature::MinConvexity = 0.5;   //最小凸度限制
double  Feature::MaxEccentricity = 0.89; //最大惯性率限制
double  Feature::MaxFitRMSE = 1.0;//轮廓椭圆拟合RMSE限制
double  Feature::MaxContourError = 2.0;   //轮廓椭圆拟合局部最大偏差限制
double  Feature::MinDefinition = 25;  //标识点分辨度限制
double  Feature::DotMarkRadius = 3;//标识点半径默认3mm
	
const Feature::ExtractMarkFunc Feature::_extractMarkFuncs[] = { extractDotMarkersSimple,
																extractDotMarkersNpp,
																extractDotMarkersCuda ,
																extractDotMarkersFast,
																extractDotMarkersSubPixel };
bool Feature::cannyNpp(const cv::Mat& srcImg, cv::Mat& dstImg,
					   const double threshold1, const double threshold2,
					   NppiMaskSize kernel_size)
{
	NppiSize srcSize = { srcImg.cols, srcImg.rows };
	NppiPoint srcOffset = { 0, 0 };

	Npp8u *srcImgDev = 0;
	int stepBytes = srcImg.step[0];

	srcImgDev = nppiMalloc_8u_C1(srcImg.cols, srcImg.rows, reinterpret_cast<int*>(&stepBytes));
	NPP_ASSERT(srcImgDev != 0);

	NPP_ASSERT(cudaSuccess == cudaMemcpy2D(srcImgDev, srcImg.step, srcImg.data, srcImg.step,
										   srcImg.cols * sizeof(Npp8u), srcImg.rows,
										   cudaMemcpyHostToDevice));

	NppiSize sizeROI = srcSize;
	Npp8u *dstImgDev = 0;
	if (!dstImgDev)
		dstImgDev = nppiMalloc_8u_C1(srcImg.cols, srcImg.rows, reinterpret_cast<int*>(&stepBytes));
	NPP_ASSERT(dstImgDev != 0);

	int nBufferSize = 0;
	Npp8u * pScratchBufferNPP = 0;
	NPP_CHECK_NPP(
		nppiFilterCannyBorderGetBufferSize(sizeROI, &nBufferSize));\
	if(!pScratchBufferNPP)
		cudaMalloc((void **)&pScratchBufferNPP, nBufferSize);

	if ((nBufferSize > 0) && (pScratchBufferNPP != 0))
	{
		NPP_CHECK_NPP(
			nppiFilterCannyBorder_8u_C1R(srcImgDev, srcImg.step, srcSize, srcOffset,
										 dstImgDev, srcImg.step, sizeROI,
										 NPP_FILTER_SOBEL, kernel_size, threshold1, threshold2,
										 nppiNormL2, NPP_BORDER_REPLICATE, pScratchBufferNPP));
	}


	cudaFree(pScratchBufferNPP);

	uchar* dstImgHost = (uchar*)calloc(srcImg.rows * srcImg.cols, sizeof(uchar));
	NPP_ASSERT(cudaSuccess == cudaMemcpy2D(dstImgHost, srcImg.step, dstImgDev, srcImg.step,
										   srcImg.cols * sizeof(Npp8u), srcImg.rows,
										   cudaMemcpyDeviceToHost));

	dstImg=cv::Mat(srcImg.rows, srcImg.cols,CV_8UC1, dstImgHost);
	nppiFree(srcImgDev);
	nppiFree(dstImgDev);
	//delete[] dstImgHost;
}



bool Feature::extractDotMarkersNpp(const cv::Mat& srcImg, MarkerList& markerList)
{
	cv::Mat blurImg;
	cv::Mat edgesImg;
	//cv::Mat closeImg; //= outImg;
	std::vector<std::vector<cv::Point>> contourList;
	std::vector<std::vector<cv::Point>> externContourList;
	std::vector<cv::Vec4i> hierarchy;

	double lowThreshold = 100;//120.0;
	double ratio = 2.5;//3.0;
	int kernel_size = 3;

	cv::medianBlur(srcImg, blurImg, 3);

	//std::vector<std::list<cv::Point>> contourLists =EdgeDrawing::detectEdges(blurImg);
	//std::vector<std::list<cv::Point>> contourLists =EdgeDrawing::detectEdges(blurImg);
	cannyNpp(blurImg, edgesImg, lowThreshold, lowThreshold*ratio); 

	cv::findContours(edgesImg, contourList, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
	delete edgesImg.data;//手动释放Mat内存(在cannyNpp中开辟)

	std::vector<int> indices;
	for (int i = 0; i < contourList.size();)
	{
		indices.emplace_back(i);
		int nextIndex = hierarchy[i][0];
		if (nextIndex == -1)
			break;
		i = nextIndex;
	}
	for (auto itr = indices.begin(); itr != indices.end(); itr++)
		externContourList.emplace_back(contourList[*itr]);

	int contoursSize = externContourList.size();

	cv::RotatedRect ellipsemege;
	for (int i = 0; i < contoursSize; i++)
	{
		double areaLen = cv::arcLength(externContourList[i], true);
		double areaSize = cv::contourArea(externContourList[i]);

		if (areaSize < MinArea || areaSize>MaxArea)
			continue;
		double ratio = areaLen*areaLen / areaSize;
		if (ratio < 4 * PI || ratio>8 * PI)
			continue;

		ellipsemege = cv::fitEllipse(externContourList[i]);

		double a = 0, b = 0;
		if (ellipsemege.size.height > ellipsemege.size.width)
		{
			a = ellipsemege.size.height / 2;
			b = ellipsemege.size.width / 2;
		}
		else
		{
			a = ellipsemege.size.width / 2;
			b = ellipsemege.size.height / 2;
		}

		double eccentricity = sqrt(a*a - b*b) / a;
		if (eccentricity > MaxEccentricity)
			continue;

		DotMarker newMarker{ ellipsemege.center.x, ellipsemege.center.y, ellipsemege.size.area(), eccentricity };
		newMarker.rect = ellipsemege;
		newMarker.contours = externContourList[i];

		if (!calcEllipseFitError(ellipsemege, externContourList[i], newMarker.ellipseFitError, MaxContourError))
		{//若轮廓椭圆拟合局部误差较大
			newMarker.fBad = true;//若拟合误差在阈值范围内，设置fbad并加优化权重
			//continue;
		}
		if (newMarker.ellipseFitError > MaxFitRMSE)//若轮廓椭圆拟合RMSE过大（通常模糊、残缺导致）
			continue;

		/*newMark.definition = calcMarkDefinition(srcImg, ellipsemege);*/
		/*if (newMark.definition < MinDefinition)
		newMark.fBad = true;*/

		bool isSame = false;
		for (MarkerList::iterator itr = markerList.begin(); itr != markerList.end(); itr++)
		{
			DotMarker& tmpMarker = *itr;
			if (std::sqrt(std::pow(newMarker.x - tmpMarker.x, 2) + std::pow(newMarker.y - tmpMarker.y, 2)) < b)
			{
				isSame = true;
				if (newMarker.areaSize < tmpMarker.areaSize)
					tmpMarker = newMarker;
				break;
			}
		}
		if (!isSame)
		{
			markerList.emplace_back(newMarker);
		}
	}
	if (markerList.empty())
		return false;

	return true;
}



bool Feature::extractDotMarkersCuda(const cv::Mat& srcImg, MarkerList& markerList)
{
	cv::cuda::GpuMat srcImgGPU = loadMat(srcImg, false);
	cv::cuda::GpuMat blurImgGPU;
	cv::cuda::GpuMat edgesImgGPU;
	//cv::Mat closeImg; //= outImg;
	std::vector<std::vector<cv::Point>> contourList;
	std::vector<std::vector<cv::Point>> externContourList;
	std::vector<cv::Vec4i> hierarchy;

	double lowThreshold = 100;//120.0;
	double ratio = 2.5;//3.0;
	int kernel_size = 3;

	cv::Ptr<cv::cuda::Filter> gaussianFilter=cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(kernel_size, kernel_size),0);
	gaussianFilter->apply(srcImgGPU, blurImgGPU);

	cv::Ptr<cv::cuda::CannyEdgeDetector> cannyDetectorGPU = cv::cuda::createCannyEdgeDetector(lowThreshold, lowThreshold*ratio, 3,true);
	cannyDetectorGPU->detect(blurImgGPU, edgesImgGPU);

	cv::Mat edgesImg = getMat(edgesImgGPU);


	cv::findContours(edgesImg, contourList, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

	std::vector<int> indices;
	for (int i = 0; i < contourList.size();)
	{
		indices.emplace_back(i);
		int nextIndex = hierarchy[i][0];
		if (nextIndex == -1)
			break;
		i = nextIndex;
	}
	for (auto itr = indices.begin(); itr != indices.end(); itr++)
		externContourList.emplace_back(contourList[*itr]);

	int contoursSize = externContourList.size();

	cv::RotatedRect ellipsemege;
	for (int i = 0; i < contoursSize; i++)
	{
		double areaLen = cv::arcLength(externContourList[i], true);
		double areaSize = cv::contourArea(externContourList[i]);

		if (areaSize < MinArea || areaSize>MaxArea)
			continue;
		double ratio = areaLen*areaLen / areaSize;
		if (ratio < 4 * PI || ratio>8 * PI)
			continue;

		ellipsemege = cv::fitEllipse(externContourList[i]);

		double a = 0, b = 0;
		if (ellipsemege.size.height > ellipsemege.size.width)
		{
			a = ellipsemege.size.height / 2;
			b = ellipsemege.size.width / 2;
		}
		else
		{
			a = ellipsemege.size.width / 2;
			b = ellipsemege.size.height / 2;
		}

		double eccentricity = sqrt(a*a - b*b) / a;
		if (eccentricity > MaxEccentricity)
			continue;

		DotMarker newMarker{ ellipsemege.center.x, ellipsemege.center.y, ellipsemege.size.area(), eccentricity };
		newMarker.rect = ellipsemege;
		newMarker.contours = externContourList[i];

		if (!calcEllipseFitError(ellipsemege, externContourList[i], newMarker.ellipseFitError, MaxContourError))
		{//若轮廓椭圆拟合局部误差较大
			newMarker.fBad = true;//若拟合误差在阈值范围内，设置fbad并加优化权重
			//continue;
		}
		if (newMarker.ellipseFitError > MaxFitRMSE)//若轮廓椭圆拟合RMSE过大（通常模糊、残缺导致）
			continue;

		/*newMark.definition = calcMarkDefinition(srcImg, ellipsemege);*/
		/*if (newMark.definition < MinDefinition)
		newMark.fBad = true;*/

		bool isSame = false;
		for (MarkerList::iterator itr = markerList.begin(); itr != markerList.end(); itr++)
		{
			DotMarker& tmpMarker = *itr;
			if (std::sqrt(std::pow(newMarker.x - tmpMarker.x, 2) + std::pow(newMarker.y - tmpMarker.y, 2)) < b)
			{
				isSame = true;
				if (newMarker.areaSize < tmpMarker.areaSize)
					tmpMarker = newMarker;
				break;
			}
		}
		if (!isSame)
		{
			markerList.emplace_back(newMarker);
		}
	}
	if (markerList.empty())
		return false;

	return true;
}



bool Feature::extractDotMarkersSimple(const cv::Mat& srcImg, MarkerList& markerList)
{
	cv::Mat blurImg;
	cv::Mat edgesImg;
	//cv::Mat closeImg; //= outImg;
	std::vector<std::vector<cv::Point>> contourList;
	std::vector<std::vector<cv::Point>> externContourList;
	std::vector<cv::Vec4i> hierarchy;

	double lowThreshold = 100;//120.0;
	double ratio = 2.5;//3.0;
	int kernel_size = 3;

	cv::medianBlur(srcImg, blurImg, 3);

	
	//std::vector<std::list<cv::Point>> contourLists =EdgeDrawing::detectEdges(blurImg);

	cv::Canny(blurImg, edgesImg, lowThreshold, lowThreshold*ratio, kernel_size, true);
	//cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	//cv::morphologyEx(edgesImg, closeImg, cv::MORPH_CLOSE, kernel);
	cv::findContours(edgesImg, contourList, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
	
	std::vector<int> indices;
	for (int i = 0; i < contourList.size();)
	{
		indices.emplace_back(i);
		int nextIndex = hierarchy[i][0];
		if (nextIndex == -1)
			break;
		i = nextIndex;
	}
	for (auto itr = indices.begin(); itr != indices.end(); itr++)
		externContourList.emplace_back(contourList[*itr]);

	int contoursSize = externContourList.size();

	cv::RotatedRect ellipsemege;
	for (int i = 0; i < contoursSize; i++)
	{
		double areaLen = cv::arcLength(externContourList[i], true);
		double areaSize = cv::contourArea(externContourList[i]);

		if (areaSize < MinArea || areaSize>MaxArea)
			continue;
		double ratio = areaLen*areaLen / areaSize;
		if (ratio < 4 * PI || ratio>8 * PI)
			continue;

		ellipsemege = cv::fitEllipse(externContourList[i]);

		double a = 0, b = 0;
		if (ellipsemege.size.height > ellipsemege.size.width)
		{
			a = ellipsemege.size.height / 2;
			b = ellipsemege.size.width / 2;
		}
		else
		{
			a = ellipsemege.size.width / 2;
			b = ellipsemege.size.height / 2;
		}

		double eccentricity = sqrt(a*a - b*b) / a;
		if (eccentricity > MaxEccentricity)
			continue;

		DotMarker newMarker{ ellipsemege.center.x, ellipsemege.center.y, ellipsemege.size.area(), eccentricity };
		newMarker.rect = ellipsemege;
		newMarker.contours= externContourList[i];

		if (!calcEllipseFitError(ellipsemege, externContourList[i], newMarker.ellipseFitError, MaxContourError))
		{//若轮廓椭圆拟合局部误差较大
			newMarker.fBad = true;//若拟合误差在阈值范围内，设置fbad并加优化权重
			//continue;
		}
		if (newMarker.ellipseFitError > MaxFitRMSE)//若轮廓椭圆拟合RMSE过大（通常模糊、残缺导致）
			continue;

		/*newMark.definition = calcMarkDefinition(srcImg, ellipsemege);*/
		/*if (newMark.definition < MinDefinition)
			newMark.fBad = true;*/

		bool isSame = false;
		for (MarkerList::iterator itr = markerList.begin(); itr != markerList.end(); itr++)
		{
			DotMarker& tmpMarker = *itr;
			if (std::sqrt(std::pow(newMarker.x - tmpMarker.x, 2) + std::pow(newMarker.y - tmpMarker.y, 2)) < b)
			{
				isSame = true;
				if (newMarker.areaSize < tmpMarker.areaSize)
					tmpMarker = newMarker;
				break;
			}
		}
		if (!isSame)
		{
			markerList.emplace_back(newMarker);
		}
	}
	if (markerList.empty())
		return false;

	return true;
}



bool Feature::extractDotMarkersFast(const cv::Mat& srcImg, MarkerList& markerList)
{
	cv::Mat binImg;
	double lowThreshold = 100.0;//120.0;
	double ratio = 2.5;//3.0;
	cv::threshold(srcImg, binImg, lowThreshold, lowThreshold*ratio, cv::THRESH_BINARY);
	std::vector < std::vector<cv::Point2i> > contours;
	cv::findContours(binImg, contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	ExtractParams params;

	double area = .0;
	double roundness = .0;//圆度（越接近1越接近标准圆形）
	double convexity = .0;//凸度(取值范围0-1，越接近1越是凸多边形）
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::Moments moms = cv::moments(cv::Mat(contours[i]));
		//面积限制
		area = moms.m00;    //零阶矩即为二值图像的面积
		if (params.filterByArea)
		{
			//area = contourArea(contours[i]);
			if (area < params.minArea || area >= params.maxArea)
				continue;
		}
		if (params.filterByCircularity)
		{
			double perimeter = cv::arcLength(cv::Mat(contours[i]), true);
			//得到圆度
			roundness = 4 * CV_PI * area / (perimeter * perimeter);
			//如果圆度超出了设定的范围，则不再考虑该斑点
			if (roundness < params.minCircularity || roundness >= params.maxCircularity)
				continue;
		}

		if (params.filterByConvexity)
		{
			std::vector <cv::Point2i> hull;
			//调用convexHull函数，得到凸壳
			cv::convexHull(cv::Mat(contours[i]), hull);
			double hullArea = cv::contourArea(cv::Mat(hull));
			convexity = area / hullArea;

			if (convexity < params.minConvexity || convexity > params.maxConvexity)
				continue;
		}

		double eccentricity = 0;
		cv::RotatedRect ellipsemege = cv::fitEllipse(contours[i]);
		if (params.filterByEccentricity)
		{
			double a = std::max(ellipsemege.size.height, ellipsemege.size.width);
			double b = std::min(ellipsemege.size.height, ellipsemege.size.width);
			eccentricity = sqrt(a*a - b*b) / a;
			if (eccentricity > params.maxEccentricity)
				continue;
		}

		DotMarker newMarker;
		newMarker.x = ellipsemege.center.x;
		newMarker.y = ellipsemege.center.y;
		newMarker.areaSize = area;
		newMarker.eccentricity = eccentricity;
		markerList.emplace_back(newMarker);
	}
	if (markerList.empty())
		return false;
	return true;
}



bool Feature::extractDotMarkersSubPixel(const cv::Mat& srcImg, MarkerList& markerList)
{
	static int  kirschArray[8][9] =
	{ { 5, 5, 5, -3, 0, -3, -3, -3, -3 },
	{ -3, 5, 5, -3, 0, 5, -3, -3, -3 },
	{ -3, -3, 5, -3, 0, 5, -3, -3, 5 },
	{ -3, -3, -3, -3, 0, 5, -3, 5, 5 },
	{ -3, -3, -3, -3, 0, -3, 5, 5, 5 },
	{ -3, -3, -3, 5, 0, -3, 5, 5, -3 },
	{ 5, -3, -3, 5, 0, -3, 5, -3, -3 },
	{ 5, 5, -3, 5, 0, -3, -3, -3, -3 } };

	cv::Mat blurImg;
	cv::Mat edgesImg;
	cv::Mat closeImg;
	std::vector<std::vector<cv::Point>> oriContours_int;
	std::vector<std::vector<cv::Point>> validContours_int;
	std::vector<std::vector<cv::Point2f>> validContours_float;
	std::vector<std::vector<cv::Point>> contours;

	std::vector<cv::Vec4i> hierarchy;
	double lowThreshold = 100.0;//120.0;
	double ratio = 2.5;//3.0;
	int kernel_size = 3;

	cv::medianBlur(srcImg, blurImg, 5);
	cv::Canny(blurImg, edgesImg, lowThreshold, lowThreshold*ratio, kernel_size, true);
	/*cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
	cv::morphologyEx(edgesImg, closeImg, cv::MORPH_CLOSE, kernel);*/
	cv::findContours(edgesImg, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

	std::vector<int> indices;
	for (int i = 0; i < contours.size();)
	{
		indices.emplace_back(i);
		int nextIndex = hierarchy[i][0];
		if (nextIndex == -1)
			break;
		i = nextIndex;
	}
	for (auto itr = indices.begin(); itr != indices.end(); itr++)
		oriContours_int.emplace_back(contours[*itr]);

	int contoursIntSize = oriContours_int.size();


	std::vector<cv::RotatedRect> rectList;
	for (int i = 0; i < contoursIntSize; i++)
	{
		double areaLen = cv::arcLength(oriContours_int[i], true);
		double areaSize = cv::contourArea(oriContours_int[i]);

		///////////////////面积、周长限制////////////////////
		if (areaSize < MinArea || areaSize>MaxArea)
			continue;
		////////////////圆率限制///////////////////
		double roundness = areaLen*areaLen / areaSize;
		if (roundness < 4 * PI || roundness>6 * PI)
			continue;

		/////////////////////凸度限制//////////////////////////////
		double convexity = .0;//取值范围0-1，越接近1越是凸多边形
		std::vector <cv::Point2i> hull;
		cv::convexHull(cv::Mat(oriContours_int[i]), hull);
		double hullArea = cv::contourArea(cv::Mat(hull));
		convexity = areaSize / hullArea;
		if (convexity < MinConvexity)
			continue;

		/////////粗拟合椭圆参数限制////////////////
		cv::RotatedRect ellipsemege = cv::fitEllipse(oriContours_int[i]);
		double a = 0, b = 0;
		if (ellipsemege.size.height>ellipsemege.size.width)
		{
			a = ellipsemege.size.height / 2;
			b = ellipsemege.size.width / 2;
		}
		else
		{
			a = ellipsemege.size.width / 2;
			b = ellipsemege.size.height / 2;
		}

		double eccentricity = sqrt(a*a - b*b) / a;//ellipsemege.size.height / ellipsemege.size.width;
		if (eccentricity > MaxEccentricity)
			continue;

		double ellipseFitError = 0;
		calcEllipseFitError(ellipsemege, oriContours_int[i], ellipseFitError, MaxContourError);

		if (ellipseFitError > MaxFitRMSE)
			continue;//若拟合RMSE过大(0.5),丢弃轮廓（通常模糊、残缺导致）

		/////////////抑制同心圆//////////////////
		bool isSame = false;
		int j = 0;
		for (auto itr = rectList.begin(); itr != rectList.end(); itr++, j++)
		{
			cv::RotatedRect& tmpRect = *itr;
			if (std::sqrt(std::pow(ellipsemege.center.x - tmpRect.center.x, 2) + std::pow(ellipsemege.center.y - tmpRect.center.y, 2)) < 15)
			{
				isSame = true;
				if (ellipsemege.size.area() < tmpRect.size.area())
				{
					tmpRect = ellipsemege;
					validContours_int[j] = oriContours_int[i];
				}

				break;
			}
		}
		if (!isSame)
		{
			rectList.emplace_back(ellipsemege);
			validContours_int.emplace_back(oriContours_int[i]);
		}
		else
			continue;
	}

	/////////////////轮廓亚像素定位//////////////
	std::vector<cv::Point2f> curContour;
	bool ret = true;
	int validContoursSize = validContours_int.size();
	int contourSize = 0;

	for (int i = 0; i < validContoursSize; i++)
	{
		std::vector<cv::Point2f> curContour;
		bool issue = true;
		contourSize = validContours_int[i].size();
		for (int j = 0; j < contourSize; j++)
		{
			//contours[i].resize(contours_int[i].size());	
			PIXELPOINT p;
			p.Width = validContours_int[i][j].x;
			p.Height = validContours_int[i][j].y;

			if (p.Width - 4 < 0 || p.Width + 4 > srcImg.cols ||
				p.Height - 4 < 0 || p.Height + 4 > srcImg.rows)
			{
				issue = false;
				break;
			}

			cv::Mat matImage;
			srcImg.copyTo(matImage);
			REALPOINT point = GetGradsSubPixelValue(matImage, p, kirschArray[0]);
			curContour.emplace_back(cv::Point2f(point.x, point.y));
			// 			contours[i][j].x = point.x;
			// 			contours[i][j].y = point.y;
		}
		if (issue)
		{
			validContours_float.emplace_back(curContour);//contours：亚像素轮廓集合
		}
	}



	//saveConcours(contours_float, "d:\\contours_float.txt");
	//saveConcoursint(contours_int, "d:\\contours_int.txt");


	if (markerList.empty())
		markerList.clear();
	//////////////////所有亚像素轮廓椭圆拟合////////////////////////
	int contoursFloatSize = validContours_float.size();
	for (int i = 0; i < contoursFloatSize; i++)
	{
		cv::RotatedRect realEllipsemege = cv::fitEllipse(validContours_float[i]);
		double realEccentricity = realEllipsemege.size.height / realEllipsemege.size.width;

		DotMarker newMarker(realEllipsemege.center.x, realEllipsemege.center.y, realEllipsemege.size.area(), realEccentricity);

		newMarker.rect = realEllipsemege;
		newMarker.contours=validContours_int[i];

		if (!calcEllipseFitError(realEllipsemege, validContours_float[i], newMarker.ellipseFitError, MaxContourError))
			newMarker.fBad = true;//若轮廓椭圆拟和局部误差过大，加优化权重（w<1）,拟合RMSE在椭圆粗拟合中判断，此处不判断

		newMarker.definition = calcMarkerDefinition(srcImg, newMarker);
		if (newMarker.definition < MinDefinition)
			newMarker.fBad = true;

		markerList.emplace_back(newMarker);
	}
	if (markerList.empty())
		return false;

	return true;
}



std::list<Feature::MarkerPose> Feature::calcMarkerPose(const DotMarker& marker, const cv::Mat& K)
{	
	//最小二乘拟合椭圆方程
	//std::vector<cv::Point> contours = marker.contours;
	//cv::RotatedRect ellipseRect=cvfitEllipse(contours);
	//cv::Mat paramsMat(5, 1, CV_64F);
	//fitEllipse2f(marker.contours, paramsMat);
	//double A = paramsMat.at<double>(0, 0);
	//double B = paramsMat.at<double>(1, 0);
	//double C = paramsMat.at<double>(2, 0);
	//double D = paramsMat.at<double>(3, 0);
	//double E = paramsMat.at<double>(4, 0);
	//EllipseParam params(1, A,B,C,D,E);
	//double err=calcEllipseFitError(marker.contours, params);

	const cv::RotatedRect& ellipseRect = marker.rect;
	EllipseParam params;
	getEllipseParam(ellipseRect, params);
	//double err=calcEllipseFitError(marker.contours, params);


	/*params.A = params.A / params.F;
	params.B = params.B / params.F;
	params.C = params.C / params.F;
	params.D = params.D / params.F;
	params.E = params.E / params.F;
	params.F = 1.;
	double err = calcEllipseFitError(marker.contours, params);*/

	double A = params.B / params.A;
	double B = params.C / params.A;
	double C = params.D / params.A;
	double D = params.E / params.A;
	double E = params.F / params.A;
	//EllipseParam params(1, A,B,C,D,E);
	//double err=calcEllipseFitError(marker.contours, params);


	//构造g矩阵
	cv::Mat g = (cv::Mat_<double>(3, 3) << 1, A / 2, C / 2,
										   A / 2, B, D / 2,
										   C / 2, D / 2, E);

	cv::Mat Q = K.t()*g*K;

	//存在 P矩阵  使P.t*Q*P = diag(λ1, λ 2, λ3)
	cv::Mat eval, evec;
	cv::eigen(Q, eval, evec);

	cv::Mat P = evec.clone();

	//然后确认特征值和特征向量 与 标准椭圆三个参数和P矩阵相应向量的对应关系
	int index_sign, index_max, index_rem;  //找出特征值符号和其他两个不一样的以及剩下两个模值比较大的
	double flag01 = eval.at<double>(0, 0)*eval.at<double>(1, 0);
	double flag02 = eval.at<double>(0, 0)*eval.at<double>(2, 0);
	if (flag01>0 && flag02<0)
	{
		index_sign = 2;
		index_max = abs(eval.at<double>(0, 0)) >= abs(eval.at<double>(1, 0)) ? 0 : 1;
	}
	else if (flag01<0 && flag02>0)
	{
		index_sign = 1;
		index_max = abs(eval.at<double>(0, 0)) >= abs(eval.at<double>(2, 0)) ? 0 : 2;
	}
	else
	{
		index_sign = 0;
		index_max = abs(eval.at<double>(1, 0)) >= abs(eval.at<double>(2, 0)) ? 1 : 2;
	}
	index_rem = 3 - index_max - index_sign;
	double lamta0, lamta1, lamta2;
	lamta2 = eval.at<double>(index_sign, 0);
	lamta0 = eval.at<double>(index_max, 0);
	lamta1 = eval.at<double>(index_rem, 0);

	evec.row(index_rem).copyTo(P.row(1));

	if (evec.at<double>(index_sign, 2) < 0)
	{
		evec.row(index_sign) = -evec.row(index_sign);
	}
	evec.row(index_sign).copyTo(P.row(2));
	cv::Mat crossM(1, 3, CV_64F);

	crossM = P.row(1).cross(P.row(2));
	crossM.copyTo(P.row(0));

	P = P.t();
	cv::Mat TT = P.t()*Q*P;  //验证TT是否为对角阵 P.t*Q*P = diag(λ1, λ 2, λ3)

	cv::Mat normalVec1(3, 1, CV_64F);
	cv::Mat normalVec2(3, 1, CV_64F);

	lamta0 = lamta0 < 0 ? -lamta0 : lamta0;
	lamta1 = lamta1 < 0 ? -lamta1 : lamta1;
	lamta2 = lamta2 < 0 ? -lamta2 : lamta2;
	//二义性
	cv::Mat dotCenter1(3, 1, CV_64F), dotCenter2(3, 1, CV_64F);
	dotCenter1.at<double>(0) = sqrt(lamta2*(lamta0 - lamta1) / (lamta0*(lamta0 + lamta2)))*DotMarkRadius;
	dotCenter1.at<double>(1) = 0;
	dotCenter1.at<double>(2) = sqrt(lamta0*(lamta1 + lamta2) / (lamta2*(lamta0 + lamta2)))*DotMarkRadius;
	normalVec1.at<double>(0) = sqrt((lamta0 - lamta1) / (lamta0 + lamta2));
	normalVec1.at<double>(1) = 0;
	normalVec1.at<double>(2) = -sqrt((lamta1 + lamta2) / (lamta0 + lamta2));
	
	dotCenter2.at<double>(0) = -sqrt(lamta2*(lamta0 - lamta1) / (lamta0*(lamta0 + lamta2)))*DotMarkRadius;
	dotCenter2.at<double>(1) = 0;
	dotCenter2.at<double>(2) = sqrt(lamta0*(lamta1 + lamta2) / (lamta2*(lamta0 + lamta2)))*DotMarkRadius;
	normalVec2.at<double>(0) = -sqrt((lamta0 - lamta1) / (lamta0 + lamta2));
	normalVec2.at<double>(1) = 0;
	normalVec2.at<double>(2) = -sqrt((lamta1 + lamta2) / (lamta0 + lamta2));
	
	normalVec1 = P * normalVec1;
	dotCenter1 = P * dotCenter1;
	normalVec2 = P * normalVec2;
	dotCenter2 = P * dotCenter2;
	
	std::list<MarkerPose> markPosePair;
	//相机坐标系下的法向量
	markPosePair.emplace_back(MarkerPose(dotCenter1, normalVec1));
	markPosePair.emplace_back(MarkerPose(dotCenter2, normalVec2));

	return markPosePair;
}



cv::RotatedRect Feature::cvfitEllipse(const cv::InputArray& contours)
{
	cv::Mat points = contours.getMat();
	int i, n = points.checkVector(2);
	int depth = points.depth();
	CV_Assert(n >= 0 && (depth == CV_32F || depth == CV_32S));

	cv::RotatedRect box;

	if (n < 5)
		CV_Error(CV_StsBadSize, "There should be at least 5 points to fit the ellipse");

	// New fitellipse algorithm, contributed by Dr. Daniel Weiss
	cv::Point2f c(0, 0);
	double gfp[5], rp[5], t;
	const double min_eps = 1e-8;
	bool is_float = depth == CV_32F;
	const cv::Point* ptsi = points.ptr<cv::Point>();
	const cv::Point2f* ptsf = points.ptr<cv::Point2f>();

	cv::AutoBuffer<double> _Ad(n * 5), _bd(n);
	double *Ad = _Ad, *bd = _bd;

	// first fit for parameters A - E
	cv::Mat A(n, 5, CV_64F, Ad);
	cv::Mat b(n, 1, CV_64F, bd);
	cv::Mat x(5, 1, CV_64F, gfp);

	for (i = 0; i < n; i++)
	{
		cv::Point2f p = is_float ? ptsf[i] : cv::Point2f((float)ptsi[i].x, (float)ptsi[i].y);
		c += p;
	}
	c.x /= n;
	c.y /= n;

	for (i = 0; i < n; i++)
	{
		cv::Point2f p = is_float ? ptsf[i] : cv::Point2f((float)ptsi[i].x, (float)ptsi[i].y);
		p -= c;

		bd[i] = 10000.0; // 1.0?
		Ad[i * 5] = -(double)p.x * p.x; // A - C signs inverted as proposed by APP
		Ad[i * 5 + 1] = -(double)p.y * p.y;
		Ad[i * 5 + 2] = -(double)p.x * p.y;
		Ad[i * 5 + 3] = p.x;
		Ad[i * 5 + 4] = p.y;
	}

	solve(A, b, x, cv::DECOMP_SVD);

	// now use general-form parameters A - E to find the ellipse center:
	// differentiate general form wrt x/y to get two equations for cx and cy
	A = cv::Mat(2, 2, CV_64F, Ad);
	b = cv::Mat(2, 1, CV_64F, bd);
	x = cv::Mat(2, 1, CV_64F, rp);
	Ad[0] = 2 * gfp[0];
	Ad[1] = Ad[2] = gfp[2];
	Ad[3] = 2 * gfp[1];
	bd[0] = gfp[3];
	bd[1] = gfp[4];
	solve(A, b, x, cv::DECOMP_SVD);

	// re-fit for parameters A - C with those center coordinates
	A = cv::Mat(n, 3, CV_64F, Ad);
	b = cv::Mat(n, 1, CV_64F, bd);
	x = cv::Mat(3, 1, CV_64F, gfp);
	for (i = 0; i < n; i++)
	{
		cv::Point2f p = is_float ? ptsf[i] : cv::Point2f((float)ptsi[i].x, (float)ptsi[i].y);
		p -= c;
		bd[i] = 1.0;
		Ad[i * 3] = (p.x - rp[0]) * (p.x - rp[0]);
		Ad[i * 3 + 1] = (p.y - rp[1]) * (p.y - rp[1]);
		Ad[i * 3 + 2] = (p.x - rp[0]) * (p.y - rp[1]);
	}
	solve(A, b, x, cv::DECOMP_SVD);

	// store angle and radii
	rp[4] = -0.5 * atan2(gfp[2], gfp[1] - gfp[0]); // convert from APP angle usage
	if (fabs(gfp[2]) > min_eps)
		t = gfp[2] / sin(-2.0 * rp[4]);
	else // ellipse is rotated by an integer multiple of pi/2
		t = gfp[1] - gfp[0];
	rp[2] = fabs(gfp[0] + gfp[1] - t);
	if (rp[2] > min_eps)
		rp[2] = std::sqrt(2.0 / rp[2]);
	rp[3] = fabs(gfp[0] + gfp[1] + t);
	if (rp[3] > min_eps)
		rp[3] = std::sqrt(2.0 / rp[3]);

	box.center.x = (float)rp[0] + c.x;
	box.center.y = (float)rp[1] + c.y;
	box.size.width = (float)(rp[2] * 2);
	box.size.height = (float)(rp[3] * 2);
	if (box.size.width > box.size.height)
	{
		float tmp;
		CV_SWAP(box.size.width, box.size.height, tmp);
		box.angle = (float)(90 + rp[4] * 180 / CV_PI);
	}
	if (box.angle < -180)
		box.angle += 360;
	if (box.angle > 360)
		box.angle -= 360;
	
	return box;
}



void Feature::fitEllipse2f(const std::vector<cv::Point>& contours, cv::Mat& params)
{
	double x1 = 0;
	double x2 = 0;
	double x3 = 0;
	//double x4 = 0;
	double y1 = 0;
	double y2 = 0;
	double y3 = 0;
	double y4 = 0;
	double x1y1 = 0;
	double x1y2 = 0;
	double x1y3 = 0;
	double x2y1 = 0;
	double x2y2 = 0;
	double x3y1 = 0;
	int num;
	std::vector<cv::Point>::const_iterator k;
	cv::Point2f tempcircle;
	num = contours.size();
	for (k = contours.begin(); k != contours.end(); k++)
	{
		x1 = x1 + (*k).x;
		x2 = x2 + pow((*k).x, 2);
		x3 = x3 + pow((*k).x, 3);
		//x4 = x4 + pow((*k).x, 4);
		y1 = y1 + (*k).y;
		y2 = y2 + pow((*k).y, 2);
		y3 = y3 + pow((*k).y, 3);
		y4 = y4 + pow((*k).y, 4);
		x1y1 = x1y1 + (*k).x * (*k).y;
		x1y2 = x1y2 + (*k).x * pow((*k).y, 2);
		x1y3 = x1y3 + (*k).x * pow((*k).y, 3);
		x2y1 = x2y1 + pow((*k).x, 2) * (*k).y;
		x2y2 = x2y2 + pow((*k).x, 2) * pow((*k).y, 2);
		x3y1 = x3y1 + pow((*k).x, 3) * (*k).y;
	}


	cv::Mat left_matrix = (cv::Mat_<double>(5, 5) << x2y2, x1y3, x2y1, x1y2, x1y1,
		x1y3, y4, x1y2, y3, y2,
		x2y1, x1y2, x2, x1y1, x1,
		x1y2, y3, x1y1, y2, y1,
		x1y1, y2, x1, y1, num);

	left_matrix /= num;

	cv::Mat right_matrix = (cv::Mat_<double>(5, 1) << -x3y1, -x2y2, -x3, -x2y1, -x2);

	right_matrix /= num;

	cv::Mat ellipse_solution(5, 1, CV_64F);

	solve(left_matrix, right_matrix, ellipse_solution, cv::DECOMP_LU);

	//ellipse_solution = left_matrix.inv()*right_matrix;

	params = ellipse_solution.clone();

}



void Feature::getEllipseParam(const cv::RotatedRect& ellipsemege, EllipseParam& param)
{
	const float theta = ellipsemege.angle * CV_PI / 180.0;
	const float a = ellipsemege.size.width / 2.0;
	const float b = ellipsemege.size.height / 2.0;

	const double& centerX = ellipsemege.center.x;
	const double& centerY = ellipsemege.center.y;

	param.A = a * a * sin(theta) * sin(theta) + b * b * cos(theta) * cos(theta);
	param.B = (-2.0) * (a * a - b * b) * sin(theta) * cos(theta);
	param.C = a * a * cos(theta) * cos(theta) + b * b * sin(theta) * sin(theta);
	param.D = -2 * param.A* centerX- param.B* centerY;
	param.E = -2 * param.C* centerY - param.B*centerX;
	param.F = (-1.0) * a * a * b * b+ param.A*pow(centerX,2)+ param.B*centerX*centerY+ param.C*pow(centerY, 2);

}


template<class T>
bool Feature::calcEllipseFitError(const cv::RotatedRect& rectEllipse, const std::vector<T>& contour, double& fitError, const double& th)
{
	double a(0), b(0);
	if (rectEllipse.size.height > rectEllipse.size.width)
	{
		a = rectEllipse.size.height / 2;
		b = rectEllipse.size.width / 2;
	}
	else
	{
		a = rectEllipse.size.width / 2;
		b = rectEllipse.size.height / 2;
	}
	double totalError2 = 0;
	double tempError = 0;
	double meanError = 0;
	double transAngle = 0;
	double pixAngel = 0;
	double pixDist = 0;
	double ri = 0;
	double x, y;
	bool fBad = false;

	double contourSize = contour.size();
	for (size_t i = 0; i < contourSize; i++)
	{
		transAngle = (90. - rectEllipse.angle) / 180. * PI;

		x = ((double)contour[i].x - rectEllipse.center.x)*cos(transAngle) -
			((double)contour[i].y - rectEllipse.center.y)*sin(transAngle);
		y = ((double)contour[i].x - rectEllipse.center.x)*sin(transAngle) +
			((double)contour[i].y - rectEllipse.center.y)*cos(transAngle);
		if (x > 0)
			pixAngel = atan(a / b*y / x);
		if (x < 0)
			pixAngel = atan(a / b*y / x) + PI;
		else
			pixAngel = atan2(y, x);

		ri = sqrt(pow(a*cos(pixAngel), 2) + pow(b*sin(pixAngel), 2));

		/*pixAngel = atan2(y, x);
		ri = a*b / sqrt(a*a*cos(pixAngel)*cos(pixAngel) + b*b*sin(pixAngel)*sin(pixAngel));*/
		pixDist = abs(sqrt(pow(x, 2) + pow(y, 2)) - ri);
		if (pixDist > th)
			fBad = true;
		totalError2 += pixDist*pixDist;

	}
	fitError = sqrt(totalError2 / (double)contourSize);
	if (fBad)
		return false;
	return true;
}


template<class T>
double Feature::calcEllipseFitError(const std::vector<T>& contours, const EllipseParam& params)
{
	const double& A = params.A;
	const double& B = params.B;
	const double& C = params.C;
	const double& D = params.D;
	const double& E = params.E;
	const double& F = params.F;

	double error = 0;

	for (int i = 0; i < contours.size(); i++)
	{
		double x = contours[i].x;
		double y = contours[i].y;
		error += abs(A*x*x + B*x*y + C*y*y + D*x + E*y + F);
	}
	error /= contours.size();
	return error;

}



double Feature::calcMarkerDefinition(const cv::Mat& srcImg, const DotMarker& marker)
{
	cv::Mat grayImg = srcImg.clone();
	if (grayImg.channels() != 1)
		cv::cvtColor(grayImg, grayImg, CV_RGB2GRAY);

	const cv::RotatedRect& rectEllipse = marker.rect;
	const double& centerX = rectEllipse.center.x;
	const double& centerY = rectEllipse.center.y;

	cv::Rect& boundingRect = rectEllipse.boundingRect();

	double l = boundingRect.tl().x - 5;
	double t = boundingRect.tl().y - 5;
	double r = boundingRect.br().x + 5;
	double b = boundingRect.br().y + 5;
	l = l < 0 ? 0 : l;
	t = t < 0 ? 0 : t;
	r = r > grayImg.cols - 1 ? grayImg.cols - 1 : r;
	b = b > grayImg.rows - 1 ? grayImg.rows - 1 : b;
	cv::Rect roiRect(l, t, r - l, b - t);

	cv::Mat imgRoi = grayImg(roiRect);

	/*cv::Mat imgRoiLap(imgRoi.rows, imgRoi.cols, CV_16SC(1));
	cv::Laplacian(imgRoi, imgRoiLap, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(imgRoiLap, imgRoiLap);*/

	cv::Mat imgRoiSobel(imgRoi.rows, imgRoi.cols, CV_16SC(1));
	cv::Sobel(imgRoi, imgRoiSobel, CV_16S, 1, 1);
	cv::convertScaleAbs(imgRoiSobel, imgRoiSobel);


	cv::Mat meanMat, stdevMat;
	double mean = 0, stdev = 0;
	cv::meanStdDev(imgRoiSobel, meanMat, stdevMat);
	mean = meanMat.at<double>(0, 0);		//均值
	stdev = stdevMat.at<double>(0, 0);		//标准差
	/*static int i = 0;

	cv::Mat imgRoiScale;
	resize(imgRoi, imgRoiScale, cv::Size(imgRoi.cols * 10, imgRoi.rows * 10));
	std::ostringstream os1, os2;
	os1 << std::setprecision(5) << "Definition: " << stdev;
	cv::putText(imgRoiScale, os1.str(), cv::Point2i(10, 30), cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(255, 255, 255), 1);
	if (stdev > 20)
	os2 << std::setprecision(5) << "BA Weight: " << 1.0;
	else
	os2 << std::setprecision(5) << "BA Weight: " << sqrt(stdev / 20.);
	cv::putText(imgRoiScale, os2.str(), cv::Point2i(10, 60), cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(255, 255, 255), 1);



	char index[10];
	itoa(i, index, 10);
	std::string name = "F:\\标志点清晰度\\";
	name += index;
	name += ".bmp";
	cv::imwrite(name, imgRoiScale);

	i++;*/

	return stdev;
}



Feature::MarkerMask Feature::getMarkerMask(const cv::RotatedRect& rectEllipse)
{
	Feature::MarkerMask markerMask;
	double a(0), b(0);
	int minx, maxx;
	int miny, maxy;
	double x, y;

	double totalError = 0;
	double tempError = 0;
	double meanError = 0;
	double transAngle = 0;
	double pixAngel = 0;
	double pixDist = 0;
	double ri = 0;

	if (rectEllipse.size.height > rectEllipse.size.width)
	{
		a = rectEllipse.size.height / 2 + 4;
		b = rectEllipse.size.width / 2 + 4;
	}
	else
	{
		a = rectEllipse.size.width / 2 + 4;
		b = rectEllipse.size.height / 2 + 4;
	}

	minx = rectEllipse.center.x - a;
	maxx = rectEllipse.center.x + a;
	miny = rectEllipse.center.y - b;
	maxy = rectEllipse.center.y + b;
	
	markerMask.oriPos.x = minx;
	markerMask.oriPos.y = miny;

	markerMask.mask=cv::Mat(2 * a + 2, 2 * b + 2, CV_8UC1, cv::Scalar::all(0));
	for (int i = minx; i < maxx; i++)
	{
		for (int j = miny; j < maxy; j++)
		{
			x = (i - rectEllipse.center.x)*cos(transAngle) -
				(j - rectEllipse.center.y)*sin(transAngle);
			y = (i - rectEllipse.center.x)*sin(transAngle) +
				(j - rectEllipse.center.y)*cos(transAngle);

			if (x*x / (a*a) + (y*y) / (b*b) < 1)
				markerMask.mask.at<unsigned char>(i - minx, j - miny) = 0;
			else markerMask.mask.at<unsigned char>(i - minx, j - miny) = 1;
		}
	}

	return markerMask;
}


};