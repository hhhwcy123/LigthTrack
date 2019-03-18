#pragma once
#include <Cuda\Cuda_Wrapper.h>


cv::Mat randomMat(cv::RNG& rng, cv::Size size, int type, double minVal, double maxVal, bool useRoi)
{
	cv::Size size0 = size;
	if (useRoi)
	{
		size0.width += std::max(rng.uniform(0, 10) - 5, 0);
		size0.height += std::max(rng.uniform(0, 10) - 5, 0);
	}

	cv::Mat m(size0, type);

	rng.fill(m, cv::RNG::UNIFORM, minVal, maxVal);
	if (size0 == size)
		return m;
	return m(cv::Rect((size0.width - size.width) / 2, (size0.height - size.height) / 2, size.width, size.height));
}

int randomInt(int minVal, int maxVal)
{
	cv::RNG rng;
	return rng.uniform(minVal, maxVal);
}

double randomDouble(double minVal, double maxVal)
{
	cv::RNG rng;
	return rng.uniform(minVal, maxVal);
}

cv::Size randomSize(int minVal, int maxVal)
{
	return cv::Size(randomInt(minVal, maxVal), randomInt(minVal, maxVal));
}

cv::Scalar randomScalar(double minVal, double maxVal)
{
	return cv::Scalar(randomDouble(minVal, maxVal), randomDouble(minVal, maxVal), randomDouble(minVal, maxVal), randomDouble(minVal, maxVal));
}

cv::Mat randomMat(cv::Size size, int type, double minVal, double maxVal)
{
	cv::RNG rng;
	return randomMat(rng, size, type, minVal, maxVal, false);
}

cv::cuda::GpuMat createMat(cv::Size size, int type, bool useRoi)
{
	cv::Size size0 = size;

	if (useRoi)
	{
		size0.width += randomInt(5, 15);
		size0.height += randomInt(5, 15);
	}

	cv::cuda::GpuMat d_m(size0, type);

	if (size0 != size)
		d_m = d_m(cv::Rect((size0.width - size.width) / 2, (size0.height - size.height) / 2, size.width, size.height));

	return d_m;
}

cv::cuda::GpuMat loadMat(const cv::Mat& m, bool useRoi)
{
	cv::cuda::GpuMat d_m = createMat(m.size(), m.type(), useRoi);
	d_m.upload(m);
	return d_m;
}

cv::Mat getMat(cv::InputArray arr)
{
	if (arr.kind() == cv::_InputArray::CUDA_GPU_MAT)
	{
		cv::Mat m;
		arr.getGpuMat().download(m);
		return m;
	}

	return arr.getMat();
}