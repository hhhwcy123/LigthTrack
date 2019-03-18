#include <opencv2/core/core.hpp>
#include <opencv2/core/cuda.hpp>


int randomInt(int minVal, int maxVal);

cv::Mat randomMat(cv::RNG& rng, cv::Size size, int type, double minVal, double maxVal, bool useRoi);

double randomDouble(double minVal, double maxVal);

cv::Size randomSize(int minVal, int maxVal);

cv::Scalar randomScalar(double minVal, double maxVal);

cv::Mat randomMat(cv::Size size, int type, double minVal, double maxVal);

cv::cuda::GpuMat createMat(cv::Size size, int type, bool useRoi);

cv::cuda::GpuMat loadMat(const cv::Mat& m, bool useRoi);

cv::Mat getMat(cv::InputArray arr);