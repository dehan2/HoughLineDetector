#include "EdgeDetector.hpp"
#include "ConstForHoughTransformer.hpp"


void EdgeDetector::detectEdges(const cv::Mat& image, cv::Mat& edges)
{
	edges = image.clone();
	apply_gray_scale(edges);
	apply_gaussian_blur(edges, BLUR_SIGMA);
	apply_canny(edges);
}



void EdgeDetector::apply_gray_scale(cv::Mat& image)
{
	cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
}



void EdgeDetector::apply_gaussian_blur(cv::Mat& image, const double& sigma)
{
	cv::GaussianBlur(image, image, cv::Size(0, 0), sigma);
}



void EdgeDetector::apply_canny(cv::Mat& image)
{
	cv::Canny(image, image, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2);
}
