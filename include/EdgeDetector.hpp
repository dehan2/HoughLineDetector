#pragma once

#include <opencv2/opencv.hpp>

class EdgeDetector
{
	public:
	EdgeDetector() = default;
	~EdgeDetector() = default;

	void detectEdges(const cv::Mat& image, cv::Mat& edges);

private:
	void apply_gray_scale(cv::Mat& image);
	void apply_gaussian_blur(cv::Mat& image, const double& sigma);
	void apply_canny(cv::Mat& image);
};

