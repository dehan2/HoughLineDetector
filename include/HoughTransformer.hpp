#pragma once

#include "Point2.hpp"
#include <opencv2/opencv.hpp>
#include <array>
#include <list>

using namespace std;

class HoughTransformer
{
private:
	list<Point2> m_detectedLines; // in polar coordinates - r and theta.

public:
	HoughTransformer() = default;
	~HoughTransformer() = default;

	const list<Point2>& get_detected_lines() const { return m_detectedLines; }

	void detect_lines(const cv::Mat& edgeImage);

private:
	list<Point2> extract_edge_points_from_image(const cv::Mat& edgeImage);
};

