#include "HoughTransformer.hpp"
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

void HoughTransformer::detect_lines(const cv::Mat& edgeImage)
{
	// Step 1. Extract edge points from edge image.
	list<Point2> edgePoints = extract_edge_points_from_image(edgeImage);

	// Step 2. Create accumulator array for polar coordinates.
	const int thetaMin = -90;
	const int thetaMax =  180;
	const int rMin = 0;
	const int rMax = sqrt(edgeImage.cols * edgeImage.cols + edgeImage.rows * edgeImage.rows); // Diagonal length of image.
	
	const int thetaStep = 1;
	const int thetaRange = floor((thetaMax - thetaMin + 1)/thetaStep);

	vector<list<const Point2*>> accumulator(rMax * thetaRange);

	// Step 3. For each edge point, calculate r and theta and increment accumulator array.
	for (const Point2& edgePoint : edgePoints)
		for (int thetaIndex = 0; thetaIndex < thetaRange; thetaIndex++)
		{
			double theta = thetaMin + (thetaIndex + 0.5) * thetaStep;
			int r = edgePoint.x() * cos(theta * M_PI / 180) + edgePoint.y() * sin(theta * M_PI / 180);
			int indexInAccumulator = r + thetaIndex * rMax;

			if(r >= 0 && r < rMax)
				accumulator.at(indexInAccumulator).push_back(&edgePoint);
		}

	// Step 4. Find the maximum value in the accumulator array.
	list<pair<Point2, int>> accumulationResult;
	for (int thetaIndex = 0; thetaIndex < thetaRange; thetaIndex++)
		for (int r = rMin; r < rMax; r++)
		{
			int indexInAccumulator = r + thetaIndex * rMax;
			double theta = thetaMin + (thetaIndex + 0.5) * thetaStep;
			accumulationResult.emplace_back(pair<Point2, int>(Point2(r, theta), accumulator.at(indexInAccumulator).size()));
		}

	accumulationResult.sort([](const pair<Point2, int>& lhs, const pair<Point2, int>& rhs) { return lhs.second > rhs.second; });

	double thetaIndex = (accumulationResult.front().first.y() - thetaMin) / thetaStep - 0.5;
	auto& mostHit = accumulator.at(accumulationResult.front().first.x() + thetaIndex * rMax);

	// Step 5. Find the lines.
	m_detectedLines.clear();
	for (auto& candidate : accumulationResult)
	{
		if (candidate.second > accumulationResult.front().second * 0.6)
		{
			m_detectedLines.emplace_back(candidate.first);
		}
		else
			break;
	}
}



list<Point2> HoughTransformer::extract_edge_points_from_image(const cv::Mat& edgeImage)
{
	list<Point2> edgePoints;

	for (int i = 0; i < edgeImage.cols; i++)
		for (int j = 0; j < edgeImage.rows; j++)
			if (edgeImage.at<uchar>(j, i) == 255)
				edgePoints.emplace_back(Point2(i, j));

	return edgePoints;
}
