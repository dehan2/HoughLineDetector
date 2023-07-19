#include "ConstForHoughTransformer.hpp"
#include "EdgeDetector.hpp"
#include "HoughTransformer.hpp"
#include "Point2.hpp"
#include <opencv2/opencv.hpp>
#include <array>
#define _USE_MATH_DEFINES
#include <math.h>

array<Point2, 2> calculate_line_end_points_from_polar_coordinates(const Point2& polarCoordinates, const cv::Mat& image);

int main(int argc, char** argv)
{
	cv::Mat image = cv::imread("road.jpg");
	cv::Mat edges;
	
	EdgeDetector edgeDetector;
	edgeDetector.detectEdges(image, edges);

	cv::imshow("edges", edges);

	HoughTransformer houghTransformer;
	houghTransformer.detect_lines(edges);

	for (const Point2& polarCoordinates : houghTransformer.get_detected_lines())
	{
		array<Point2, 2> lineEndPoints = calculate_line_end_points_from_polar_coordinates(polarCoordinates, image);
		cv::line(image, cv::Point(lineEndPoints.at(0).x(), lineEndPoints.at(0).y()) , cv::Point(lineEndPoints.at(1).x(), lineEndPoints.at(1).y()), cv::Scalar(0, 0, 255), 2);
	}

	cv::imshow("image", image);
	cv::waitKey(0);

	return 0;
}



array<Point2, 2> calculate_line_end_points_from_polar_coordinates(const Point2& polarCoordinates, const cv::Mat& image)
{
	const double& r = polarCoordinates.x();
	const double& theta = polarCoordinates.y() * M_PI / 180;

	double yCross = r / sin(theta);
	double xCross = r / cos(theta);

	// y = ax + b form
	double b = yCross;
	double a = -yCross / xCross;

	//Find four points: x=0, x=image.cols, y=0, y=image.rows
	int y0 = ceil(yCross);
	int yBoundary = floor(a * image.cols + b);
	int x0 = ceil(xCross);
	int xBoundary = floor((image.rows - b) / a);

	list<Point2> boundaryPoints;
	if(y0 >= 0 && y0 < image.rows)
		boundaryPoints.push_back(Point2(0, y0));
	if (yBoundary >= 0 && yBoundary < image.rows)
		boundaryPoints.push_back(Point2(image.cols, yBoundary));
	if (x0 >= 0 && x0 < image.cols)
		boundaryPoints.push_back(Point2(x0, 0));
	if (xBoundary >= 0 && xBoundary < image.cols)
		boundaryPoints.push_back(Point2(xBoundary, image.rows));

	cout<<"Polar: ["<<polarCoordinates.x()<<", "<< polarCoordinates.y()<<"], y0: "<<y0<<", yBoundary : "<<yBoundary<<", x0 : "<<x0<<", xBoundary : "<<xBoundary<<endl;

	return { boundaryPoints.front(), boundaryPoints.back() };
}