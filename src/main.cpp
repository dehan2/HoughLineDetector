#include "ConstForHoughTransformer.hpp"
#include "EdgeDetector.hpp"
#include "HoughTransformer.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
	cv::Mat image = cv::imread("road.jpg");
	cv::Mat edges;
	
	EdgeDetector edgeDetector;
	edgeDetector.detectEdges(image, edges);

	cv::imshow("edges", edges);
	cv::waitKey(0);

	return 0;
}