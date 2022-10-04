#pragma once

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include"opencv2/ximgproc.hpp"
#include<opencv2/aruco.hpp>
//#include<opencv2/aruco_detector.hpp>
#include<opencv2/aruco/dictionary.hpp>

#include <iostream>
#include<filesystem>
#include<math.h>

using namespace cv;
using namespace std;

typedef  cv::Point2f pt2f;

namespace camMarkers {

	class cameraMarkers {
	public:
		cameraMarkers() {};
		cameraMarkers(int lt, int ht) : lowerThres(lt), highThres(ht)
		{};
		static cv::Mat createArucoMarkers(int id, int sizePixel, int borderSize = 1, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName = cv::aruco::DICT_6X6_250);
		static std::vector<cv::Mat> createMultipleMarkers(int numMarkers, int startId, int size, int borderSize = 1, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName = cv::aruco::DICT_6X6_250);

		void detectMarkers(cv::Mat inputMarkers, std::vector<std::vector<cv::Point2f>>& markerCorners, std::vector<int>& markerIds,
			std::vector<std::vector<cv::Point2f>>& rejectedCandidates, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName = cv::aruco::DICT_6X6_250);

		// Finds the intersection of two lines, or returns false.
			// The lines are defined by (o1, p1) and (o2, p2).
		bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
			Point2f& r);
		void ConvertMatToPoint(cv::Mat img, std::vector<cv::Point>& points);
		void ContourPtsToCVPts(const std::vector<cv::Point>& cnts, vector<Point2f>& pts);
		void CannyThreshold(const cv::Mat& srcImage);
		std::vector<std::pair<pt2f, pt2f>> detectContours(const cv::Mat& inImage, int lt = 1000, int ht = 1200, Scalar linecolor = Scalar(0, 255, 255));

		vector<Point> getContours(int idx);
		vector<vector<Point> >getContours();
		static void drawLines(const cv::Mat& cannyEdgeImage, const std::vector<cv::Point2f>& cvLines);
		cv::Mat getDetectedEdges();
		// returns a vector of cv::Mat, each Mat of size(4,1) representing a 2d ray-line with ( xo,yo) as origin pt. and dir vector(vx,vy)
		std::vector<cv::Mat> getAllFittedLines();
		bool MarkerDetected(const cv::Mat& inImage, int markerID1 = 1, int markerID2 = 2, int markerID3 = 3, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName = cv::aruco::DICT_6X6_250);

	protected:
		cv::Mat detectedEdges;
		//cv::Mat ArucoImage;
		vector<vector<Point> > contours;
		std::vector<cv::Mat>fittedLines;
		vector<Vec4i> hierarchy;
		int max_lowThreshold = 100;
		int ratio = 3;
		int kernel_size = 3;
		int lowerThres;
		int highThres;
	};
}
