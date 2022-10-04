#include <iostream>
#include<filesystem>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include<opencv2/aruco.hpp>
//#include<opencv2/aruco_detector.hpp>
#include<opencv2/aruco/dictionary.hpp>
#include <opencv2/imgproc.hpp>

#include"camera-markers.h"

//#define MARKER_DETECT
#define LINES_DETECT
//#define WRITE

using namespace cv;
using namespace std;
using namespace camMarkers;



void main()
{

    int lowT = 45;
    int HighT = 200;
    cameraMarkers Markers(lowT, HighT);

#ifdef MARKER_DETECT
    // create arruco markers
    cv::Mat marker = Markers.createArucoMarkers(23, 800);
#ifdef WRITE
    imwrite("aruco-23-new.jpg", marker);
#endif // WRITE
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    Markers.detectMarkers(marker, markerCorners, markerIds, rejectedCandidates);
#endif // MARKER_DETECT

#ifdef LINES_DETECT

    std::string inImage = "target.jpg";
    cv::Mat src = imread(inImage, IMREAD_COLOR); // Load an image
    //namedWindow(cv::String("Original-Image"), WINDOW_AUTOSIZE);
    //imshow(cv::String("Original-Image"), src);

    //// canny threshold
    //Markers.CannyThreshold(src);
    //cv::Mat edges = Markers.getDetectedEdges();
    //cv::Mat dst;
    //if (!edges.empty())
    //{
    //    src.copyTo(dst, edges);
    //    imshow(cv::String("Edge-Map"), edges);
    //}

    //// Contour detection
    //// input binary edge map
    //std::vector<std::pair<pt2f, pt2f>>linePts;
    //linePts = Markers.detectContours(src,150,1000);

    //cv::Point2f intersect_pt;
    //bool stat = Markers.intersection(linePts[0].first, linePts[0].second, linePts[1].first, linePts[1].second, intersect_pt);
    //cv::Point pt1(int(intersect_pt.x), int(intersect_pt.y));
    //cv::circle(src, pt1, 7, Scalar(0, 0, 255), -1);
    //imshow(cv::String("line-Image"), src);


    cv::Mat  gray;
    std::vector< Vec4f> lines;
    cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();
    cv::cvtColor(src, gray,cv::COLOR_BGR2GRAY);
    lsd->detect(gray, lines);
    for (int i = 0;i < lines.size(); ++i)
        cv::line(src,cv::Point2f(lines[i][0], lines[i][1]), cv::Point2f(lines[i][2], lines[i][3]), (0,  0, 255));
    cv::imwrite("LSDLines.jpg", src);
    waitKey(0);

#endif // LINES_DETECT


}
