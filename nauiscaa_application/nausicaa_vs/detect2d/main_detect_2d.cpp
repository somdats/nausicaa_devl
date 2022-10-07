#include <iostream>
#include<filesystem>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include<opencv2/aruco.hpp>
//#include<opencv2/aruco_detector.hpp>
#include<opencv2/aruco/dictionary.hpp>
#include <opencv2/imgproc.hpp>

#include"camera-markers.h"

#define MARKER_DETECT


using namespace cv;
using namespace std;
using namespace camMarkers;



void main()
{

    int lowT = 45;
    int HighT = 200;
    cameraMarkers Markers(lowT, HighT);

    //cv::Mat marker = Markers.createArucoMarkers(0, 800);
    //imwrite("aruco-0.jpg", marker);
    //  marker = Markers.createArucoMarkers(1, 800);
    //imwrite("aruco-1.jpg", marker);
    //  marker = Markers.createArucoMarkers(2, 800);
    //imwrite("aruco-2.jpg", marker);
    // marker = Markers.createArucoMarkers(3, 800);
    //imwrite("aruco-3.jpg", marker);

    

 /*   std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    cv::Mat marker = cv::imread("marker_new.jpg");
    Markers.detectMarkers(marker, markerCorners, markerIds, rejectedCandidates);
    
    cv::Mat outputImage = marker.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
   

    std::vector<cv::Point2f> pts[4];
    for (int i = 0; i < markerCorners.size(); ++i)
        if (markerIds[i] < 4)
            pts[markerIds[i]].push_back(markerCorners[i][0]);

    if (!pts[0].empty())
        cv::circle(outputImage, pts[0][0], 20, cv::Scalar(0, 0,255,255),20);
 
    cv::Point2f p;
    std::vector<cv::Point2f> int_points;
    for(int i= 1; i <= 3; ++i)        
        if (pts[i].size() == 2)
            for (int j = 1; j <= 3; ++j)
                if(i!=j)
                if (pts[j].size() == 2)
                    if (Markers.intersection(pts[i][0], pts[i][1], pts[j][0], pts[j][1], p))
                        int_points.push_back(p);
    for (int i = 1; i <= 3; ++i)
        if (pts[i].size() == 2)
            cv::line(outputImage,pts[i][0], pts[i][1], cv::Scalar((i == 0) ? 255 : 0, (i == 1) ? 255 : 0, (i == 2) ? 255 : 0), 2);

    for(int i = 0; i < int_points.size(); ++i)
        cv::circle(outputImage, pts[0][0], 30, cv::Scalar((i==0)?255:0, (i == 1) ? 255:0, (i == 2) ? 255:0), 10);

    cv::resize(outputImage, outputImage, cv::Size(outputImage.cols / 4, outputImage.rows / 4));
    cv::imshow("out", outputImage);*/

    cv::Mat marker = cv::imread("marker_new.jpg");
    cv::Point2f pos;
    Markers.detectMarker(marker, pos);
    cv::circle(marker,pos, 30, cv::Scalar(0,0,255), 10);
    cv::resize(marker, marker, cv::Size(marker.cols / 4, marker.rows / 4));
    cv::imshow("out", marker);
    cv::waitKey(0);
}
