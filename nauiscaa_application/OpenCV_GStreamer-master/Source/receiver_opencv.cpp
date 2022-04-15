
#include "pch.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace cv;

#include <iostream>
#include <fstream>
#include <time.h>
using namespace std;

#define FETCH_FOR_CALIBRATION


int main()
{
    Mat frame;
    cout << "OpenCV version : " << CV_VERSION << endl;
    cout << cv::getBuildInformation() << endl;

    VideoCapture cap("udpsrc port=5000 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! decodebin ! videoconvert !  appsink",
        CAP_GSTREAMER );
    if (!cap.isOpened()) {
        cerr << "VideoCapture-1 not opened" << endl;
        exit(-1);
    }

   /* VideoCapture cap2("udpsrc port=5001 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! decodebin ! videoconvert !  appsink",
        CAP_GSTREAMER);
    if (!cap2.isOpened()) {
        cerr << "VideoCapture-2 not opened" << endl;
        exit(-1);
    }*/

//#ifndef FETCH_FOR_CALIBRATION
//     cv::Mat  new_camera_matrix;
//     cv::Mat cameraMatrix,distCoeffs;
//     cameraMatrix = cv::Mat(3,3,CV_32F);
//     distCoeffs = cv::Mat(1,4,CV_32F);
//     FILE *ip=fopen("camera_intrinsics.txt","r");
//     for(int i=0; i < 9; ++i) {
//         float v;
//         fscanf(ip,"%f",&v);
//         cameraMatrix.at<float>(i/3,i%3) = v;
//     }
//     for(int i=0; i < 4; ++i) {
//         float v;
//         fscanf(ip,"%f",&v);
//         distCoeffs.at<float>(0,i) = v;
//     }
//     fclose(ip);
//     std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
//     std::cout << "distCoeffs : " << distCoeffs << std::endl;
//
//     cv::Mat dst, map1, map2;
//     cv::Size imageSize(cv::Size(1948,1096));
//     // Refining the camera matrix using parameters obtained by calibration
//     new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
//     cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs,   imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);
//
//#endif
     int n=0;
     int start = clock();
     char key;
    while (true) {

        Mat frame,frame2;
        cap.read(frame);
        //cap2.read(frame2);
        imshow("receiver-1", frame);
        //imshow("receiver-2", frame2);
        key = waitKey(30);
      
        if (key == 115) {
            std::string fileName = (std::string("D:/CamImages_28032022/") + std::string("28032022_new") + std::to_string(n++) + ".jpg");
            bool writeSet = imwrite(fileName, frame);
            cout << "s was pressed. saving image " << n << endl;
        }

        if (waitKey(1) == 'b') {
            break;
        }

//#ifndef FETCH_FOR_CALIBRATION
//        cv::remap(frame, dst, map1, map2, cv::INTER_LINEAR);
//        imshow("receiver und", dst);
//#else
//        imshow("receiver", frame);
//#endif
//
//
//#ifdef FETCH_FOR_CALIBRATION
//        if(clock()-start < 3000000) continue;
//        start = clock();
//        imwrite((std::string("D:/CamImages/")+std::to_string(n++)+".png").c_str(),frame);
//#endif
//
//        if (waitKey(1) == 'b') {
//             break;
//        }
    }


    return 0;
    /*VideoCapture cap("udpsrc port=5000 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! decodebin ! videoconvert !  appsink",
        CAP_GSTREAMER);

    if (!cap.isOpened()) {
        cerr << "VideoCapture not opened" << endl;
        exit(-1);
    }
    int n = 0;
    char key;
    while (true) {

        Mat frame;

        cap.read(frame);

        imshow("receiver", frame);
       
        key = waitKey(30);

        if (waitKey(1) == 27) {
            break;
        }
        if (key == 115) {
            std::string fileName = (std::string("D:/CamImages/") + std::string("Fisheye_") + std::to_string(n++) + ".jpg");
            bool writeSet = imwrite(fileName, frame);
            cout << "s was pressed. saving image " << writeSet << endl;
        }

        
    }

    return 0;*/

}
