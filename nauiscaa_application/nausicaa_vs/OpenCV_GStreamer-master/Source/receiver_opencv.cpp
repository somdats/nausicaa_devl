
#include "pch.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace cv;

#include <iostream>
#include <fstream>
#include <time.h>
using namespace std;
cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows) {
    int N = vecMat.size();
    nRows = nRows > N ? N : nRows;
    int edgeThickness = 10;
    int imagesPerRow = 2;// ceil(double(N) / nRows);
    int resizeHeight = floor(2.0 * ((floor(double(windowHeight - edgeThickness) / nRows)) / 2.0)) - edgeThickness;
    int maxRowLength = 0;

    std::vector<int> resizeWidth;
    for (int i = 0; i < N;) {
        int thisRowLen = 0;
        for (int k = 0; k < imagesPerRow; k++) {
            double aspectRatio = double(vecMat[i].cols) / vecMat[i].rows;
            int temp = int(ceil(resizeHeight * aspectRatio));
            resizeWidth.push_back(temp);
            thisRowLen += temp;
            if (++i == N) break;
        }
        if ((thisRowLen + edgeThickness * (imagesPerRow + 1)) > maxRowLength) {
            maxRowLength = thisRowLen + edgeThickness * (imagesPerRow + 1);
        }
    }
    int windowWidth = maxRowLength;
    cv::Mat canvasImage(windowHeight, windowWidth, CV_8UC3, Scalar(0, 0, 0));

    for (int k = 0, i = 0; i < nRows; i++) {
        int y = i * resizeHeight + (i + 1) * edgeThickness;
        int x_end = edgeThickness;
        for (int j = 0; j < imagesPerRow && k < N; k++, j++) {
            int x = x_end;
            cv::Rect roi(x, y, resizeWidth[k], resizeHeight);
            cv::Size s = canvasImage(roi).size();
            // change the number of channels to three
            cv::Mat target_ROI(s, CV_8UC3);
            if (vecMat[k].channels() != canvasImage.channels()) {
                if (vecMat[k].channels() == 1) {
                    cv::cvtColor(vecMat[k], target_ROI, COLOR_GRAY2BGR);
                }
            }
            else {
                vecMat[k].copyTo(target_ROI);
            }
            cv::resize(target_ROI, target_ROI, s);
            if (target_ROI.type() != canvasImage.type()) {
                target_ROI.convertTo(target_ROI, canvasImage.type());
            }
            target_ROI.copyTo(canvasImage(roi));
            x_end += resizeWidth[k] + edgeThickness;
        }
    }
    return canvasImage;
}

int main()
{
    Mat frame;
    cout << "OpenCV version : " << CV_VERSION << endl;
    cout << cv::getBuildInformation() << endl;
    cv::Mat win_mat;// (cv::Size(3896, 2192), CV_8UC3);
    std::vector<cv::Mat>frames;
    VideoCapture cap("udpsrc port=5000 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! h264parse ! decodebin ! videoconvert !  appsink drop=1",
        CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "VideoCapture-1 not opened" << endl;
        exit(-1);
    }

    VideoCapture cap2("udpsrc port=5001 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! h264parse ! decodebin ! videoconvert !  appsink drop=1",
        CAP_GSTREAMER);
    if (!cap2.isOpened()) {
        cerr << "VideoCapture-2 not opened" << endl;
        exit(-1);
    }

 /*   VideoCapture cap3("udpsrc port=5002 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! decodebin ! videoconvert !  appsink drop = 1",
        CAP_GSTREAMER);
    if (!cap3.isOpened()) {
        cerr << "VideoCapture-2 not opened" << endl;
        exit(-1);
    }

    VideoCapture cap4("udpsrc port=5003 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! decodebin ! videoconvert !  appsink",
        CAP_GSTREAMER);
    if (!cap4.isOpened()) {
        cerr << "VideoCapture-2 not opened" << endl;
        exit(-1);
    }*/

   
    int n = 0;
    int start = clock();
    char key;
    while (true) {

        Mat frame, frame2, frame3, frame4;
        cap.read(frame);
        cap2.read(frame2);
       // //cap3.read(frame3);
       // //cap4.read(frame4);
        frames.push_back(frame2);
        frames.push_back(frame);
        frames.push_back(frame2);
        frames.push_back(frame);
        frames.push_back(frame2);
        frames.push_back(frame);

        win_mat = makeCanvas(frames, 3288,8);
       // frames.clear();

        /*frame.copyTo(win_mat(cv::Rect(0, 0, 1948, 1096)));
        frame2.copyTo(win_mat(cv::Rect(1948, 0, 3896, 1096)));
        frame3.copyTo(win_mat(cv::Rect(0, 1096, 1948, 2192)));
        frame4.copyTo(win_mat(cv::Rect(1948, 1096, 3896, 2192)));*/
        /*  imshow("receiver-1", frame);
          imshow("receiver-2", frame2);
          imshow("receiver-3", frame3);
          imshow("receiver-4", frame4);*/
          // Display big mat
        cv::imshow("Images", win_mat);
        key = waitKey(30);

        if (key == 115) {
            auto currTime = std::chrono::system_clock::now();
            auto UTC = std::chrono::duration_cast<std::chrono::milliseconds>(currTime.time_since_epoch()).count();
            std::string fileName = (std::string("D:/TestDump/Calibration_12082022/") + std::to_string(12082022) + "_outdoor_" + std::to_string(n++) + ".jpg");
            bool writeSet = imwrite(fileName, frame2);
            cout << "s was pressed. saving image " << n << endl;
        }

        if (waitKey(1) == 'b') {
            break;
        }

    }


    return 0;
  
}
