#ifndef CAMERA_READER_H
#define CAMERA_READER_H

#include "defines.h"
#include "common.h"

#include <opencv2/opencv.hpp>

#include <mutex>
#include <thread>
#include "point_and_axis_calibration.h"
#include <vcg/math/camera.h>

#include"..\headers\ocam_functions.h"
#include"..\headers\calib_converter.h"
#include "opencv2/ccalib/omnidir.hpp"
#include "camera-markers.h"
#include"..\headers\imgproc.h"

struct Camera {
    Camera() :aligned(false), used(false) { p3.clear(); p2i.clear(); }
    Camera(const Camera& _) {}
    cv::VideoCapture cap;
    cv::Mat map1, map2;
    cv::Mat dst;
    unsigned char* oglTextureData;

    vcg::Camera<float> vcg_cam;
    cv::Mat cameraMatrix;
    vcg::Matrix44f cameraMatrix44;
    vcg::Matrix44f gl_cameraMatrix44;
    struct ocam_model o;
    int scaleFactor = 4;
    int camID;
    uint inStPort;
    int ax;
    bool aligned;
    bool used;
    cameraMarkers markerFinder;

    float _debk;

    void opencv2opengl_camera_params(cv::Mat cam, int wx, int wy, float nr, float& r, float& l, float& t, float& b) {
        float k = cam.at<float>(0, 0) / cam.at<float>(1, 1); // fx/fy
        float ks = nr / cam.at<float>(0, 0);

        r = (wx - cam.at<float>(0, 2)) * ks;
        l = -cam.at<float>(0, 2) * ks;

        // REVERSED CENTER FOR OPENCV->OPENGL (otherwise "wy-" should be on t
        t = (cam.at<float>(1, 2)) * k * ks;
        b = -(wy - cam.at<float>(1, 2)) * k * ks;
    }

    vcg::Matrix44f opencv2opengl_camera(cv::Mat cam, int wx, int wy, float nr, float fr) {
        vcg::Matrix44f P;
        P.SetIdentity();
        float k = cam.at<float>(0, 0) / cam.at<float>(1, 1); // fx/fy
        float ks = nr / cam.at<float>(0, 0);
        this->_debk = k * ks;

        float r = (wx - cam.at<float>(0, 2)) * ks;
        float l = -cam.at<float>(0, 2) * ks;

        // REVERSED CENTER FOR OPENCV->OPENGL (otherwise "wy-" should be on t
        float t = (cam.at<float>(1, 2)) * k * ks;
        float b = -(wy - cam.at<float>(1, 2)) * k * ks;

        P[0][0] = 2 * nr / (r - l);
        P[0][2] = (r + l) / (r - l);

        P[1][1] = 2 * nr / (t - b);
        P[1][2] = (t + b) / (t - b);

        P[2][2] = -(nr + fr) / (fr - nr);
        P[2][3] = -2 * fr * nr / (fr - nr);
        P[3][2] = -1;
        P[3][3] = 0;

        // DEBUG ----------------
//        float x = 2.0;
//        float y = 1.0;
//        float z = -4.0;

//        cv::Mat p3m, p3_pr;
//        p3m=cv::Mat(3,1,CV_32F);
//        p3m.at<float>(0,0) = x;
//        p3m.at<float>(1,0) = - y;
//        p3m.at<float>(2,0) = -z;
//        vcg::Point4f p4(x,y,z,1.0),p4_pr;


//        std::cout << " cam "<< cam<< std::endl;
//        p3_pr  = cam*p3m;
//        std::cout << " opencv p3_pr "<< p3_pr<< std::endl;
//        p3_pr /= p3_pr.at<float>(2,0);
//        std::cout << " opencv p3_pr norm "<< p3_pr<< std::endl;
//        p3_pr.at<float>(0,0) =  p3_pr.at<float>(0,0)*2.0/wx-1.0;
//        p3_pr.at<float>(1,0) =  (p3_pr.at<float>(1,0)/wy)*2.0-1.0;
//        p4_pr = P * p4;
//        p4_pr /= p4_pr[3];

//        std::cout << " opencv "<< p3_pr<< std::endl;
//        std::cout << " vcg "<< p4_pr.X() << " "<<  p4_pr.Y() <<" " <<  p4_pr.Z()<<  " " << p4_pr.W()<< std::endl;

// -----------------------

        return P;
    }

    void _debug() {
        vcg::Matrix44f P = this->opencv2opengl_camera(this->cameraMatrix, 1948, 1096, 0.5, 10.0);
        vcg::Matrix44f E = this->opengl_extrinsics();

        float x = 0.0;
        float y = 0.0;
        float z = 0.0;

        cv::Mat p3m, p3_pr;
        p3m = cv::Mat(3, 1, CV_32F);

        vcg::Point4f p4(x, y, z, 1.0), p4_vcg, p4_ocv;

        // to camera space
        p4_ocv = this->extrinsics * p4;
        p3m.at<float>(0, 0) = p4_ocv[0];
        p3m.at<float>(1, 0) = p4_ocv[1];
        p3m.at<float>(2, 0) = p4_ocv[2];

        p4_vcg = E * p4;

        std::cout << "CS opencv " << p3m << std::endl;
        std::cout << "CS vcg " << p4_vcg.X() << " " << p4_vcg.Y() << " " << p4_vcg.Z() << " " << p4_vcg.W() << std::endl;


        p3_pr = this->cameraMatrix * p3m;
        p3_pr /= p3_pr.at<float>(2, 0);
        p3_pr.at<float>(0, 0) = p3_pr.at<float>(0, 0) * 2.0 / 1948 - 1.0;
        p3_pr.at<float>(1, 0) = (p3_pr.at<float>(1, 0) / 1096) * 2.0 - 1.0;

        p4_vcg = P * p4_vcg;
        p4_vcg /= p4_vcg[3];


        std::cout << " opencv " << p3_pr << std::endl;
        std::cout << " vcg " << p4_vcg.X() << " " << p4_vcg.Y() << " " << p4_vcg.Z() << " " << p4_vcg.W() << std::endl;


        //        std::cout << " cam "<< cam<< std::endl;
        //        p3_pr  = cam*p3m;
        //        std::cout << " opencv p3_pr "<< p3_pr<< std::endl;
        //        p3_pr /= p3_pr.at<float>(2,0);
        //        std::cout << " opencv p3_pr norm "<< p3_pr<< std::endl;
        //        p3_pr.at<float>(0,0) =  p3_pr.at<float>(0,0)*2.0/wx-1.0;
        //        p3_pr.at<float>(1,0) =  (p3_pr.at<float>(1,0)/wy)*2.0-1.0;
        //        p4_pr = P * p4;
        //        p4_pr /= p4_pr[3];

        //        std::cout << " opencv "<< p3_pr<< std::endl;
        //        std::cout << " vcg "<< p4_pr.X() << " "<<  p4_pr.Y() <<" " <<  p4_pr.Z()<<  " " << p4_pr.W()<< std::endl;


    }

    vcg::Matrix44f  opengl_extrinsics() {
        vcg::Matrix44f res = extrinsics;

        vcg::Point3f t = res.GetColumn3(3);
        res.SetColumn(3, vcg::Point3f(0, 0, 0));

        res.transposeInPlace();
        t = res * t * (-1); // now t is the point of view

        res.SetColumn(1, res.GetColumn3(1) * -1);
        res.SetColumn(2, res.GetColumn3(2) * -1);
        res.transposeInPlace();
        t = res * t * (-1); // now t is the translation
        res.SetColumn(3, t);
        return res;
    }

    void computeOpenGLFrustrumScaramuzza(const cv::Mat& camMatrix, int wx, int wy, float nr, float& r, float& l, float& t, float& b) {
        float f = (camMatrix.at<float>(0, 0));
        float k = 1.0; // fx/fy
        float ks = nr / f;

        r = (wx - camMatrix.at<float>(0, 2)) * ks;
        l = -camMatrix.at<float>(0, 2) * ks;

        // REVERSED CENTER FOR OPENCV->OPENGL (otherwise "wy-" should be on t
        t = (camMatrix.at<float>(1, 2)) * k * ks;
        b = -(wy - camMatrix.at<float>(1, 2)) * k * ks;

    }

    cv::Mat computeOpenCVMatrixFromScaraMuzza(const ocam_model& camModel) {
        cv::Mat camMatrix = cv::Mat::zeros(3, 3, CV_32F);

        camMatrix.at<float>(0, 0) = -(camModel.pol[0]);
        camMatrix.at<float>(0, 2) = camModel.yc;
        camMatrix.at<float>(1, 1) = -(camModel.pol[0]);
        camMatrix.at<float>(1, 2) = camModel.xc;
        camMatrix.at<float>(2, 2) = 1;
        return camMatrix;



    }
    cv::Mat getDistCoeffiecient(const ocam_model& o)
    {
        int len = o.length_pol;
        cv::Mat distCoeffs = cv::Mat(1, 4, CV_32F);
        /* for (int i = 0; i < 4; ++i)
             distCoeffs.at<float>(0, i) = 0.0f;*/
        for (int i = 1; i < len; i++)
            distCoeffs.at<float>(0, i - 1) = o.pol[i];
        std::cout << "M = " << std::endl << " " << distCoeffs << std::endl << std::endl;
        return distCoeffs;
    }
    void setHistogramEqualize(bool stat)
    {
        histoEq = stat;
    }

    vcg::Matrix44f extrinsics;

    cv::Mat distCoeffs;

    vcg::Shotf calibrated;

    std::mutex latest_frame_mutex;

    bool reading;

    std::vector < cv::Point2f> p2i;             // 2D point correspondences
    std::vector<vcg::Point3f> p3;   // 3D point correspondences

    void init(uint port, std::string camera_intrinsics_file, int cameraID, bool scaramuzza = true, bool hEq = false);

    void export_camera_match(CameraMatch& c);

    void start_reading();
    void stop_reading();
    vcg::Shotf SolvePnP(std::vector<vcg::Point3f> p3vcg);
    vcg::Shotf SolvePnP_new(std::vector <Correspondence3D2D> corrs);
    bool histoEq = false;

    std::vector<std::pair<unsigned long long, std::string>> timed_images;
};

#endif // CAMERA_READER_H
