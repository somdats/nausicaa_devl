#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <time.h>
#include <fstream>
#include <Eigen/Dense>
#include"ocam_functions.h"
#include"..\headers\calib_converter.h"

using namespace cv;
using namespace std;


static std::string fishEyeCalibrationFile = "D:/CamImages/calib_results.txt";

struct OcamCalibData
{
    const int32_t pd = 5;
    float poly[5];
    const int32_t ipd = 15;
    float inv_poly[15];
    float c;
    float d;
    float e;
    float cx;
    float cy;
    float iw;
    float ih;
};
bool UndistortImageMei(const ocam_model& ocm, cv::Mat& map1, cv::Mat& map2);
bool loadOcamCalibFile(const std::string& calib_f_name, OcamCalibData& calib_data);
bool loadOcamCalibFileCPP(const std::string& calib_f_name, OcamCalibData& calib_data);
void undistortImageOcamCalib(const ocam_model& cd,
    const uint8_t* img,
    int num_ch,
    const Eigen::Matrix3f& new_cam_mat_inv,
    uint8_t* o_img,
    int32_t ow,
    int32_t oh,
    int32_t onum_ch)
{
    double min_polyval = 1e100;
    double max_polyval = 0;
    for (auto j = 0; j < oh; ++j)
    {
        for (auto i = 0; i < ow; ++i)
        {
            Eigen::Vector2f p((float)i, (float)j);
            Eigen::Vector3f tmp = new_cam_mat_inv * p.homogeneous();
            p = tmp.hnormalized();

            float dist = p.norm();

            float rho = atan2f(-1, dist);
            float tmp_rho = 1;
            float polyval = 0;
            for (auto k = 0; k < cd.length_invpol; ++k)
            {
                float coeff = cd.invpol[/*ocam_model_inv.size() - 1 - */k];
                polyval += coeff * tmp_rho;
                tmp_rho *= rho;
            }
            if (polyval < min_polyval)
                min_polyval = polyval;
            if (polyval > max_polyval)
                max_polyval = polyval;
            float xx = p.x() / dist * polyval;
            float yy = p.y() / dist * polyval;

            xx = yy * cd.e + xx + cd.yc;
            yy = yy * cd.c + xx * cd.d + cd.xc;


            if (yy < (float)cd.height && xx < (float)cd.width && yy > 0 && xx > 0)
            {
                //val = img[int(yy) * int(cd.iw) * num_ch + int(xx) * num_ch];
                memcpy(&o_img[j * ow * onum_ch + i * onum_ch], &img[int(yy) * int(cd.width) * num_ch + int(xx) * num_ch], onum_ch);
            }
            else
            {
                memset(&o_img[j * ow * onum_ch + i * onum_ch], 0, onum_ch);
            }
            //o_img[j * ow *onum_ch + i * onum_ch] = val;
        }
    }
    min_polyval = 0;
    max_polyval = 0;
}
void undistortImageOcam(const std::vector<float>& ocam_model_inv,
    float c,
    float d,
    float e,
    float cx,
    float cy,
    uint8_t* img,
    int32_t iw,
    int32_t ih,
    float new_cam_mat[9],
    uint8_t* o_img,
    int32_t ow,
    int32_t oh,
    float cam_rot_x)
{

    for (auto j = 0; j < oh; ++j)
    {
        for (auto i = 0; i < ow; ++i)
        {
            //float x = i - (float) ow / 2.0f;
            //float y = j - (float) oh / 2.0f;
            float x = (i - new_cam_mat[2]) / new_cam_mat[0];
            float y = (j - new_cam_mat[5]) / new_cam_mat[4];
            float z = 1;

            float alfa = cam_rot_x / 180.0f * 3.141592f;

            float co = cosf(alfa);
            float si = sinf(alfa);

            float x2 = co * x + si * z;
            z = -si * x + co * z;
            x = x2 / z;
            y /= z;
            z /= z;

            float dist = sqrtf(x * x + y * y);

            float rho = atan2f(-z, dist);
            float tmp_rho = 1;
            float polyval = 0;
            for (auto k = 0; k < ocam_model_inv.size(); ++k)
            {
                float coeff = ocam_model_inv[/*ocam_model_inv.size() - 1 - */k];
                polyval += coeff * tmp_rho;
                tmp_rho *= rho;
            }

            float xx = x / dist * polyval;
            float yy = y / dist * polyval;

            xx = yy * e + xx + cx;
            yy = yy * c + xx * d + cy;

            uint8_t val = 0;
            if (yy < (float)ih && xx < (float)iw && yy > 0 && xx > 0)
                val = img[int(yy) * iw + int(xx)];
            o_img[j * ow + i] = val;
        }
    }

}


int main()
{

    std::string image_name = "D:/CamImages/28032022_test0.jpg";
    std::string calib_name = "D:/TestDump/Calibration_12082022/calib_results_12082022.txt";
   // OcamCalibData ocd;
    struct ocam_model ocd;
    get_ocam_model(&ocd, calib_name.c_str());
    /*if (!loadOcamCalibFileCPP(calib_name, ocd))
        return -1;*/

   /* cv::Mat img = cv::imread(image_name);

    cv::Mat gray1, gray2, gray3, gray4;
    cv::cvtColor(img, gray1, cv::COLOR_BGR2GRAY);*/

    // image_name = "d:/tmp/conti_calib/front_0/1_calib00006.png";
    // img = cv::imread(image_name);
    // cv::cvtColor(img, gray2, cv::COLOR_BGR2GRAY);
    //
    // image_name = "d:/tmp/conti_calib/front_0/2_calib00006.png";
    // img = cv::imread(image_name);
    // cv::cvtColor(img, gray3, cv::COLOR_BGR2GRAY);
    //
    // image_name = "d:/tmp/conti_calib/front_0/28032022_test0.jpg";
    // img = cv::imread(image_name);
    // cv::cvtColor(img, gray4, cv::COLOR_BGR2GRAY);

     //cv::imshow("image", gray1);

    ////////////////////////mei undistortion
    cv::Mat map1, map2, undistorted;
    UndistortImageMei(ocd, map1, map2);
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = "D:/TestDump/Calibration_12082022/*.jpg";

    cv::glob(path, images);
    for (auto img : images)
    {
        cv::Mat frame = cv::imread(img);
        cv::remap(frame, undistorted, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::imshow("undistorted", undistorted);
        /*std::string fileUnd = img + "_undistort.jpg";
        cv::imwrite("D:/TestDump/Calibration_12082022/Undistorted/" + fileUnd, undistorted);*/
        cv::waitKey(0);
    }
    

   /* cv::Mat undistorted(gray1.rows, gray1.cols, CV_8UC1);

    float n_f = ocd.width / (2 * tanf(46.1 / 180.0f * 3.141592f));
    Eigen::Matrix3f new_cam_mat;
    new_cam_mat <<
        n_f, 0.0f, ocd.width / 2.0f,
        0, n_f, ocd.height / 2.0f,
        0.0f, 0.0f, 1.0f;
    new_cam_mat = new_cam_mat * Eigen::Vector3f(0.5f, 0.5f, 1.0f).asDiagonal();
    Eigen::Matrix3f cam_mat_inv = new_cam_mat.inverse();
    undistortImageOcamCalib(ocd, gray1.data, 1,
        cam_mat_inv, undistorted.data, gray1.cols, gray1.rows, 1);*/

    //undistortImageOcam(ocd, gray2.data, 1,
    //    cam_mat_inv, undistorted1.data, gray2.cols, gray2.rows, 1);
    //
    //undistortImageOcam(ocd, gray3.data, 1,
    //    cam_mat_inv, undistorted2.data, gray3.cols, gray3.rows, 1);
    //int from = 0;
    //
   


}
bool UndistortImageMei(const ocam_model& ocm, cv::Mat& map1, cv::Mat& map2) {
    cv::Size imageSize(cv::Size(ocm.width,ocm.height));
    cv::Mat mapx_persp = cv::Mat(imageSize, CV_32FC1);
    cv::Mat mapy_persp = cv::Mat(imageSize, CV_32FC1); //CV_32FC1
    map1 = mapx_persp.clone();
    map2 = mapy_persp.clone();
    Eigen::Vector2d img_size(ocm.width, ocm.height);
   
    Eigen::Vector2d principal_point{ ocm.yc, ocm.xc };
    std::vector<double>  invpoly;
    invpoly.assign(ocm.invpol, ocm.invpol + ocm.length_invpol);
    Eigen::Matrix3f K_out;
    std::array<float, 5> D_out;
    float xi_out;
    calib_converter::convertOcam2Mei(invpoly, principal_point, img_size, ocm.c, ocm.d, ocm.e, K_out, D_out, xi_out);
  
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F);
    cv::Mat distCoeffs = cv::Mat(1, 4, CV_32F);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            cameraMatrix.at<float>(i, j) =K_out(i, j);
        }



    }

    for (int i = 0; i < 4; ++i)
    {
        distCoeffs.at<float>(i) = D_out[i];
    }
   /* distCoeffs.at<float>(0) = -0.135759;
    distCoeffs.at<float>(1) = 0.257251;
    distCoeffs.at<float>(2) =-0.560308;
    distCoeffs.at<float>(3) = 1.844500;
    distCoeffs.at<float>(0) = 0;*/
    cv::Mat new_camera_matrix = cv::Mat(cv::Matx33f(float(ocm.width)/4, 0, cameraMatrix.at<float>(0, 2),
        0, float(ocm.width)/4, cameraMatrix.at<float>(1, 2),
        0, 0, 1));
    std::cout << "distortion coefficients:" << distCoeffs << std::endl;
    std::cout << "Camera Intrinsics for rectification:" << new_camera_matrix << std::endl;

    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    omnidir::initUndistortRectifyMap(cameraMatrix, distCoeffs, xi_out, R, new_camera_matrix, imageSize,
        CV_32F, map1, map2, cv::omnidir::RECTIFY_PERSPECTIVE);
    return true;
}

bool loadOcamCalibFile(const std::string& calib_f_name, OcamCalibData& calib_data)
{
    std::ifstream fs(calib_f_name);
    if (!fs.is_open())
        return false;
    std::string str;
    int poly_read = 0;
    while (!fs.eof())
    {
        getline(fs, str);
        if (str.size() == 0 || str[0] == '#')
            continue;
        if (poly_read == 0)
        {
            int32_t s;
            std::stringstream ss(str);
            ss >> s;
            for (int i = 0; i < calib_data.pd; ++i)
                ss >> calib_data.poly[i];
            ++poly_read;
        }
        else if (poly_read == 1)
        {
            int32_t s;
            std::stringstream ss(str);
            ss >> s;
            for (int i = 0; i < calib_data.ipd; ++i)
                ss >> calib_data.inv_poly[i];
            ++poly_read;
        }
        else if (poly_read == 2)
        {
            std::stringstream ss(str);
            ss >> calib_data.cy;
            ss >> calib_data.cx;
            ++poly_read;
        }
        else if (poly_read == 3)
        {
            std::stringstream ss(str);
            ss >> calib_data.c;
            ss >> calib_data.d;
            ss >> calib_data.e;
            ++poly_read;
        }
        else if (poly_read == 4)
        {
            std::stringstream ss(str);
            ss >> calib_data.ih;
            ss >> calib_data.iw;
        }
    }

    return poly_read == 4;
}

bool loadOcamCalibFileCPP(const std::string& calib_f_name, OcamCalibData& calib_data)
{
    std::ifstream fs(calib_f_name);
    if (!fs.is_open())
        return false;
    std::string str;
    int poly_read = 0;
    getline(fs, str);
    while (!fs.eof() && poly_read < 4)
    {
        getline(fs, str);
        str = str.substr(str.find(':') + 1);
        if (str.size() == 0 || str[0] == '#')
            continue;
        if (poly_read == 2)
        {
            std::stringstream ss(str);
            for (int i = 0; i < calib_data.pd; ++i)
                ss >> calib_data.poly[i];
            ++poly_read;
        }
        else if (poly_read == 3)
        {
            std::stringstream ss(str);
            memset(calib_data.inv_poly, 0, sizeof(calib_data.inv_poly));
            for (int i = 0; i < calib_data.ipd; ++i)
                ss >> calib_data.inv_poly[i];
            ++poly_read;
        }
        else if (poly_read == 1)
        {
            std::stringstream ss(str);
            ss >> calib_data.cy;
            ss >> calib_data.cx;
            ++poly_read;
        }
        else if (poly_read == 0)
        {
            std::stringstream ss(str);
            ss >> calib_data.ih;
            ss >> calib_data.iw;
            ++poly_read;
        }
    }
    calib_data.c = 1.0;
    calib_data.d = 0.0;
    calib_data.e = 0.0;
    return poly_read == 4;
}

//static void calcChessboardCorners(const Size& boardSize, const Size2d& squareSize, Mat& corners)
//{
//    // corners has type of CV_64FC3
//    corners.release();
//    int n = boardSize.width * boardSize.height;
//    corners.create(n, 1, CV_64FC3);
//    Vec3d* ptr = corners.ptr<Vec3d>();
//    for (int i = 0; i < boardSize.height; ++i)
//    {
//        for (int j = 0; j < boardSize.width; ++j)
//        {
//            ptr[i * boardSize.width + j] = Vec3d(double(j * squareSize.width), double(i * squareSize.height), 0.0);
//        }
//    }
//}
//
//static bool detecChessboardCorners(const vector<string>& list, vector<string>& list_detected,
//    vector<Mat>& imagePoints, Size boardSize, Size& imageSize)
//{
//    imagePoints.resize(0);
//    list_detected.resize(0);
//    int n_img = (int)list.size();
//    Mat img;
//    for (int i = 0; i < n_img; ++i)
//    {
//        cout << list[i] << "... ";
//        Mat points;
//        img = imread(list[i], IMREAD_GRAYSCALE);
//        bool found = findChessboardCorners(img, boardSize, points);
//        if (found)
//        {
//         /*   cv::drawChessboardCorners(img, cv::Size(6,9), points, found);
//            cv::imshow("Image", img);
//            if (cv::waitKey(1) == 'b') {
//                break;
//            }*/
//            if (points.type() != CV_64FC2)
//                points.convertTo(points, CV_64FC2);
//            imagePoints.push_back(points);
//            list_detected.push_back(list[i]);
//        }
//        cout << (found ? "FOUND" : "NO") << endl;
//    }
//    if (!img.empty())
//        imageSize = img.size();
//    if (imagePoints.size() < 3)
//        return false;
//    else
//        return true;
//}
//
//static bool readStringList(const string& filename, vector<string>& l)
//{
//    l.resize(0);
//    FileStorage fs(filename, FileStorage::READ);
//    if (!fs.isOpened())
//        return false;
//    FileNode n = fs.getFirstTopLevelNode();
//    if (n.type() != FileNode::SEQ)
//        return false;
//    FileNodeIterator it = n.begin(), it_end = n.end();
//    for (; it != it_end; ++it)
//        l.push_back((string)*it);
//    return true;
//}
//
//static void saveCameraParams(const string& filename, int flags, const Mat& cameraMatrix,
//    const Mat& distCoeffs, const double xi, const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
//    vector<string> detec_list, const Mat& idx, const double rms, const vector<Mat>& imagePoints)
//{
//    FileStorage fs(filename, FileStorage::WRITE);
//
//    time_t tt;
//    time(&tt);
//    struct tm* t2 = localtime(&tt);
//    char buf[1024];
//    strftime(buf, sizeof(buf) - 1, "%c", t2);
//
//    fs << "calibration_time" << buf;
//
//    if (!rvecs.empty())
//        fs << "nFrames" << (int)rvecs.size();
//
//    if (flags != 0)
//    {
//        sprintf(buf, "flags: %s%s%s%s%s%s%s%s%s",
//            flags & omnidir::CALIB_USE_GUESS ? "+use_intrinsic_guess" : "",
//            flags & omnidir::CALIB_FIX_SKEW ? "+fix_skew" : "",
//            flags & omnidir::CALIB_FIX_K1 ? "+fix_k1" : "",
//            flags & omnidir::CALIB_FIX_K2 ? "+fix_k2" : "",
//            flags & omnidir::CALIB_FIX_P1 ? "+fix_p1" : "",
//            flags & omnidir::CALIB_FIX_P2 ? "+fix_p2" : "",
//            flags & omnidir::CALIB_FIX_XI ? "+fix_xi" : "",
//            flags & omnidir::CALIB_FIX_GAMMA ? "+fix_gamma" : "",
//            flags & omnidir::CALIB_FIX_CENTER ? "+fix_center" : "");
//        //cvWriteComment( *fs, buf, 0 );
//    }
//
//    fs << "flags" << flags;
//
//    fs << "camera_matrix" << cameraMatrix;
//    fs << "distortion_coefficients" << distCoeffs;
//    fs << "xi" << xi;
//
//    //cvWriteComment( *fs, "names of images that are acturally used in calibration", 0 );
//    fs << "used_imgs" << "[";
//    for (int i = 0; i < (int)idx.total(); ++i)
//    {
//        fs << detec_list[(int)idx.at<int>(i)];
//    }
//    fs << "]";
//
//    if (!rvecs.empty() && !tvecs.empty())
//    {
//        Mat rvec_tvec((int)rvecs.size(), 6, CV_64F);
//        for (int i = 0; i < (int)rvecs.size(); ++i)
//        {
//            Mat(rvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(0, i, 3, 1)));
//            Mat(tvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(3, i, 3, 1)));
//        }
//        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
//        fs << "extrinsic_parameters" << rvec_tvec;
//    }
//
//    fs << "rms" << rms;
//
//    if (!imagePoints.empty())
//    {
//        Mat imageMat((int)imagePoints.size(), (int)imagePoints[0].total(), CV_64FC2);
//        for (int i = 0; i < (int)imagePoints.size(); ++i)
//        {
//            Mat r = imageMat.row(i).reshape(2, imageMat.cols);
//            Mat imagei(imagePoints[i]);
//            imagei.copyTo(r);
//        }
//        fs << "image_points" << imageMat;
//    }
//}
//
//int main(int argc, char** argv)
//{
//    /*cv::CommandLineParser parser(argc, argv,
//        "{w||board width}"
//        "{h||board height}"
//        "{sw|1.0|square width}"
//        "{sh|1.0|square height}"
//        "{o|out_camera_params.xml|output file}"
//        "{fs|false|fix skew}"
//        "{fp|false|fix principal point at the center}"
//        "{@input||input file - xml file with a list of the images, created with cpp-example-imagelist_creator tool}"
//        "{help||show help}"
//    );
//    parser.about("This is a sample for omnidirectional camera calibration. Example command line:\n"
//        "    omni_calibration -w=6 -h=9 -sw=80 -sh=80 imagelist.xml \n");
//    if (parser.has("help") || !parser.has("w") || !parser.has("h"))
//    {
//        parser.printMessage();
//        return 0;
//    }*/
//
//    Size boardSize(6, 9);
//    Size2d squareSize(35, 35);
//    int flags =  omnidir::CALIB_FIX_SKEW | omnidir::CALIB_FIX_XI;
//    const string outputFilename ="D:/CamImages_28032022/calib_mei.txt";
//    const string inputFilename = "";
//
//    // Extracting path of individual image stored in a given directory
//    std::vector<cv::String> images;
//    vector<string> image_list, detec_list;
//    // Path of the folder containing checkerboard images
//    std::string path = "D:/CamImages_28032022/*.jpg"; //meiModelImages/
//
//    cv::glob(path, image_list);
//
//    // get image name list
//   
// /*   if (!readStringList(inputFilename, image_list))
//    {
//        cout << "Can not read imagelist" << endl;
//        return -1;
//    }*/
//
//    // find corners in images
//    // some images may be failed in automatic corner detection, passed cases are in detec_list
//    cout << "Detecting chessboards (" << image_list.size() << ")" << endl;
//    vector<Mat> imagePoints;
//    Size imageSize;
//    if (!detecChessboardCorners(image_list, detec_list, imagePoints, boardSize, imageSize))
//    {
//        cout << "Not enough corner detected images" << endl;
//        return -1;
//    }
//
//    // calculate object coordinates
//    vector<Mat> objectPoints;
//    Mat object;
//    calcChessboardCorners(boardSize, squareSize, object);
//    for (int i = 0; i < (int)detec_list.size(); ++i)
//        objectPoints.push_back(object);
//
//    // run calibration, some images are discarded in calibration process because they are failed
//    // in initialization. Retained image indexes are in idx variable.
//    Mat K, D, xi, idx;
//    vector<Vec3d> rvecs, tvecs;
//    double _xi, rms;
//    TermCriteria criteria(3, 2000, 1e-8);
//    rms = omnidir::calibrate(objectPoints, imagePoints, imageSize, K, xi, D, rvecs, tvecs, flags, criteria, idx);
//    _xi = xi.at<double>(0);
//    cout << "Saving camera params to " << outputFilename << endl;
//    saveCameraParams(outputFilename, flags, K, D, _xi,
//        rvecs, tvecs, detec_list, idx, rms, imagePoints);
//
//    // Trying to undistort the image using the camera parameters obtained from calibration
//
//  for(int i = 0; i < image_list.size();++i)
//  {
//      cv::Mat image;
//      image = cv::imread("D:/CamImages/undistorted_perspective_test_25032022.jpg");
//      cv::Mat dst, map1, map2, newMat ;
//      cv::Size newSize;
//     /* Mat newCamMatrix = cv::getOptimalNewCameraMatrix(K, D, imageSize, 1, imageSize, 0);
//      cv::omnidir::initUndistortRectifyMap(K, D, _xi, rvecs, newCamMatrix,imageSize, CV_32F, map1, map2, omnidir::RECTIFY_PERSPECTIVE);*/
//      //cv::Mat  new_camera_matrix;
//    /*  K.at<double>(0, 0) = 468.04;
//      K.at<double>(1, 1) = 468.04;*/
//      cv::Mat new_camera_matrix = cv::Mat(cv::Matx33f(K.at<double>(0,0)/2.0, 0, 1948.0/2.0,
//          0, K.at<double>(0, 0)/2.0, 1096.0/2.0,
//          0, 0, 1));
//      std::cout << new_camera_matrix << std::endl;
//      //new_camera_matrix = cv::getOptimalNewCameraMatrix(K, D, cv::Size(1948, 1096), 1, cv::Size(1948, 1096), 0);
//      cv::omnidir::undistortImage(image, dst, K, D, xi, omnidir::RECTIFY_PERSPECTIVE, new_camera_matrix, cv::Size(1948, 1096));
//      //cv::remap(image, dst, map1, map2, cv::INTER_LINEAR,cv::BORDER_CONSTANT);
//
//      //Displaying the undistorted image
//      cv::imshow("undistorted image",dst);
//
//      if (cv::waitKey(0) == 'b') {
//           break;
//      }
//  }
//}
