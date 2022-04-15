#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <time.h>

using namespace cv;
using namespace std;

static std::string fishEyeCalibrationFile = "D:/CamImages/calib_results.txt";

static void calcChessboardCorners(const Size& boardSize, const Size2d& squareSize, Mat& corners)
{
    // corners has type of CV_64FC3
    corners.release();
    int n = boardSize.width * boardSize.height;
    corners.create(n, 1, CV_64FC3);
    Vec3d* ptr = corners.ptr<Vec3d>();
    for (int i = 0; i < boardSize.height; ++i)
    {
        for (int j = 0; j < boardSize.width; ++j)
        {
            ptr[i * boardSize.width + j] = Vec3d(double(j * squareSize.width), double(i * squareSize.height), 0.0);
        }
    }
}

static bool detecChessboardCorners(const vector<string>& list, vector<string>& list_detected,
    vector<Mat>& imagePoints, Size boardSize, Size& imageSize)
{
    imagePoints.resize(0);
    list_detected.resize(0);
    int n_img = (int)list.size();
    Mat img;
    for (int i = 0; i < n_img; ++i)
    {
        cout << list[i] << "... ";
        Mat points;
        img = imread(list[i], IMREAD_GRAYSCALE);
        bool found = findChessboardCorners(img, boardSize, points);
        if (found)
        {
         /*   cv::drawChessboardCorners(img, cv::Size(6,9), points, found);
            cv::imshow("Image", img);
            if (cv::waitKey(1) == 'b') {
                break;
            }*/
            if (points.type() != CV_64FC2)
                points.convertTo(points, CV_64FC2);
            imagePoints.push_back(points);
            list_detected.push_back(list[i]);
        }
        cout << (found ? "FOUND" : "NO") << endl;
    }
    if (!img.empty())
        imageSize = img.size();
    if (imagePoints.size() < 3)
        return false;
    else
        return true;
}

static bool readStringList(const string& filename, vector<string>& l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((string)*it);
    return true;
}

static void saveCameraParams(const string& filename, int flags, const Mat& cameraMatrix,
    const Mat& distCoeffs, const double xi, const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
    vector<string> detec_list, const Mat& idx, const double rms, const vector<Mat>& imagePoints)
{
    FileStorage fs(filename, FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm* t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty())
        fs << "nFrames" << (int)rvecs.size();

    if (flags != 0)
    {
        sprintf(buf, "flags: %s%s%s%s%s%s%s%s%s",
            flags & omnidir::CALIB_USE_GUESS ? "+use_intrinsic_guess" : "",
            flags & omnidir::CALIB_FIX_SKEW ? "+fix_skew" : "",
            flags & omnidir::CALIB_FIX_K1 ? "+fix_k1" : "",
            flags & omnidir::CALIB_FIX_K2 ? "+fix_k2" : "",
            flags & omnidir::CALIB_FIX_P1 ? "+fix_p1" : "",
            flags & omnidir::CALIB_FIX_P2 ? "+fix_p2" : "",
            flags & omnidir::CALIB_FIX_XI ? "+fix_xi" : "",
            flags & omnidir::CALIB_FIX_GAMMA ? "+fix_gamma" : "",
            flags & omnidir::CALIB_FIX_CENTER ? "+fix_center" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "xi" << xi;

    //cvWriteComment( *fs, "names of images that are acturally used in calibration", 0 );
    fs << "used_imgs" << "[";
    for (int i = 0; i < (int)idx.total(); ++i)
    {
        fs << detec_list[(int)idx.at<int>(i)];
    }
    fs << "]";

    if (!rvecs.empty() && !tvecs.empty())
    {
        Mat rvec_tvec((int)rvecs.size(), 6, CV_64F);
        for (int i = 0; i < (int)rvecs.size(); ++i)
        {
            Mat(rvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(0, i, 3, 1)));
            Mat(tvecs[i]).reshape(1, 1).copyTo(rvec_tvec(Rect(3, i, 3, 1)));
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << rvec_tvec;
    }

    fs << "rms" << rms;

    if (!imagePoints.empty())
    {
        Mat imageMat((int)imagePoints.size(), (int)imagePoints[0].total(), CV_64FC2);
        for (int i = 0; i < (int)imagePoints.size(); ++i)
        {
            Mat r = imageMat.row(i).reshape(2, imageMat.cols);
            Mat imagei(imagePoints[i]);
            imagei.copyTo(r);
        }
        fs << "image_points" << imageMat;
    }
}

int main(int argc, char** argv)
{
    /*cv::CommandLineParser parser(argc, argv,
        "{w||board width}"
        "{h||board height}"
        "{sw|1.0|square width}"
        "{sh|1.0|square height}"
        "{o|out_camera_params.xml|output file}"
        "{fs|false|fix skew}"
        "{fp|false|fix principal point at the center}"
        "{@input||input file - xml file with a list of the images, created with cpp-example-imagelist_creator tool}"
        "{help||show help}"
    );
    parser.about("This is a sample for omnidirectional camera calibration. Example command line:\n"
        "    omni_calibration -w=6 -h=9 -sw=80 -sh=80 imagelist.xml \n");
    if (parser.has("help") || !parser.has("w") || !parser.has("h"))
    {
        parser.printMessage();
        return 0;
    }*/

    Size boardSize(6, 9);
    Size2d squareSize(35, 35);
    int flags =  omnidir::CALIB_FIX_SKEW | omnidir::CALIB_FIX_XI;
    const string outputFilename ="D:/CamImages_28032022/calib_mei.txt";
    const string inputFilename = "";

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    vector<string> image_list, detec_list;
    // Path of the folder containing checkerboard images
    std::string path = "D:/CamImages_28032022/*.jpg"; //meiModelImages/

    cv::glob(path, image_list);

    // get image name list
   
 /*   if (!readStringList(inputFilename, image_list))
    {
        cout << "Can not read imagelist" << endl;
        return -1;
    }*/

    // find corners in images
    // some images may be failed in automatic corner detection, passed cases are in detec_list
    cout << "Detecting chessboards (" << image_list.size() << ")" << endl;
    vector<Mat> imagePoints;
    Size imageSize;
    if (!detecChessboardCorners(image_list, detec_list, imagePoints, boardSize, imageSize))
    {
        cout << "Not enough corner detected images" << endl;
        return -1;
    }

    // calculate object coordinates
    vector<Mat> objectPoints;
    Mat object;
    calcChessboardCorners(boardSize, squareSize, object);
    for (int i = 0; i < (int)detec_list.size(); ++i)
        objectPoints.push_back(object);

    // run calibration, some images are discarded in calibration process because they are failed
    // in initialization. Retained image indexes are in idx variable.
    Mat K, D, xi, idx;
    vector<Vec3d> rvecs, tvecs;
    double _xi, rms;
    TermCriteria criteria(3, 2000, 1e-8);
    rms = omnidir::calibrate(objectPoints, imagePoints, imageSize, K, xi, D, rvecs, tvecs, flags, criteria, idx);
    _xi = xi.at<double>(0);
    cout << "Saving camera params to " << outputFilename << endl;
    saveCameraParams(outputFilename, flags, K, D, _xi,
        rvecs, tvecs, detec_list, idx, rms, imagePoints);

    // Trying to undistort the image using the camera parameters obtained from calibration

  for(int i = 0; i < image_list.size();++i)
  {
      cv::Mat image;
      image = cv::imread("D:/CamImages/undistorted_perspective_test_25032022.jpg");
      cv::Mat dst, map1, map2, newMat ;
      cv::Size newSize;
     /* Mat newCamMatrix = cv::getOptimalNewCameraMatrix(K, D, imageSize, 1, imageSize, 0);
      cv::omnidir::initUndistortRectifyMap(K, D, _xi, rvecs, newCamMatrix,imageSize, CV_32F, map1, map2, omnidir::RECTIFY_PERSPECTIVE);*/
      //cv::Mat  new_camera_matrix;
    /*  K.at<double>(0, 0) = 468.04;
      K.at<double>(1, 1) = 468.04;*/
      cv::Mat new_camera_matrix = cv::Mat(cv::Matx33f(K.at<double>(0,0)/2.0, 0, 1948.0/2.0,
          0, K.at<double>(0, 0)/2.0, 1096.0/2.0,
          0, 0, 1));
      std::cout << new_camera_matrix << std::endl;
      //new_camera_matrix = cv::getOptimalNewCameraMatrix(K, D, cv::Size(1948, 1096), 1, cv::Size(1948, 1096), 0);
      cv::omnidir::undistortImage(image, dst, K, D, xi, omnidir::RECTIFY_PERSPECTIVE, new_camera_matrix, cv::Size(1948, 1096));
      //cv::remap(image, dst, map1, map2, cv::INTER_LINEAR,cv::BORDER_CONSTANT);

      //Displaying the undistorted image
      cv::imshow("undistorted image",dst);

      if (cv::waitKey(0) == 'b') {
           break;
      }
  }
}
