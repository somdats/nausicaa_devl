/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images

   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM

   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"
#include <opencv2/imgproc/imgproc.hpp>
#include"calib_converter.h"
#include "opencv2/ccalib/omnidir.hpp"

static std::string fishEyeCalibrationFile = "D:/CamImages/calib_results_25032022.txt";
static std::string catadioptricCalibrationFile = "";

void DrawMarkersImage(cv::Mat& inImage, std::vector<cv::Point2i>markers, int markerType, cv::Scalar color)

{
    int markerSize = 15; int thickness = 2;
    int count = 0;
    for (const cv::Point2i& mk : markers)
    {
        std::string txt = std::to_string(count);
        cv::Scalar txtColor(255, 255, 255);
        cv::putText(inImage, txt, mk, cv::FONT_HERSHEY_SIMPLEX, 1, txtColor, 1, cv::LINE_8);
        cv::drawMarker(inImage, mk, color, markerType, markerSize, thickness);
        ++count;

    }
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

std::string typestr(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

//int main(int argc, char *argv[])
//{   
//  /* --------------------------------------------------------------------*/
//  /* Read the parameters of the omnidirectional camera from the TXT file */
//  /* --------------------------------------------------------------------*/
//  struct ocam_model o, o_cata; // our ocam_models for the fisheye and catadioptric cameras
//  get_ocam_model(&o, fishEyeCalibrationFile.c_str());
//  //get_ocam_model(&o_cata, catadioptricCalibrationFile.c_str());
//  /* --------------------------------------------------------------------*/    
//  /* Print ocam_model parameters                                         */
//  /* --------------------------------------------------------------------*/  
//  int i;
//  printf("pol =\n");    for (i=0; i<o.length_pol; i++){    printf("\t%e\n",o.pol[i]); };    printf("\n");
//  printf("invpol =\n"); for (i=0; i<o.length_invpol; i++){ printf("\t%e\n",o.invpol[i]); }; printf("\n");  
//  printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o.xc,o.yc,o.width,o.height);
//
//  /* --------------------------------------------------------------------*/
//  /* WORLD2CAM projects 3D point into the image                          */
//  /* NOTE!!! The coordinates are expressed according the C convention,   */
//  /* that is, from the origin (0,0) instead than from 1 (MATLAB).        */
//  /* --------------------------------------------------------------------*/
//  double point3D[3] = { 1.0,1.0,1.0 };       // a sample 3D point
//  double point2D[2];                              // the image point in pixel coordinates  
//  world2cam(point2D, point3D, &o); // The behaviour of this function is the same as in MATLAB
//  
//  /* --------------------------------------------------------------------*/  
//  /* Display re-projected coordinates                                    */
//  /* --------------------------------------------------------------------*/  
//  printf("\nworld2cam: pixel coordinates reprojected onto the image\n");  
//  printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);
//
//
//  /* --------------------------------------------------------------------*/
//  /* CAM2WORLD back-projects pixel points on to the unit sphere          */
//  /* The behaviour of this function is the same as in MATLAB             */
//  /* --------------------------------------------------------------------*/
//
//  cam2world(point3D, point2D, &o); 
//
//  /* --------------------------------------------------------------------*/  
//  /* Display back-projected normalized coordinates (on the unit sphere)  */
//  /* --------------------------------------------------------------------*/  
//  printf("\ncam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)\n");
//  printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);
//  
//  /* --------------------------------------------------------------------*/  
//  /* Allocate space for the unistorted images                            */
//  /* --------------------------------------------------------------------*/  
//  cv::Mat src1         = cv::imread("D:/CamImages/25032022_new0.jpg");      // source image 1
//  //cv::Mat src2         = cv::imread("./test_catadioptric.jpg");      // source image 2  
//
//  cv::Mat dst_persp   = cv::Mat( src1.size(),src1.type());   // undistorted perspective and panoramic image
//  //cv::Size size_pan_image = cv::Size(1200,400);        // size of the undistorted panoramic image
//  //cv::Mat dst_pan     = cv::Mat(size_pan_image, CV_8UC3, 3);     // undistorted panoramic image
//  std::cout << typestr(src1.type()) << std::endl;
//
//  cv::Mat mapx_persp = cv::Mat(src1.size(), CV_32FC1);
//  cv::Mat mapy_persp = cv::Mat(src1.size(), CV_32FC1);
//  //cv::Mat mapx_pan   = cv::Mat(dst_pan.rows,dst_pan.cols, CV_32FC1);
//  //cv::Mat mapy_pan   = cv::Mat(dst_pan.rows, dst_pan.cols, CV_32FC1);
//  
//  /* --------------------------------------------------------------------  */  
//  /* Create Look-Up-Table for perspective undistortion                     */
//  /* SF is kind of distance from the undistorted image to the camera       */
//  /* (it is not meters, it is justa zoom fator)                            */
//  /* Try to change SF to see how it affects the result                     */
//  /* The undistortion is done on a  plane perpendicular to the camera axis */
//  /* --------------------------------------------------------------------  */
//  float sf = 4;
//  create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &o, sf );
//
//  /* --------------------------------------------------------------------  */  
//  /* Create Look-Up-Table for panoramic undistortion                       */
//  /* The undistortoin is just a simple cartesia-to-polar transformation    */
//  /* Note, only the knowledge of image center (xc,yc) is used to undisort the image      */
//  /* xc, yc are the row and column coordinates of the image center         */
//  /* Note, if you would like to flip the image, just inverte the sign of theta in this function */
//  /* --------------------------------------------------------------------  */  
// // float Rmax = 470;  // the maximum radius of the region you would like to undistort into a panorama
//  //float Rmin = 20;   // the minimum radius of the region you would like to undistort into a panorama  
//  //create_panoramic_undistortion_LUT ( mapx_pan, mapy_pan, Rmin, Rmax, o_cata.xc, o_cata.yc);  
//
//  /* --------------------------------------------------------------------*/  
//  /* Undistort using specified interpolation method                      */
//  /* Other possible values are (see OpenCV doc):                         */
//  /* CV_INTER_NN - nearest-neighbor interpolation,                       */
//  /* CV_INTER_LINEAR - bilinear interpolation (used by default)          */
//  /* CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives moire-free results. In case of zooming it is similar to CV_INTER_NN method. */
//  /* CV_INTER_CUBIC - bicubic interpolation.                             */
//  /* --------------------------------------------------------------------*/
//  cv::remap( src1, dst_persp, mapx_persp, mapy_persp, cv::INTER_LINEAR,cv::WARP_FILL_OUTLIERS,cv::Scalar(0,0,0));//,CV_WARP_FILL_OUTLIERS, cvScalarAll(0) 
//  //cv::remap(src2, dst_pan, mapx_pan, mapy_pan, cv::INTER_LINEAR, cv::WARP_FILL_OUTLIERS, cv::Scalar(0, 0, 0));
//
//  
//  /* --------------------------------------------------------------------*/
//  /* Display image                                                       */
//  /* --------------------------------------------------------------------*/  
//  //drawMarker(src1, cv::Point(point2D[0], point2D[1]), cv::Scalar(255, 0, 0), cv::MarkerTypes::MARKER_CROSS, 20, 3, 8);
//  cv::namedWindow( "Original fisheye camera image",1 );
//  cv::imshow( "Original fisheye camera image", src1 );
//
//  cv::namedWindow( "Undistorted Perspective Image", 1 );
//  cv::imshow( "Undistorted Perspective Image", dst_persp );
//  
// /* cv::namedWindow( "Original Catadioptric camera image", 1 );
//  cv::imshow( "Original Catadioptric camera image", src2 );*/
//
//  //cv::namedWindow( "Undistorted Panoramic Image", 1 );
//  //cv::imshow( "Undistorted Panoramic Image", dst_pan );
//
//  /* --------------------------------------------------------------------*/    
//  /* Save image                                                          */
//  /* --------------------------------------------------------------------*/  
//  cv::imwrite("D:/CamImages/undistorted_perspective_test_25032022_sc.jpg", dst_persp);
//  //printf("\nImage %s saved\n","D:/CamImages/undistorted_perspective.jpg");
//
//  /*cv::imwrite("undistorted_panoramic.jpg",dst_pan);
//  printf("\nImage %s saved\n","undistorted_panoramic.jpg");*/
//
//  /* --------------------------------------------------------------------*/    
//  /* Wait until key presses                                              */
//  /* --------------------------------------------------------------------*/  
//  cv::waitKey();
//
//  /* --------------------------------------------------------------------*/    
//  /* Free memory                                                         */
//  /* --------------------------------------------------------------------*/  
///*  cvReleaseImage(&src1);
//  cvReleaseImage(&src2);  
//  cvReleaseImage(&dst_persp);
//  cvReleaseImage(&dst_pan);  
//  cvReleaseMat(&mapx_persp);
//  cvReleaseMat(&mapy_persp);  
//  cvReleaseMat(&mapx_pan);
//  cvReleaseMat(&mapy_pan);*/    
//
//  return 0;
//}

void main()
{
    struct ocam_model o;
    get_ocam_model(&o, fishEyeCalibrationFile.c_str());
    cv::Mat src1 = cv::imread("D:/CamImages/rectified_streamed_output_04052022.jpg"); //rectified_streamed_output
    std::vector<cv::Point2i> p2, pt_ex, pt_ex2;

    // temporary hard-coded image pixel values -gt
    p2.push_back(cv::Point2i(793, 344));
    p2.push_back(cv::Point2i(1630, 202));
    p2.push_back(cv::Point2i(633, 380));
    p2.push_back(cv::Point2i(1380, 30));
    /*p2.push_back(cv::Point2i(433, 1028));
    p2.push_back(cv::Point2i(816, 1029));*/

    cv::Scalar color(0, 0, 255); // BGR
    DrawMarkersImage(src1, p2, 0, color);

    // image pixel values - only PnP
    pt_ex.push_back(cv::Point2i(793, 343));
    pt_ex.push_back(cv::Point2i(1629, 205));
    pt_ex.push_back(cv::Point2i(634, 381));
    pt_ex.push_back(cv::Point2i(1379, 26));
    /*pt_ex.push_back(cv::Point2i(383, 978));
    pt_ex.push_back(cv::Point2i(888, 918));*/

    cv::Scalar color_1(0, 255, 255);
    DrawMarkersImage(src1, pt_ex, 5, color_1);
    cv::imwrite("D:/CamImages/image_with_marker_04052022_f4.jpg", src1);

    //////////////////////

    double theta = 2.0 * (atan2(1948 / 2.0, 475.04) * 180 / 3.141592);
    Eigen::Vector2d img_size(o.width, o.height);
    Eigen::Matrix3f K_out;
    std::array<float, 5> D_out;
    float xi_out;
    Eigen::Vector2d principal_point{ o.yc, o.xc };
    std::vector<double>  invpoly;
    invpoly.assign(o.invpol, o.invpol + o.length_invpol);
    calib_converter::convertOcam2Mei(invpoly, principal_point, img_size, o.c, o.d, o.e, K_out, D_out, xi_out);
    cv::Mat dst, map1, map2, newMat;
    cv::Size newSize;
    cv::Mat d = cv::Mat(1, 4, CV_32F);
    cv::Mat K(3, 3, cv::DataType<float>::type);
    //K = computeOpenCVMatrixFromScaraMuzza(o);
    //d = getDistCoeffiecient(o);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            K.at<float>(i, j) = K_out(i, j);
        }


    }

    for (int i = 0; i < 4; ++i)
    {
        d.at<float>(i) = D_out[i];
    }

    std::cout << K << std::endl;
    std::cout << "distortion coefficients:" << d << std::endl;
    cv::Size s = src1.size();
    cv::Mat Knew = cv::Mat(cv::Matx33f(-o.pol[0], 0, K.at<float>(0, 2),
        0, -o.pol[0], K.at<float>(1, 2),
        0, 0, 1));
    std::cout << Knew << std::endl;
    cv::Mat Ro = cv::Mat::eye(3, 3, CV_32F);
    /*   cv::omnidir::undistortImage(src1, dst, K, d, xi_out, cv::omnidir::RECTIFY_PERSPECTIVE, Knew, s,Ro);
       cv::Mat R = cv::Mat::eye(3, 3, CV_32F);*/
    cv::omnidir::initUndistortRectifyMap(K, d, xi_out, Ro, Knew, s,
        CV_32F, map1, map2, cv::omnidir::RECTIFY_PERSPECTIVE);
    cv::remap(src1, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    // cv::fisheye::undistortImage(src1, dst, K, d, K, s);
    cv::imwrite("D:/CamImages/undistorted_perspective_test_conv_new_updated_test.jpg", dst);

    // // convert ocam to mei model

    // //cv::Mat src1 = cv::imread("D:/CamImages/25032022_new0.jpg");
    ///* cv::namedWindow("Original fisheye camera image", 1);
    // cv::imshow("Original fisheye camera image", src1);*/



    cv::Mat cameraMatrix = computeOpenCVMatrixFromScaraMuzza(o);

    cv::Size imageSize(cv::Size(1948, 1096));
    // cv::Mat map1, map2;
    cv::Mat distCoeffs = getDistCoeffiecient(o);

    ///*cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
    //std::cout << "distCoeffs UND : " << distCoeffs << std::endl;*/

    //cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, imageSize, CV_16SC2, map1, map2);

    //cv::remap(src1, dst, map1, map2, cv::INTER_LINEAR, cv::WARP_FILL_OUTLIERS, cv::Scalar(0, 0, 0));

    std::vector<cv::Point3f>points;
    // Creating the Rotation Matrix
    cv::Mat R(3, 3, cv::DataType<float>::type);

    R.at<float>(0, 0) = 1;
    R.at<float>(1, 0) = 0;
    R.at<float>(2, 0) = 0;

    R.at<float>(0, 1) = 0;
    R.at<float>(1, 1) = 1;
    R.at<float>(2, 1) = 0;

    R.at<float>(0, 2) = 0;
    R.at<float>(1, 2) = 0;
    R.at<float>(2, 2) = 1;
    cv::Mat r, t;
    r = cv::Mat(3, 1, CV_32F);
    t = cv::Mat(3, 1, CV_32F);
    cv::Rodrigues(R, r);
    t.at<float>(0) = 0.0f;
    t.at<float>(1) = 0.0f;
    t.at<float>(2) = 0.0f;
    points.push_back(cv::Point3f(1.0, 1.0, 1.0));
    std::vector<cv::Point2f>point2D;
    cv::fisheye::projectPoints(points, point2D, r, t, cameraMatrix, distCoeffs);
    int point_cv1 = std::floor(point2D[0].x);
    int point_cv2 = std::floor(point2D[0].y);
    std::cout << " Projected to " << point2D[0] << std::endl;

    double Point3D[3] = { 1.0,1.0,1.0 };       // a sample 3D point
    double Point2D[2];                              // the image point in pixel coordinates  
    world2cam(Point2D, Point3D, &o);
    int point_sc1 = std::floor(Point2D[0]);
    int point_sc2 = std::floor(Point2D[1]);
    printf("m_row= %2.4f, m_col=%2.4f\n", Point2D[0], Point2D[1]);
    int markerSize = 30; int thickness = 2;
    /* cv::drawMarker(src1, cv::Point(point_cv1, point_cv2), cv::Vec3b(0, 0, 200), cv::MARKER_CROSS,markerSize,thickness );
     cv::drawMarker(src1, cv::Point(point_sc1, point_sc2), cv::Vec3b(255, 0, 0), cv::MARKER_CROSS, markerSize, thickness);
     cv::imwrite("D:/CamImages/undistorted_perspective_test_25032022_marker.jpg", src1); */




     /* cv::namedWindow("Undistorted Perspective Image", 1);
      cv::imshow("Undistorted Perspective Image", dst);
      cv::imwrite("D:/CamImages/undistorted_perspective_test_25032022_new.jpg", dst);*/

    cv::waitKey(0);
}
