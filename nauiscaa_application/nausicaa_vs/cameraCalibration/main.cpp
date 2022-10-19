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
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <filesystem>

#include <chrono>

namespace fs = std::filesystem;
using namespace cv;
bool clahe = false;

using std::chrono::high_resolution_clock;

enum ConvolutionType {
    /* Return the full convolution, including border */
    CONVOLUTION_FULL,
    /* Return only the part that corresponds to the original image */
    CONVOLUTION_SAME,
    /* Return only the submatrix containing elements that were not influenced by the border */
    CONVOLUTION_VALID
};



static std::string fishEyeCalibrationFile = "D:/naus/nausicaa_devl/nauiscaa_application/calib_results_02092022.txt";
static std::string catadioptricCalibrationFile = "";

const int histSize = 256;

//#define RECTIFY 

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
void OutputTime(const std::string& str, const high_resolution_clock::time_point& start_time, const high_resolution_clock::time_point& end_time)
{
    std::cout << str << ":" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << "ms" << std::endl;
    return;
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
typedef struct {
    Eigen::Matrix3f meiCameraMatrix;
    std::array<float, 5> distortionParameter;
    float xiFactor;
}MeiCalibration;
std::string inputFolderPath = "D:/Nausicaa_Data/Data_222092022/";
std::string outputRootPath = "D:/Nausicaa_Data/Data_rectified/";
std::string meiCalibFile = "D:/naus/nausicaa_devl/nauiscaa_application/calib_results_02092022_mei.txt";



bool readMeiCalibration(std::string calibrationFile, MeiCalibration& meiCalib)
{

    FILE* f;
    char buf[CMV_MAX_BUF];
    int i;
    //Open file
    if (!(f = fopen(calibrationFile.c_str(), "r")))
    {
        printf("File %s cannot be opened\n", calibrationFile);
        return false;
    }
    //Read xi- constant
     //Read center coordinates
    //fscanf(f, "\n");
    fgets(buf, CMV_MAX_BUF, f);
    std::cout << "reading:" << buf << std::endl;
    fscanf(f, "\n");
    char szParam1[50], szParam2[50], szParam3[50];
    fscanf(f, "%s", szParam1);
    meiCalib.xiFactor = atof(szParam1);

    //Read xi- camera parameter
    fscanf(f, "\n");
    fgets(buf, CMV_MAX_BUF, f);
    std::cout << "reading:" << buf << std::endl;
    fscanf(f, "\n");

    float val = -1;
    for (i = 0; i < 9; i++)
    {

        fscanf(f, " %s", szParam2);
        int r = i % 3;
        int c = i / 3;
        meiCalib.meiCameraMatrix(c, r) = atof(szParam2);
    }

    //Read distortion parameters
    fscanf(f, "\n");
    fgets(buf, CMV_MAX_BUF, f);
    std::cout << "reading:" << buf << std::endl;
    fscanf(f, "\n");
    for (i = 0; i < 5; i++)
    {
        fscanf(f, " %s", szParam3);
        meiCalib.distortionParameter[i] = atof(szParam3);
    }

    fclose(f);
    return true;
}

void drawHistogram(cv::Mat& b_hist, std::string channel, bool eq = false) {

    int hist_w = 512;
    int hist_h = 400;
    int bin_w = cvRound((double)hist_w / histSize);

    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1,
        cv::Mat());
    /* cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1,
         cv::Mat());
     cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1,
         cv::Mat());*/

    for (int i = 1; i < histSize; i++) {
        cv::Scalar color;
        if (channel == "Blue")
            color = cv::Scalar(255, 0, 0);
        if (channel == "Green")
            color = cv::Scalar(0, 255, 0);
        if (channel == "Red")
            color = cv::Scalar(0, 0, 255);
        if (eq)
        {
            cv::line(
                histImage,
                cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
                cv::Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
                cv::Scalar(0, 255, 255), 2, 8, 0);
            continue;
        }
        cv::line(
            histImage,
            cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
            cv::Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
            color, 2, 8, 0);
        /*  cv::line(
              histImage,
              cv::Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
              cv::Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
              cv::Scalar(0, 255, 0), 2, 8, 0);
          cv::line(
              histImage,
              cv::Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
              cv::Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
              cv::Scalar(0, 0, 255), 2, 8, 0);*/
    }
    if (eq)
    {
        std::string histoChannel = "histogram-" + channel;
        cv::namedWindow(cv::String(histoChannel), cv::WINDOW_NORMAL);
        cv::imshow(cv::String(histoChannel), histImage);
        return;
    }
    std::string histoChannel = "histogram-" + channel;
    cv::namedWindow(cv::String(histoChannel), cv::WINDOW_NORMAL);
    cv::imshow(cv::String(histoChannel), histImage);

}

void adaptiveImageEnhancement(const cv::Mat& src, cv::Mat& dst)
{
    int r = src.rows;
    int c = src.cols;
    int n = r * c;

    cv::Mat HSV;
    cv::cvtColor(src, HSV, COLOR_BGR2HSV_FULL);
    std::vector<cv::Mat> HSV_channels;
    cv::split(HSV, HSV_channels);
    cv::Mat S = HSV_channels[1];
    cv::Mat V = HSV_channels[2];

    int ksize = 5;
    cv::Mat gauker1 = cv::getGaussianKernel(ksize, 15);
    cv::Mat gauker2 = cv::getGaussianKernel(ksize, 80);
    cv::Mat gauker3 = cv::getGaussianKernel(ksize, 250);

    cv::Mat gauV1, gauV2, gauV3;
    cv::filter2D(V, gauV1, CV_64F, gauker1, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    cv::filter2D(V, gauV2, CV_64F, gauker2, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    cv::filter2D(V, gauV3, CV_64F, gauker3, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    cv::Mat V_g = (gauV1 + gauV2 + gauV3) / 3.0;

    cv::Scalar avg_S = cv::mean(S);
    double k1 = 0.1 * avg_S[0];
    double k2 = avg_S[0];

    cv::Mat V_double;
    V.convertTo(V_double, CV_64F);

    cv::Mat V1 = ((255 + k1) * V_double).mul(1.0 / (cv::max(V_double, V_g) + k1));
    cv::Mat V2 = ((255 + k2) * V_double).mul(1.0 / (cv::max(V_double, V_g) + k2));

    cv::Mat X1 = V1.reshape(0, n);
    cv::Mat X2 = V2.reshape(0, n);

    cv::Mat X(n, 2, CV_64F);
    X1.copyTo(X(cv::Range(0, n), cv::Range(0, 1)));
    X2.copyTo(X(cv::Range(0, n), cv::Range(1, 2)));

    cv::Mat covar, mean;
    cv::calcCovarMatrix(X, covar, mean, COVAR_NORMAL | COVAR_ROWS, CV_64F);

    cv::Mat eigenValues; //The eigenvalues are stored in the descending order.
    cv::Mat eigenVectors; //The eigenvectors are stored as subsequent matrix rows.
    cv::eigen(covar, eigenValues, eigenVectors);

    double w1 = eigenVectors.at<double>(0, 0) / (eigenVectors.at<double>(0, 0) + eigenVectors.at<double>(0, 1));
    double w2 = 1 - w1;

    cv::Mat F = w1 * V1 + w2 * V2;

    F.convertTo(F, CV_8U);

    HSV_channels[2] = F;
    cv::merge(HSV_channels, HSV);
    cv::cvtColor(HSV, dst, COLOR_HSV2BGR_FULL);

    return;
}

void WTHE(const cv::Mat& src, cv::Mat& dst, float ru = 0.9, float vu = 0.9)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat YUV;
    std::vector<cv::Mat> YUV_channels;
    if (channels == 1) {
        L = src.clone();
    }
    else {
        cv::cvtColor(src, YUV, COLOR_BGR2YUV);
        cv::split(YUV, YUV_channels);
        L = YUV_channels[0];
    }

    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat hist;
    calcHist(&L, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    float total_pixels_inv = 1.0f / total_pixels;
    cv::Mat P = hist.clone();
    for (int i = 0; i < 256; i++) {
        P.at<float>(i) = P.at<float>(i) * total_pixels_inv;
    }

    cv::Mat Pwt = P.clone();
    double minP, maxP;
    cv::minMaxLoc(P, &minP, &maxP);
    float Pu = vu * maxP;
    float Pl = minP;
    for (int i = 0; i < 256; i++) {
        float Pi = P.at<float>(i);
        if (Pi > Pu)
            Pwt.at<float>(i) = Pu;
        else if (Pi < Pl)
            Pwt.at<float>(i) = 0;
        else
            Pwt.at<float>(i) = std::pow((Pi - Pl) / (Pu - Pl), ru) * Pu;
    }

    cv::Mat Cwt = Pwt.clone();
    float cdf = 0;
    for (int i = 0; i < 256; i++) {
        cdf += Pwt.at<float>(i);
        Cwt.at<float>(i) = cdf;
    }

    float Wout = 255.0f;
    float Madj = 0.0f;
    std::vector<uchar> table(256, 0);
    for (int i = 0; i < 256; i++) {
        table[i] = cv::saturate_cast<uchar>(Wout * Cwt.at<float>(i) + Madj);
    }

    cv::LUT(L, table, L);


    if (channels == 1) {
        dst = L.clone();
    }
    else {
        cv::merge(YUV_channels, dst);
        cv::cvtColor(dst, dst, COLOR_YUV2BGR);
    }
    //histogram after processing
    cv::Mat eqHist;
    calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
    //drawhistogram
    drawHistogram(eqHist, "eq-Blue", true);
    return;
}

void JHE(const cv::Mat& src, cv::Mat& dst)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat YUV;
    std::vector<cv::Mat> YUV_channels;
    if (channels == 1) {
        L = src.clone();
    }
    else {
        cv::cvtColor(src, YUV, COLOR_BGR2YUV);
        cv::split(YUV, YUV_channels);
        L = YUV_channels[0];
    }

    // Compute average image.
    cv::Mat avg_L;
    cv::boxFilter(L, avg_L, -1, cv::Size(3, 3), cv::Point(-1, -1), true, cv::BORDER_CONSTANT);

    // Computer joint histogram.
    cv::Mat jointHist = cv::Mat::zeros(256, 256, CV_32S);
    for (int r = 0; r < rows; r++) {
        uchar* L_it = L.ptr<uchar>(r);
        uchar* avg_L_it = avg_L.ptr<uchar>(r);
        for (int c = 0; c < cols; c++) {
            int i = L_it[c];
            int j = avg_L_it[c];
            jointHist.at<int>(i, j)++;
        }
    }

    // Compute CDF.
    cv::Mat CDF = cv::Mat::zeros(256, 256, CV_32S);
    int min_CDF = total_pixels + 1;
    int cumulative = 0;
    for (int i = 0; i < 256; i++) {
        int* jointHist_it = jointHist.ptr<int>(i);
        int* CDF_it = CDF.ptr<int>(i);
        for (int j = 0; j < 256; j++) {
            int count = jointHist_it[j];
            cumulative += count;
            if (cumulative > 0 && cumulative < min_CDF)
                min_CDF = cumulative;
            CDF_it[j] = cumulative;
        }
    }

    // Compute equalized joint histogram.
    cv::Mat h_eq = cv::Mat::zeros(256, 256, CV_8U);
    for (int i = 0; i < 256; i++) {
        uchar* h_eq_it = h_eq.ptr<uchar>(i);
        int* cdf_it = CDF.ptr<int>(i);
        for (int j = 0; j < 256; j++) {
            int cur_cdf = cdf_it[j];
            h_eq_it[j] = cv::saturate_cast<uchar>(255.0 * (cur_cdf - min_CDF) / (total_pixels - 1));
        }
    }

    // Map to get enhanced image.
    for (int r = 0; r < rows; r++) {
        uchar* L_it = L.ptr<uchar>(r);
        uchar* avg_L_it = avg_L.ptr<uchar>(r);
        for (int c = 0; c < cols; c++) {
            int i = L_it[c];
            int j = avg_L_it[c];
            L_it[c] = h_eq.at<uchar>(i, j);
        }
    }

    if (channels == 1) {
        dst = L.clone();
    }
    else {
        cv::merge(YUV_channels, dst);
        cv::cvtColor(dst, dst, COLOR_YUV2BGR);
    }
    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat eqHist;
    calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
    //drawhistogram
    drawHistogram(eqHist, "eq-Blue", true);
    return;
}


void AGCIE(const cv::Mat& src, cv::Mat& dst)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1) {
        L = src.clone();
    }
    else {
        cv::cvtColor(src, HSV, COLOR_BGR2HSV_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    cv::Mat L_norm;
    L.convertTo(L_norm, CV_64F, 1.0 / 255.0);

    cv::Mat mean, stddev;
    cv::meanStdDev(L_norm, mean, stddev);
    double mu = mean.at<double>(0, 0);
    double sigma = stddev.at<double>(0, 0);

    double tau = 3.0;

    double gamma;
    //if (4 * sigma <= 1.0 / tau) { // low-contrast
    //    gamma = -std::log2(sigma);
    //}
    //else { // high-contrast
    //    gamma = std::exp((1.0 - mu - sigma) / 2.0);
    //}
    gamma = 0.45;

    std::vector<double> table_double(256, 0);
    for (int i = 1; i < 256; i++) {
        table_double[i] = i / 255.0;
    }

    if (mu >= 0.5) { // bright image
        for (int i = 1; i < 256; i++) {
            table_double[i] = std::pow(table_double[i], gamma);
        }
    }
    else { // dark image
        double mu_gamma = std::pow(mu, gamma);
        for (int i = 1; i < 256; i++) {
            double in_gamma = std::pow(table_double[i], gamma);;
            table_double[i] = in_gamma / (in_gamma + (1.0 - in_gamma) * mu_gamma);
        }
    }

    std::vector<uchar> table_uchar(256, 0);
    for (int i = 1; i < 256; i++) {
        table_uchar[i] = cv::saturate_cast<uchar>(255.0 * table_double[i]);
    }

    cv::LUT(L, table_uchar, L);

    if (channels == 1) {
        dst = L.clone();
    }
    else {
        cv::merge(HSV_channels, dst);
        cv::cvtColor(dst, dst, COLOR_HSV2BGR_FULL);
    }
    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat eqHist;
    calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
    //drawhistogram
    drawHistogram(eqHist, "eq-Blue", true);

    return;
}

void GCEHistMod(const cv::Mat& src, cv::Mat& dst, int threshold = 5, int b = 23, int w = 230, double alpha = 2, int g = 10)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1) {
        L = src.clone();
    }
    else {
        cv::cvtColor(src, HSV, COLOR_BGR2HSV_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    std::vector<int> hist(256, 0);

    int k = 0;
    int count = 0;
    for (int r = 0; r < rows; r++) {
        const uchar* data = L.ptr<uchar>(r);
        for (int c = 0; c < cols; c++) {
            int diff = (c < 2) ? data[c] : std::abs(data[c] - data[c - 2]);
            k += diff;
            if (diff > threshold) {
                hist[data[c]]++;
                count++;
            }
        }
    }

    double kg = k * g;
    double k_prime = kg / std::pow(2, std::ceil(std::log2(kg)));

    double umin = 10;
    double u = std::min(count / 256.0, umin);

    std::vector<double> modified_hist(256, 0);
    double sum = 0;
    for (int i = 0; i < 256; i++) {
        if (i > b && i < w)
            modified_hist[i] = std::round((1 - k_prime) * u + k_prime * hist[i]);
        else
            modified_hist[i] = std::round(((1 - k_prime) * u + k_prime * hist[i]) / (1 + alpha));
        sum += modified_hist[i];
    }

    std::vector<double> CDF(256, 0);
    double culsum = 0;
    for (int i = 0; i < 256; i++) {
        culsum += modified_hist[i] / sum;
        CDF[i] = culsum;
    }

    std::vector<uchar> table_uchar(256, 0);
    for (int i = 1; i < 256; i++) {
        table_uchar[i] = cv::saturate_cast<uchar>(255.0 * CDF[i]);
    }

    cv::LUT(L, table_uchar, L);

    if (channels == 1) {
        dst = L.clone();
    }
    else {
        cv::merge(HSV_channels, dst);
        cv::cvtColor(dst, dst, COLOR_HSV2BGR_FULL);
    }
    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat eqHist;
    calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
    //drawhistogram
    drawHistogram(eqHist, "eq-Blue", true);
    return;
}



cv::Mat histogramEqualize(const cv::Mat& inImage, bool equalize)
{
    //Convert the image from BGR to YCrCb color space
    Mat hist_equalized_image;
    cvtColor(inImage, hist_equalized_image, COLOR_BGR2YCrCb);
    // hist_equalized_image = inImage.clone();

        // calculate histogram
    float range[] = { 0, 256 };
    const float* histRange = { range };

    bool uniform = true;
    bool accumulate = false;

    cv::Mat b_hist, g_hist, r_hist;


    ///// CLAHE
    if (clahe)
    {
        cv::Mat clahe_img;
        cvtColor(inImage, clahe_img, COLOR_BGR2Lab);
        std::vector<Mat>lab_planes;
        split(clahe_img, lab_planes);
        cv::calcHist(&lab_planes[0], 1, 0, cv::
            Mat(), b_hist, 1, &histSize,
            &histRange, uniform, accumulate);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(16, 16));
        clahe->apply(lab_planes[0], lab_planes[0]);
        merge(lab_planes, clahe_img);
    }


    //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
    std::vector<Mat> bgr_planes;
    split(hist_equalized_image, bgr_planes);


    cv::Scalar m, std;
    cv::meanStdDev(bgr_planes[0], m, std, cv::Mat());
    std::cout << "Mean:" << m[0] << ",stdDev:" << std[0] << std::endl;

    cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize,
        &histRange, uniform, accumulate);
    /*  cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize,
          &histRange, uniform, accumulate);
      cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize,
          &histRange, uniform, accumulate);*/

    drawHistogram(b_hist, "Blue");
    /*drawHistogram(g_hist, "Green");
    drawHistogram(r_hist, "Red");*/

    if (equalize)
    {
        //Equalize the histogram of only the B channel 
        equalizeHist(bgr_planes[0], bgr_planes[0]);

        //Equalize the histogram of only the G channel 
        //equalizeHist(bgr_planes[1], bgr_planes[1]);

        //Equalize the histogram of only the R channel 
        //equalizeHist(bgr_planes[2], bgr_planes[2]);

        //Merge 3 channels in the vector to form the color image in color space.
        merge(bgr_planes, hist_equalized_image);
        cv::Mat b_hist_eq, g_hist_eq, r_hist_eq;

        cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist_eq, 1, &histSize,
            &histRange, uniform, accumulate);
        /* cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist_eq, 1, &histSize,
             &histRange, uniform, accumulate);
         cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist_eq, 1, &histSize,
             &histRange, uniform, accumulate);*/

        drawHistogram(b_hist_eq, "eq-Blue", true);
        /* drawHistogram(g_hist_eq, "eq-Green",true);
         drawHistogram(r_hist_eq, "eq-Red",true);*/
        cv::Mat result;
        cvtColor(hist_equalized_image, result, COLOR_YCrCb2BGR);
        return result;
    }
    else
        return inImage;

    //Convert the histogram equalized image from YCrCb to BGR color space again
   // cvtColor(hist_equalized_image, hist_equalized_image, COLOR_YCrCb2BGR);

}

// This is a OpenCV-based implementation of conv2 in Matlab.
cv::Mat conv2(const cv::Mat& img, const cv::Mat& ikernel, ConvolutionType type)
{
    cv::Mat dest;
    cv::Mat kernel;
    cv::flip(ikernel, kernel, -1);
    cv::Mat source = img;
    if (CONVOLUTION_FULL == type)
    {
        source = cv::Mat();
        const int additionalRows = kernel.rows - 1, additionalCols = kernel.cols - 1;
        copyMakeBorder(img, source, (additionalRows + 1) / 2, additionalRows / 2, (additionalCols + 1) / 2, additionalCols / 2, cv::BORDER_CONSTANT, cv::Scalar(0));
    }
    cv::Point anchor(kernel.cols - kernel.cols / 2 - 1, kernel.rows - kernel.rows / 2 - 1);
    int borderMode = cv::BORDER_CONSTANT;
    filter2D(source, dest, img.depth(), kernel, anchor, 0, borderMode);

    if (CONVOLUTION_VALID == type)
    {
        dest = dest.colRange((kernel.cols - 1) / 2, dest.cols - kernel.cols / 2).rowRange((kernel.rows - 1) / 2, dest.rows - kernel.rows / 2);
    }
    return dest;
}

void LDR(const cv::Mat& src, cv::Mat& dst, double alpha)
{
    int R = src.rows;
    int C = src.cols;

    cv::Mat Y;
    std::vector<cv::Mat> YUV_channels;
    if (src.channels() == 1) {
        Y = src.clone();
    }
    else {
        cv::Mat YUV;
        cv::cvtColor(src, YUV, COLOR_BGR2YUV);
        cv::split(YUV, YUV_channels);
        Y = YUV_channels[0];
    }

    cv::Mat U = cv::Mat::zeros(255, 255, CV_64F);
    {
        cv::Mat tmp_k(255, 1, CV_64F);
        for (int i = 0; i < 255; i++)
            tmp_k.at<double>(i) = i + 1;

        for (int layer = 1; layer <= 255; layer++) {
            cv::Mat mi, ma;
            cv::min(tmp_k, 256 - layer, mi);
            cv::max(tmp_k - layer, 0, ma);
            cv::Mat m = mi - ma;
            m.copyTo(U.col(layer - 1));
        }
    }

    // unordered 2D histogram acquisition
    cv::Mat h2d = cv::Mat::zeros(256, 256, CV_64F);
    for (int j = 0; j < R; j++) {
        for (int i = 0; i < C; i++) {
            uchar ref = Y.at<uchar>(j, i);

            if (j != R - 1) {
                uchar trg = Y.at<uchar>(j + 1, i);
                h2d.at<double>(std::max(ref, trg), std::min(ref, trg)) += 1;
            }
            if (i != C - 1) {
                uchar trg = Y.at<uchar>(j, i + 1);
                h2d.at<double>(std::max(ref, trg), std::min(ref, trg)) += 1;
            }
        }
    }

    // Intra-Layer Optimization
    cv::Mat D = cv::Mat::zeros(255, 255, CV_64F);
    cv::Mat s = cv::Mat::zeros(255, 1, CV_64F);

    for (int layer = 1; layer <= 255; layer++) {
        cv::Mat h_l = cv::Mat::zeros(256 - layer, 1, CV_64F);

        int tmp_idx = 1;
        for (int j = 1 + layer; j <= 256; j++) {
            int i = j - layer;
            h_l.at<double>(tmp_idx - 1) = std::log(h2d.at<double>(j - 1, i - 1) + 1); // Equation (2)
            tmp_idx++;
        }

        s.at<double>(layer - 1) = cv::sum(h_l)[0];

        if (s.at<double>(layer - 1) == 0)
            continue;

        cv::Mat kernel = cv::Mat::ones(layer, 1, CV_64F);
        cv::Mat m_l = conv2(h_l, kernel, ConvolutionType::CONVOLUTION_FULL); // Equation (30)

        double mi;
        cv::minMaxLoc(m_l, &mi, 0);
        cv::Mat d_l = m_l - mi;
        d_l = d_l.mul(1.0 / U.col(layer - 1)); // Equation (33)

        if (cv::sum(d_l)[0] == 0)
            continue;

        D.col(layer - 1) = d_l / cv::sum(d_l)[0];
    }

    // Inter - Layer Aggregation
    double max_s;
    cv::minMaxLoc(s, 0, &max_s);
    cv::Mat W;
    cv::pow(s / max_s, alpha, W); // Equation (23)
    cv::Mat d = D * W; // Equation (24)

    // reconstruct transformation function
    d /= cv::sum(d)[0];
    cv::Mat tmp = cv::Mat::zeros(256, 1, CV_64F);
    for (int k = 1; k <= 255; k++) {
        tmp.at<double>(k) = tmp.at<double>(k - 1) + d.at<double>(k - 1);
    }
    tmp.convertTo(tmp, CV_8U, 255.0);

    cv::LUT(Y, tmp, Y);

    if (src.channels() == 1) {
        dst = Y.clone();
    }
    else {
        cv::merge(YUV_channels, dst);
        cv::cvtColor(dst, dst, COLOR_YUV2BGR);
    }

    return;
}
void IAGCWD(const cv::Mat& src, cv::Mat& dst, double alpha_dimmed = 0.75, double alpha_bright = 0.25, int T_t = 112, double tau_t = 0.3, double tau = 0.5)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1) {
        L = src.clone();
    }
    else {
        cv::cvtColor(src, HSV, COLOR_HSV2BGR_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    double mean_L = cv::mean(L).val[0];
    double t = (mean_L - T_t) / T_t;

    double alpha;
    bool truncated_cdf;
    if (t < -tau_t) {
        //process dimmed image
        alpha = alpha_dimmed;
        truncated_cdf = false;
    }
    else if (t > tau_t) {
        //process bright image
        alpha = alpha_bright;
        truncated_cdf = true;
        L = 255 - L;
    }
    else {
        //do nothing
        dst = src.clone();
        return;
    }

    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat hist;
    calcHist(&L, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    double total_pixels_inv = 1.0 / total_pixels;
    cv::Mat PDF = cv::Mat::zeros(256, 1, CV_64F);
    for (int i = 0; i < 256; i++) {
        PDF.at<double>(i) = hist.at<float>(i) * total_pixels_inv;
    }

    double pdf_min, pdf_max;
    cv::minMaxLoc(PDF, &pdf_min, &pdf_max);
    cv::Mat PDF_w = PDF.clone();
    for (int i = 0; i < 256; i++) {
        PDF_w.at<double>(i) = pdf_max * std::pow((PDF_w.at<double>(i) - pdf_min) / (pdf_max - pdf_min), alpha);
    }

    cv::Mat CDF_w = PDF_w.clone();
    double culsum = 0;
    for (int i = 0; i < 256; i++) {
        culsum += PDF_w.at<double>(i);
        CDF_w.at<double>(i) = culsum;
    }
    CDF_w /= culsum;

    cv::Mat inverse_CDF_w = 1.0 - CDF_w;
    if (truncated_cdf) {
        inverse_CDF_w = cv::max(tau, inverse_CDF_w);
    }

    std::vector<uchar> table(256, 0);
    for (int i = 1; i < 256; i++) {
        table[i] = cv::saturate_cast<uchar>(255.0 * std::pow(i / 255.0, inverse_CDF_w.at<double>(i)));
    }

    cv::LUT(L, table, L);

    if (t > tau_t) {
        L = 255 - L;
    }

    if (channels == 1) {
        dst = L.clone();
    }
    else {
        cv::merge(HSV_channels, dst);
        cv::cvtColor(dst, dst, COLOR_HSV2BGR_FULL);
    }

    return;
}
void AGCWD(const cv::Mat& src, cv::Mat& dst, double alpha)
{
    int rows = src.rows;
    int cols = src.cols;
    int channels = src.channels();
    int total_pixels = rows * cols;

    cv::Mat L;
    cv::Mat HSV;
    std::vector<cv::Mat> HSV_channels;
    if (channels == 1) {
        L = src.clone();
    }
    else {
        cv::cvtColor(src, HSV, COLOR_BGR2HSV_FULL);
        cv::split(HSV, HSV_channels);
        L = HSV_channels[2];
    }

    int histsize = 256;
    float range[] = { 0,256 };
    const float* histRanges = { range };
    int bins = 256;
    cv::Mat hist;
    calcHist(&L, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

    double total_pixels_inv = 1.0 / total_pixels;
    cv::Mat PDF = cv::Mat::zeros(256, 1, CV_64F);
    for (int i = 0; i < 256; i++) {
        PDF.at<double>(i) = hist.at<float>(i) * total_pixels_inv;
    }

    double pdf_min, pdf_max;
    cv::minMaxLoc(PDF, &pdf_min, &pdf_max);
    cv::Mat PDF_w = PDF.clone();
    for (int i = 0; i < 256; i++) {
        PDF_w.at<double>(i) = pdf_max * std::pow((PDF_w.at<double>(i) - pdf_min) / (pdf_max - pdf_min), alpha);
    }

    cv::Mat CDF_w = PDF_w.clone();
    double culsum = 0;
    for (int i = 0; i < 256; i++) {
        culsum += PDF_w.at<double>(i);
        CDF_w.at<double>(i) = culsum;
    }
    CDF_w /= culsum;

    std::vector<uchar> table(256, 0);
    for (int i = 1; i < 256; i++) {
        table[i] = cv::saturate_cast<uchar>(255.0 * std::pow(i / 255.0, 1 - CDF_w.at<double>(i)));
    }

    cv::LUT(L, table, L);

    if (channels == 1) {
        dst = L.clone();
    }
    else {
        cv::merge(HSV_channels, dst);
        cv::cvtColor(dst, dst, COLOR_HSV2BGR_FULL);
    }

    cv::Mat eqHist;
    calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
    //drawhistogram
    drawHistogram(eqHist, "eq-Blue", true);
    return;
}


void main()
{
    // Histogram Equalization
    std::string inImage = "D:/Nausicaa_Data/Data_23092022_2/Images/5000/1663929753113.jpeg";
    bool equalize = true;
    high_resolution_clock::time_point start_time, end_time;

    //load image
    cv::Mat srcFrame, dstFrame;
    srcFrame = cv::imread(inImage);
    cv::namedWindow(cv::String(std::string("Original - Image")), cv::WINDOW_KEEPRATIO);
    cv::imshow(cv::String(std::string("Original - Image")), srcFrame);

    //histogram equalize
    //dstFrame=histogramEqualize(srcFrame, equalize);

    //GCEHistMod(srcFrame, dstFrame);
    
    
    //AGCIE(srcFrame, dstFrame);

    // JHE(srcFrame, dstFrame);

     start_time = high_resolution_clock::now();
     WTHE(srcFrame, dstFrame);
     end_time = high_resolution_clock::now();
     OutputTime("WTHE", start_time, end_time);

    //LDR(srcFrame, dstFrame, 4.0);

   /*  start_time = high_resolution_clock::now();
     AGCWD(srcFrame, dstFrame,0.5);
     end_time = high_resolution_clock::now();
     OutputTime("ADIE", start_time, end_time);*/

    /* start_time = high_resolution_clock::now();
     adaptiveImageEnhancement(srcFrame, dstFrame);
     end_time = high_resolution_clock::now();
     OutputTime("ADIE", start_time, end_time);*/
   
     //IAGCWD(srcFrame, dstFrame);

    //cv::imwrite("D:/Nausicaa_Data/Histogram/agcwd.jpg", dstFrame);

    cv::namedWindow(cv::String(std::string("Equalized - Image")), cv::WINDOW_KEEPRATIO);
    cv::imshow(cv::String(std::string("Equalized - Image")), dstFrame);
    cv::waitKey(0);


#ifdef RECTIFY
    struct ocam_model o;
    get_ocam_model(&o, fishEyeCalibrationFile.c_str());
    cv::Size imgSize(cv::Size(1948, 1096));
    MeiCalibration mei;
    readMeiCalibration(meiCalibFile, mei);
    for (const auto& dirpath : fs::directory_iterator(inputFolderPath + "Images/"))
    {
        std::cout << dirpath.path() << std::endl;
        auto imgFolder = dirpath.path();
        auto parentFolder = imgFolder.parent_path();
        std::string DataFolder = parentFolder.parent_path().string().substr(parentFolder.parent_path().string().find_last_of("/") + 1);;
        std::string camID = imgFolder.string().substr(imgFolder.string().find_last_of("/") + 1);
        std::string outFolder = outputRootPath + DataFolder + "/" + "Images/" + camID;
        if (!fs::exists(outFolder))
            fs::create_directories(outFolder);
        for (const auto& entry : fs::directory_iterator(imgFolder))
        {
            //std::cout << entry.path() << std::endl;
            std::string ff = entry.path().string();
            //std::cout << ff << std::endl;
            std::string base_filename = ff.substr(ff.find_last_of("/\\") + 1);

            std::string outFileName = outFolder + "/" + base_filename;

            cv::Mat map1, map2;
            cv::Mat inFrame, dstFrame;
            if (base_filename != "timestamps.txt")
            {

                inFrame = cv::imread(ff);


                cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F);// computeOpenCVMatrixFromScaraMuzza(o);
               //d = getDistCoeffiecient(o);
                cv::Mat distCoeffs = cv::Mat(1, 4, CV_32F);
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        cameraMatrix.at<float>(i, j) = mei.meiCameraMatrix(i, j);
                    }


                }

                for (int i = 0; i < 4; ++i)
                {
                    distCoeffs.at<float>(i) = mei.distortionParameter[i];
                }
                // std::cout << cameraMatrix << std::endl;
                //std::cout << "distortion coefficients:" << distCoeffs << std::endl;
                cv::Mat new_camera_matrix = cv::Mat(cv::Matx33f(o.width / 4.0, 0, cameraMatrix.at<float>(0, 2),
                    0, o.width / 4.0, cameraMatrix.at<float>(1, 2),
                    0, 0, 1));

                //std::cout << "Camera Intrinsics for rectification:" << new_camera_matrix << std::endl;
                cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
                cv::omnidir::initUndistortRectifyMap(cameraMatrix, distCoeffs, mei.xiFactor, R, new_camera_matrix, imgSize,
                    CV_32F, map1, map2, cv::omnidir::RECTIFY_PERSPECTIVE);
                cv::remap(inFrame, dstFrame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

                cv::imwrite(outFileName, dstFrame);
                std::cout << "Done rectifying:" << base_filename << std::endl;
            }
            else
                continue;

        }
    }
#endif // RECTIFY


    //cv::Mat src1 = cv::imread("rectified_streamed_output_04052022.jpg"); //rectified_streamed_output
    //std::vector<cv::Point2i> p2, pt_ex, pt_ex2;

    //// temporary hard-coded image pixel values -gt
    //p2.push_back(cv::Point2i(793, 344));
    //p2.push_back(cv::Point2i(1630, 202));
    //p2.push_back(cv::Point2i(633, 380));
    //p2.push_back(cv::Point2i(1380, 30));
    ///*p2.push_back(cv::Point2i(433, 1028));
    //p2.push_back(cv::Point2i(816, 1029));*/

    //cv::Scalar color(0, 0, 255); // BGR
    //DrawMarkersImage(src1, p2, 0, color);

    //// image pixel values - only PnP
    //pt_ex.push_back(cv::Point2i(793, 343));
    //pt_ex.push_back(cv::Point2i(1629, 205));
    //pt_ex.push_back(cv::Point2i(634, 381));
    //pt_ex.push_back(cv::Point2i(1379, 26));
    ///*pt_ex.push_back(cv::Point2i(383, 978));
    //pt_ex.push_back(cv::Point2i(888, 918));*/

    //cv::Scalar color_1(0, 255, 255);
    //DrawMarkersImage(src1, pt_ex, 5, color_1);
    //cv::imwrite("D:/CamImages/image_with_marker_04052022_f4.jpg", src1);

    ////////////////////////

    //double theta = 2.0 * (atan2(1948 / 2.0, 475.04) * 180 / 3.141592);
    //Eigen::Vector2d img_size(o.width, o.height);
    //Eigen::Matrix3f K_out;
    //std::array<float, 5> D_out;
    //float xi_out;
    //Eigen::Vector2d principal_point{ o.yc, o.xc };
    //std::vector<double>  invpoly;
    //invpoly.assign(o.invpol, o.invpol + o.length_invpol);
    //calib_converter::convertOcam2Mei(invpoly, principal_point, img_size, o.c, o.d, o.e, K_out, D_out, xi_out);
    //cv::Mat dst, map1, map2, newMat;
    //cv::Size newSize;
    //cv::Mat d = cv::Mat(1, 4, CV_32F);
    //cv::Mat K(3, 3, cv::DataType<float>::type);
    ////K = computeOpenCVMatrixFromScaraMuzza(o);
    ////d = getDistCoeffiecient(o);
    //for (int i = 0; i < 3; ++i)
    //{
    //    for (int j = 0; j < 3; ++j)
    //    {
    //        K.at<float>(i, j) = K_out(i, j);
    //    }


    //}

    //for (int i = 0; i < 4; ++i)
    //{
    //    d.at<float>(i) = D_out[i];
    //}

    //std::cout << K << std::endl;
    //std::cout << "distortion coefficients:" << d << std::endl;
    //cv::Size s = src1.size();
    //cv::Mat Knew = cv::Mat(cv::Matx33f(-o.pol[0], 0, K.at<float>(0, 2),
    //    0, -o.pol[0], K.at<float>(1, 2),
    //    0, 0, 1));
    //std::cout << Knew << std::endl;
    //cv::Mat Ro = cv::Mat::eye(3, 3, CV_32F);
    ///*   cv::omnidir::undistortImage(src1, dst, K, d, xi_out, cv::omnidir::RECTIFY_PERSPECTIVE, Knew, s,Ro);
    //   cv::Mat R = cv::Mat::eye(3, 3, CV_32F);*/
    //cv::omnidir::initUndistortRectifyMap(K, d, xi_out, Ro, Knew, s,
    //    CV_32F, map1, map2, cv::omnidir::RECTIFY_PERSPECTIVE);
    //cv::remap(src1, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    //// cv::fisheye::undistortImage(src1, dst, K, d, K, s);
    //cv::imwrite("D:/CamImages/undistorted_perspective_test_conv_new_updated_test.jpg", dst);

    //// // convert ocam to mei model

    //// //cv::Mat src1 = cv::imread("D:/CamImages/25032022_new0.jpg");
    /////* cv::namedWindow("Original fisheye camera image", 1);
    //// cv::imshow("Original fisheye camera image", src1);*/



    //cv::Mat cameraMatrix = computeOpenCVMatrixFromScaraMuzza(o);

    //cv::Size imageSize(cv::Size(1948, 1096));
    //// cv::Mat map1, map2;
    //cv::Mat distCoeffs = getDistCoeffiecient(o);

    /////*cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
    ////std::cout << "distCoeffs UND : " << distCoeffs << std::endl;*/

    ////cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, imageSize, CV_16SC2, map1, map2);

    ////cv::remap(src1, dst, map1, map2, cv::INTER_LINEAR, cv::WARP_FILL_OUTLIERS, cv::Scalar(0, 0, 0));

    //std::vector<cv::Point3f>points;
    //// Creating the Rotation Matrix
    //cv::Mat R(3, 3, cv::DataType<float>::type);

    //R.at<float>(0, 0) = 1;
    //R.at<float>(1, 0) = 0;
    //R.at<float>(2, 0) = 0;

    //R.at<float>(0, 1) = 0;
    //R.at<float>(1, 1) = 1;
    //R.at<float>(2, 1) = 0;

    //R.at<float>(0, 2) = 0;
    //R.at<float>(1, 2) = 0;
    //R.at<float>(2, 2) = 1;
    //cv::Mat r, t;
    //r = cv::Mat(3, 1, CV_32F);
    //t = cv::Mat(3, 1, CV_32F);
    //cv::Rodrigues(R, r);
    //t.at<float>(0) = 0.0f;
    //t.at<float>(1) = 0.0f;
    //t.at<float>(2) = 0.0f;
    //points.push_back(cv::Point3f(1.0, 1.0, 1.0));
    //std::vector<cv::Point2f>point2D;
    //cv::fisheye::projectPoints(points, point2D, r, t, cameraMatrix, distCoeffs);
    //int point_cv1 = std::floor(point2D[0].x);
    //int point_cv2 = std::floor(point2D[0].y);
    //std::cout << " Projected to " << point2D[0] << std::endl;

    //double Point3D[3] = { 1.0,1.0,1.0 };       // a sample 3D point
    //double Point2D[2];                              // the image point in pixel coordinates  
    //world2cam(Point2D, Point3D, &o);
    //int point_sc1 = std::floor(Point2D[0]);
    //int point_sc2 = std::floor(Point2D[1]);
    //printf("m_row= %2.4f, m_col=%2.4f\n", Point2D[0], Point2D[1]);
    //int markerSize = 30; int thickness = 2;
    ///* cv::drawMarker(src1, cv::Point(point_cv1, point_cv2), cv::Vec3b(0, 0, 200), cv::MARKER_CROSS,markerSize,thickness );
    // cv::drawMarker(src1, cv::Point(point_sc1, point_sc2), cv::Vec3b(255, 0, 0), cv::MARKER_CROSS, markerSize, thickness);
    // cv::imwrite("D:/CamImages/undistorted_perspective_test_25032022_marker.jpg", src1); */




    // /* cv::namedWindow("Undistorted Perspective Image", 1);
    //  cv::imshow("Undistorted Perspective Image", dst);
    //  cv::imwrite("D:/CamImages/undistorted_perspective_test_25032022_new.jpg", dst);*/

    //cv::waitKey(0);


}
