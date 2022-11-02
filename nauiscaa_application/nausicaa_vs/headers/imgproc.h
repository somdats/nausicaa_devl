#pragma once



#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <filesystem>

#include <chrono>


using std::chrono::high_resolution_clock;
const int histSize = 256;


namespace imgprocess {



    enum ConvolutionType {
        /* Return the full convolution, including border */
        CONVOLUTION_FULL,
        /* Return only the part that corresponds to the original image */
        CONVOLUTION_SAME,
        /* Return only the submatrix containing elements that were not influenced by the border */
        CONVOLUTION_VALID
    };

    void drawImageHistogram(cv::Mat& b_hist, std::string channel, bool eq = false);

    void AGCWD(const cv::Mat& src, cv::Mat& dst, double alpha, bool drawHis = false);

    void WTHE(const cv::Mat& src, cv::Mat& dst, float ru = 0.9, float vu = 0.9, bool drawHis = false);

    void JHE(const cv::Mat& src, cv::Mat& dst, bool drawHis = false);
}