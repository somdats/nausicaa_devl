

#include"..\headers\imgproc.h"

using namespace cv;

void imgprocess::drawImageHistogram(cv::Mat& b_hist, std::string channel, bool eq) {

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
void imgprocess::AGCWD(const cv::Mat& src, cv::Mat& dst, double alpha, bool drawHis) {

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
    if (drawHis)
    {
        cv::Mat eqHist;
        calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
        //drawhistogram
        drawImageHistogram(eqHist, "eq-Blue", true);
    }
    return;

}
void imgprocess::WTHE(const cv::Mat& src, cv::Mat& dst, float ru, float vu, bool drawHis) {

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
    if (drawHis)
    {
        //histogram after processing
        cv::Mat eqHist;
        calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
        //drawhistogram
        drawImageHistogram(eqHist, "eq-Blue", true);
    }
    return;

}
void imgprocess::JHE(const cv::Mat& src, cv::Mat& dst, bool drawHis) {

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
    if (drawHis)
    {
        int histsize = 256;
        float range[] = { 0,256 };
        const float* histRanges = { range };
        int bins = 256;
        cv::Mat eqHist;
        calcHist(&L, 1, 0, cv::Mat(), eqHist, 1, &histsize, &histRanges, true, false);
        //drawhistogram
        drawImageHistogram(eqHist, "eq-Blue", true);
    }
    return;
}