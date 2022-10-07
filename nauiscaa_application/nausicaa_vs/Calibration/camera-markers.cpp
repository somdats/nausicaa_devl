
#include"camera-markers.h"

using namespace cv;
using namespace std;

//const int max_lowThreshold = 100;
//const int ratio = 3;
//const int kernel_size = 3;
//
////Mat dst, detected_edges;
//int lowThreshold = 45;
//
//const char* window_name = "Edge Map";
//std::vector<cv::Vec2f> lines;
//cv::Scalar meanVal, stdDev;
RNG rng(12345);
int margin = 2;

using namespace camMarkers;


cv::Mat cameraMarkers::createArucoMarkers(int id, int sizePixel, int borderSize, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName) {

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictName);
    cv::Mat ArucoImage;
    cv::aruco::drawMarker(dictionary, id, sizePixel, ArucoImage, borderSize);
    return ArucoImage;
}

std::vector<cv::Mat> cameraMarkers::createMultipleMarkers(int numMarkers, int startId, int size, int borderSize, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName) {

    std::vector<cv::Mat>multiMarkers;
    int arucoCounter = 0;
    int count = 0;
    while (count < numMarkers)
    {
        cv::Mat ArucoImage;
        ArucoImage = createArucoMarkers(startId, size, borderSize, dictName);
        multiMarkers.emplace_back(ArucoImage);
        arucoCounter++;
        count++;
        startId = startId + arucoCounter;
    }
    return multiMarkers;

}

void cameraMarkers::detectMarkers(cv::Mat inputMarkers, std::vector<std::vector<cv::Point2f>>& markerCorners, std::vector<int>& markerIds,
    std::vector<std::vector<cv::Point2f>>& rejectedCandidates, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName) {

    cv::Mat in_gray;
    if (inputMarkers.channels() >= 3)
        cv::cvtColor(inputMarkers, in_gray, COLOR_BGR2GRAY);
    else
        in_gray = inputMarkers.clone();
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictName);
    cv::aruco::detectMarkers(in_gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


}



// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool cameraMarkers::intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
    Point2f& r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    r = o1 + d1 * t1;
    return true;
}


void cameraMarkers::ConvertMatToPoint(cv::Mat img, std::vector<cv::Point>& points)
{
    for (int x = 0; x < img.cols; x++)
        for (int y = 0; y < img.rows; y++)
            points.push_back(cv::Point(x, y));
}


void cameraMarkers::ContourPtsToCVPts(const std::vector<cv::Point>& cnts, vector<Point2f>& pts)
{
    size_t count = cnts.size();
    std::transform(cnts.begin(), cnts.end(),
        std::back_inserter(pts),
        [](const Point& p) { return (Point2f)p; });
}

void cameraMarkers::drawLines(const cv::Mat& cannyEdgeImage, const std::vector<cv::Point2f>& cvLines) {

    cv::Mat edge_lines = cannyEdgeImage.clone();
    for (const auto& line : cvLines) {
        float rho = line.x;
        float theta = line.y;
        float a = cos(theta);
        float b = sin(theta);
        float x0 = a * rho;
        float y0 = b * rho;
        float x1 = int(x0 + 1000 * (-b));
        float y1 = int(y0 + 1000 * (a));
        float x2 = int(x0 - 1000 * (-b));
        float y2 = int(y0 - 1000 * (a));
        cv::line(edge_lines, cv::Point(x1, y1), cv::Point(x2, y2), Scalar(0, 0, 255),
            2, LINE_8);
    }
    namedWindow(cv::String("line-Image"), WINDOW_AUTOSIZE);
    imshow(cv::String("line-Image"), edge_lines);
}


void cameraMarkers::CannyThreshold(const cv::Mat& srcImage)
{
    cv::Mat src_gray;
    cvtColor(srcImage, src_gray, COLOR_BGR2GRAY);
    blur(src_gray, detectedEdges, Size(3, 3));
    //Canny(detected_edges, detected_edges, meanVal(0)-  2.0 *stdDev(0), meanVal(0) + 2.0 * stdDev(0), kernel_size);
    Canny(detectedEdges, detectedEdges, lowerThres, lowerThres * 3.0, kernel_size);
    /*  cv::Mat dst;
      srcImage.copyTo(dst, detectedEdges);
      imshow(window_name, detectedEdges);*/

}

// returns a set of lines represented by two extremas ( Po - t*dir, Po + t*dir) with high no. of contour pts within a range (lt,ht)
// returns all fitted-line w detected from a binary-edge image 
std::vector<std::pair<pt2f, pt2f>> cameraMarkers::detectContours(const cv::Mat& inImage, int lt, int ht, Scalar linecolor) {

    contours.clear();
    fittedLines.clear();
    findContours(detectedEdges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    int count = 0;
    std::vector<std::pair<pt2f, pt2f>>lines_pts;
    float t = (float)(inImage.cols + inImage.rows);
    for (int i = 0; i < contours.size(); i++)
    {
        // contour-pts to cvpts
        std::vector<cv::Point2f> pts;
        ContourPtsToCVPts(contours[i], pts);
        Mat fitLine;
        cv::fitLine(pts, fitLine, DIST_L1, 0, 0.01, 0.01);
        fittedLines.emplace_back(fitLine);
        if (contours[i].size() < ht && contours[i].size() > lt) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            //drawContours(src, contours, i, color, 2, 8, hierarchy, 0);



            float vx = fitLine.at<float>(0, 0);
            float vy = fitLine.at<float>(1, 0);
            float x0 = fitLine.at<float>(2, 0);
            float y0 = fitLine.at<float>(3, 0);

            // normalize dir.vec
            float d = sqrt((float)vx * vx + (float)vy * vy);
            /* vx /= d;
             vy /= d;*/

            cv::Point O = cv::Point(int(x0 - t * vx), int(y0 - t * vy));
            cv::Point P = cv::Point(int(x0 + t * vx), int(y0 + t * vy));
            cv::line(inImage, O, P, linecolor, 2, LINE_AA, 0);
            if (contours[i].size() < ht && contours[i].size() > lt)
            {
                pt2f sPt = cv::Point2f(x0 - t * vx, y0 - t * vy);
                pt2f ePt = cv::Point2f(x0 + t * vx, y0 + t * vy);
                lines_pts.emplace_back(std::make_pair(sPt, ePt));

            }
            /* namedWindow(cv::String("line-Image"), WINDOW_AUTOSIZE);
             imshow(cv::String("line-Image"), inImage);*/
            count++;
        }
    }
    return lines_pts;
}

vector<Point>  cameraMarkers::getContours(int idx)
{
    vector<Point>singleContour;
    if (idx < contours.size())
        singleContour = contours[idx];
    else
        singleContour;

    return singleContour;

}
vector<vector<Point> >cameraMarkers::getContours()
{
    return contours;
}

cv::Mat cameraMarkers::getDetectedEdges() {
    return detectedEdges;
}

// returns a vector of cv::Mat, each Mat of size(4,1) representing a 2d ray-line with ( xo,yo) as origin pt. and dir vector(vx,vy)
std::vector<cv::Mat> cameraMarkers::getAllFittedLines() {
    return fittedLines;
}

bool cameraMarkers::MarkerDetected(const cv::Mat& inImage, int markerID1, int markerID2, int markerID3, cv::aruco::PREDEFINED_DICTIONARY_NAME dictName) {

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    detectMarkers(inImage, markerCorners, markerIds, rejectedCandidates, dictName);
    bool allMarkers = false;
    if (markerIds.size() < 3)
    {
        std::cout << "Warning:Not all Marker detected" << std::endl;
        if (std::find(markerIds.begin(), markerIds.end(), markerID1) == markerIds.end())
            std::cout << "Marker with ID not detected:" << markerID1 << std::endl;

        if (std::find(markerIds.begin(), markerIds.end(), markerID2) == markerIds.end())
            std::cout << "Marker with ID not detected:" << markerID3 << std::endl;

        if (std::find(markerIds.begin(), markerIds.end(), markerID3) == markerIds.end())
            std::cout << "Marker with ID not detected:" << markerID3 << std::endl;
    }
    else
        allMarkers = true;
    return allMarkers;



}

bool cameraMarkers::detectMarker(const cv::Mat& inImage, cv::Point2f& marker) {
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
 
    this->detectMarkers(inImage, markerCorners, markerIds, rejectedCandidates);

    std::vector<cv::Point2f> pts[4];
    for (int i = 0; i < markerCorners.size(); ++i)
        if (markerIds[i] < 4)
            pts[markerIds[i]].push_back(markerCorners[i][0]);

    std::vector<cv::Point2f> candidates;

    if (!pts[0].empty()) {
        marker = pts[0][0];
        return true;
    }

    cv::Point2f p;
    for (int i = 1; i <= 3; ++i)
        if (pts[i].size() == 2)
            for (int j = 1; j <= 3; ++j)
                if (i != j)
                    if (pts[j].size() == 2)
                        if (this->intersection(pts[i][0], pts[i][1], pts[j][0], pts[j][1], p))
                            candidates.push_back(p);

    if (candidates.empty())
        return false;

    p = cv::Point2f(0.f, 0.f);
    for (int i = 0; i < candidates.size(); ++i)
        p += candidates[i];

    p = p*(1.f/candidates.size());
    return true;

}
