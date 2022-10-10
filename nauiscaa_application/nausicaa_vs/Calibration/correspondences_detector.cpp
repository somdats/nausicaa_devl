#include "correspondences_detector.h"
#include "lidar_render.h"


bool CorrespondencesDetector::detect3D(int lidarID, vcg::Point3f& p) { return true; }

bool CorrespondencesDetector::detect2D(int camID, vcg::Point2f& p) { return true; }

bool CorrespondencesDetector::lockAll() {
	// locklidar
	lidars[0].lidar.latest_frame_mutex.lock();

	return true;
}
bool CorrespondencesDetector::unlockAll(){ return true; }

bool CorrespondencesDetector::detect(){ return true; }


bool CorrespondencesDetector::alignCamera(int camID){ return true; }
bool CorrespondencesDetector::alignLidars(vcg::Matrix44f& transf){ return true; }
