#include "correspondences_detector.h"
#include "lidar_render.h"
#include <vcg/space/distance3.h>


bool CorrespondencesDetector::detect3D(int lidarID, vcg::Point3f& p) { 
	md.points.clear();
	 for (unsigned int i = 0; i < lidars[lidarID].lidar.latest_frame.x.size();++i) {		 
		 vcg::Point3f p = lidars[lidarID].transfLidar * vcg::Point3f(lidars[lidarID].lidar.latest_frame.x[i], lidars[i].lidar.latest_frame.y[i], lidars[i].lidar.latest_frame.z[i]);
		 if (vcg::Distance<float>(p, currentP3D[lidarID]) < 0.4)
			 md.points.push_back(p);
		}
	if (md.detect_corner(p, p0[lidarID], p1[lidarID], p2[lidarID]))
		return true;

	return false;
}



bool CorrespondencesDetector::detect2D(int camID, vcg::Point2f& p) { return true; }

bool CorrespondencesDetector::lockAll() {
	// locklidar
	lidars[0].lidar.latest_frame_mutex.lock();
	lidars[1].lidar.latest_frame_mutex.lock();

	for (unsigned int i = 0; i < cameras.size(); ++i)
		cameras[i].latest_frame_mutex.lock();


	return true;
}
bool CorrespondencesDetector::unlockAll(){ 
	lidars[0].lidar.latest_frame_mutex.unlock();
	lidars[1].lidar.latest_frame_mutex.unlock();
	
	for (unsigned int i = 0; i < cameras.size(); ++i)
		cameras[i].latest_frame_mutex.unlock();

	return true; 
}

bool CorrespondencesDetector::detect(){ 
	lockAll();

	vcg::Point3f p3D[2];
	bool detected3D[2] = { false,false };

	for (unsigned int i = 0; i < 2; ++i)
		detected3D[i] = detect3D(i, p3D[i]);

	// if detected[0] || detected[1] // if no 3d point wasdetected skip the detection on images

	std::vector<bool> detected2D(nCams,false);
	std::vector < vcg::Point2f> p2D(nCams);

	for (unsigned int i = 0; i < nCams; ++i)
		detected2D[i] = detect2D(i, p2D[i]);


	unlockAll();
	return true; 
}


bool CorrespondencesDetector::alignCamera(int camID){ return true; }
bool CorrespondencesDetector::alignLidars(vcg::Matrix44f& transf){ return true; }
