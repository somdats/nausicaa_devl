#include <vcg/space/distance3.h>
#include <vcg/space/point_matching.h>
#include "correspondences_detector.h"
#include "lidar_render.h"
#include "camera_reader.h"


// dart throwing Poisson
extern std::vector<::Camera> cameras;

template <class VTYPE>
void  PoissonDistribution(std::vector<VTYPE> corrs, unsigned int n, float radius,  std::vector<VTYPE> &res ) {
	unsigned int n_tries = 0;
	while (res.size() < n && n_tries < corrs.size()) {
		unsigned int i = rand() / float(RAND_MAX) * (corrs.size() - 1);
		int y;
		for ( y = 0; y < res.size(); ++y)
				if ((corrs[i].second - res[y].second).Norm() < radius)
					break;
		if (y == res.size())
			res.push_back(corrs[i]);
		n_tries++;
	}
} 




bool CorrespondencesDetector::detect3D(int lidarID, vcg::Point3f& p) { 
	md.points.clear();
	 for (unsigned int i = 0; i < lidars[lidarID].lidar.latest_frame.x.size();++i) {		 
		 vcg::Point3f p = lidars[lidarID].transfLidar * vcg::Point3f(lidars[lidarID].lidar.latest_frame.x[i], lidars[lidarID].lidar.latest_frame.y[i], lidars[lidarID].lidar.latest_frame.z[i]);
		 if (vcg::Distance<float>(p, currentP3D[lidarID]) < 0.5) //   marker size
			 md.points.push_back(p);
		}
//	 if (md.detect_corner(p, p0[lidarID], p1[lidarID], p2[lidarID])) {
	 if ( md.points.size()>20 && md.detect_quad_center(p)) {
		 correspondences3D3Dbbox.Add(p);
		 return true;
	 }

	return false;
}



bool CorrespondencesDetector::detect2D(int camID, vcg::Point2f& p) { 
	cv::Point2f p_cv;
	if (md2d.detectMarker(cameras[camID].dst, p_cv)) {
		p = vcg::Point2f(p_cv.x, p_cv.y);
		cv::circle(cameras[camID].dst, p_cv, 30, cv::Scalar(0, 0, 255), 10);
		return true;
	}
	return false; 
}

void CorrespondencesDetector::drawAll2DCorrs(int camID) {
	vcg::Point2f  p;
	for (unsigned int i = 0; i < correspondences3D2D[camID].size();++i) {
		p = correspondences3D2D[camID][i].second;
		cv::circle(cameras[camID].dst, cv::Point2f(p[0],p[1]), 30, cv::Scalar(0, 0, 255), 10);
	}
}

void CorrespondencesDetector::draw_debug() {
	for (unsigned int i = 0; i < cameras.size(); ++i) 
		drawAll2DCorrs(i);
}


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

void CorrespondencesDetector::detect(){ 
	lockAll();

	 
	bool detected3D[2] = { false,false };
 	for (unsigned int i = 0; i < 2; ++i)
 		detected3D[i] = detect3D(i, currentP3D[i]);

	if (detected3D[0] && detected3D[1])
		correspondences3D3D.push_back(Correspondence3D3D(std::pair(currentP3D[0], currentP3D[1])));

	if (detected3D[0] || detected3D[1])
	{
		std::vector<bool> detected2D(nCams, false);
		std::vector < vcg::Point2f> p2D(nCams);

		if (!cameras.empty())
			for (unsigned int i = 0; i < nCams; ++i)
				if (cameras[i].reading)
					if (detect2D(i, p2D[i]))
						for (int iL = 0; iL < 2; ++iL) if
							(detected3D[0])
								correspondences3D2D[i].push_back(Correspondence3D2D(std::pair(currentP3D[0], p2D[i])));

	}


	unlockAll();
}


bool CorrespondencesDetector::alignCamera(int camID){ 

	int n = 4;
	std::vector<Correspondence3D2D> corr;
	PoissonDistribution(this->correspondences3D2D[camID], n, 150, corr);
	if (corr.size() < n)
		return false;

	cameras[camID].p2i.clear();
	for (unsigned int i = 0; i < n; ++i)
		cameras[camID].p2i.push_back(cv::Point2f(corr[i].second.X(), corr[i].second.Y()));

	cameras[camID].calibrated = cameras[camID].SolvePnP_new(corr);
	return true; 
}

void CorrespondencesDetector::alignCameras() {
	for (unsigned int i = 0; i < cameras.size(); ++i)
		alignCamera(i);
}


bool CorrespondencesDetector::alignLidars(){ 
	std::vector<vcg::Point3f> pFix;
	std::vector<vcg::Point3f> pMov;

	std::vector<  Correspondence3D3D   > res;
	PoissonDistribution(correspondences3D3D, 10, 1, res);
	if (res.size() < 4 || correspondences3D3Dbbox.Diag() < 3.0)
		return false;

	for (unsigned int i = 0; i < res.size();++i) {
		pFix.push_back(res[i].first);
		pMov.push_back(res[i].second);
	}

	lidars[1].transfLidar = vcg::ComputeLeastSquaresRigidMotion(pFix, pMov);

	return true; }


void CorrespondencesDetector::save_correspondences(const char* filename) {
	FILE* fo = fopen(filename, "wb");
	int n = correspondences3D3D.size();
	fwrite(& n, 4, 1, fo);
	fwrite(&*correspondences3D3D.begin(), sizeof(Correspondence3D3D), correspondences3D3D.size(), fo);
	n = correspondences3D2D.size();
	fwrite(&n, 4, 1, fo);
	fwrite(&*correspondences3D3D.begin(), sizeof(Correspondence3D2D), correspondences3D2D.size(), fo);
	fclose(fo);
}

void CorrespondencesDetector::load_correspondences(const char* filename) {
	FILE* fo = fopen(filename, "rb");
	int n;
	fread(& n, 4, 1, fo);
	correspondences3D3D.resize(n);
	fread(&* correspondences3D3D.begin(), sizeof(Correspondence3D3D), correspondences3D3D.size(), fo);
	n = correspondences3D2D.size();
	fread(&n, 4, 1, fo);
	fread(&*correspondences3D3D.begin(), sizeof(Correspondence3D2D), correspondences3D2D.size(), fo);
	fclose(fo);
}