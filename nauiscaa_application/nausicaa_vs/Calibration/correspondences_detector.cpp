#include <vcg/space/distance3.h>
#include <vcg/space/point_matching.h>
#include <vcg/space/fitting3.h>
#include "correspondences_detector.h"
#include "lidar_render.h"
#include "camera_reader.h"


// dart throwing Poisson
extern std::vector<::Camera> cameras;

template <class VTYPE>
void  PoissonDistribution(std::vector<VTYPE> corrs, unsigned int n, float min_radius,  std::vector<VTYPE> &res ) {
	
	float radius = cameras[0].dst.rows / 3;
	do {
		unsigned int n_tries = 0;
		res.clear();
		while (res.size() < n && n_tries < corrs.size() * corrs.size()) {
			unsigned int i = rand() / float(RAND_MAX) * (corrs.size() - 1);
			int y;
			for (y = 0; y < res.size(); ++y)
				if ((corrs[i].second - res[y].second).Norm() < radius)
					break;
			if (y == res.size())
				res.push_back(corrs[i]);
			n_tries++;
		}
		radius *= 0.9;
	} while (radius > min_radius && res.size() < n);
} 


bool  CorrespondencesDetector::region_selection(int lidarID) {
	md.points.clear();
		switch (0 /* trackingState[lidarID] */ ) {
		case 0:  
			for (unsigned int i = 0; i < lidars[lidarID].lidar.latest_frame.x.size();++i) {
				vcg::Point3f p = /*lidars[lidarID].transfLidar **/ vcg::Point3f(lidars[lidarID].lidar.latest_frame.x[i], lidars[lidarID].lidar.latest_frame.y[i], lidars[lidarID].lidar.latest_frame.z[i]);
				if (vcg::Distance<float>(p, currentP3D[lidarID]) < 0.9) //   marker size
					md.points.push_back(p);
			}
			break;
		case 1:  
			for (unsigned int i = 0; i < lidars[lidarID].lidar.latest_frame.x.size();++i) {
				vcg::Point3f p = /*lidars[lidarID].transfLidar **/ vcg::Point3f(lidars[lidarID].lidar.latest_frame.x[i], lidars[lidarID].lidar.latest_frame.y[i], lidars[lidarID].lidar.latest_frame.z[i]);
				float planedist = fabs((p - currentP3D[lidarID]) * currentN3D[lidarID]);
				if ((vcg::Distance<float>(p, currentP3D[lidarID]) < 0.7) && (planedist < 0.1))
					md.points.push_back(p);
			}
			break;
		case 2:
			vcg::Plane3f searchPlane;
			vcg::Box3f b;b.SetNull();
			std::vector<vcg::Point3f> toFit;
			
			for (unsigned int i = trackingHistory[lidarID].size() - 1; i > 0;i--) {
				b.Add(trackingHistory[lidarID][i]);
				toFit.push_back(trackingHistory[lidarID][i]);
				if (b.Diag() > 0.1)
					break;
			}
			if (b.Diag() > 0.1) {
				vcg::FitPlaneToPointSet(toFit, searchPlane);
				for (unsigned int i = 0; i < lidars[lidarID].lidar.latest_frame.x.size();++i) {
					vcg::Point3f p = /*lidars[lidarID].transfLidar **/ vcg::Point3f(lidars[lidarID].lidar.latest_frame.x[i], lidars[lidarID].lidar.latest_frame.y[i], lidars[lidarID].lidar.latest_frame.z[i]);
					float planedist = fabs(vcg::SignedDistancePointPlane(p, searchPlane));
					if ((vcg::Distance<float>(p, currentP3D[lidarID]) < 1.7) && (planedist < 0.2))
						md.points.push_back(p);
				}
			}
			break;
		}
	return md.points.size() > 20;
}

bool CorrespondencesDetector::detect3D(int lidarID, vcg::Point3f& p) { 
	if (region_selection(lidarID)) {
		vcg::Point3f n;
		if  (md.detect_quad_center(p, n)) {
			correspondences3D3Dbbox.Add(p);
			trackingHistory[lidarID].push_back(p);
			trackingState[lidarID] = 1;
			currentN3D[lidarID] = n;
			return true;
		}
		else
			trackingState[lidarID] = 2;
	}
	else
		trackingState[lidarID] = 0;

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
		//cv::circle(cameras[camID].dst, cv::Point2f(p[0],p[1]), 30, cv::Scalar(0, 0, 255), 10);
		cv::drawMarker(cameras[camID].dst, cv::Point2f(p[0], p[1]), cv::Scalar(0, 0, 255),cv::MARKER_CROSS, 10);
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
	lidars[1].lidar.latest_frame_mutex.unlock();
	lidars[0].lidar.latest_frame_mutex.unlock();
	
	for (unsigned int i = 0; i < cameras.size(); ++i)
		cameras[i].latest_frame_mutex.unlock();

	return true; 
}

void CorrespondencesDetector::detect(){ 

	int delta_th = 33;

	lockAll();

	int delta = max(lidars[0].epochtime, lidars[1].epochtime) - min(lidars[0].epochtime, lidars[1].epochtime);

	bool detected3D[2] = { false,false };
 	for (unsigned int i = 0; i < 2; ++i)
 		detected3D[i] = detect3D(i, currentP3D[i]);

	if (detected3D[0] && detected3D[1]) {
		if (delta < delta_th) {
			correspondences3D3D.push_back(Correspondence3D3D(std::pair(currentP3D[0], currentP3D[1])));
			correspondences3D3D_size = correspondences3D3D.size();
		}
		std::cout << "3d delta "<< delta << std::endl;
	}

	if (detected3D[0] || detected3D[1])
	{
		std::vector<bool> detected2D(nCams, false);
		std::vector < vcg::Point2f> p2D(nCams);

		if (!cameras.empty())
			for (unsigned int i = 0; i < nCams; ++i)
				if (cameras[i].reading)
					if (detect2D(i, p2D[i]))
						for (int iL = 0; iL < 2; ++iL) {
							delta = max(cameras[i].epochtime, lidars[iL].epochtime) - min(cameras[i].epochtime, lidars[iL].epochtime);
							if (detected3D[iL] && delta < delta_th*2) {
								std::cout << "image  delta " << delta << std::endl;
								correspondences3D2D[i].push_back(Correspondence3D2D(std::pair(currentP3D[iL], p2D[i]), iL));
								correspondences3D2D_size[i] = correspondences3D2D[i].size();
							}
						}
	}


	unlockAll();
}


bool CorrespondencesDetector::alignCamera(int camID){ 

	int n = 4;
	std::vector<Correspondence3D2D> corr;
	bool lidars_aligned = (lidars[1].transfLidar != vcg::Matrix44f().Identity());

	std::vector<Correspondence3D2D> allcorrs;
	for (unsigned int i = 0; i < correspondences3D2D[camID].size(); ++i)
		if (correspondences3D2D[camID][i].iLidar == 0)
			allcorrs.push_back(correspondences3D2D[camID][i]);
		else
			if (lidars_aligned) {
				allcorrs.push_back(correspondences3D2D[camID][i]);
				allcorrs.back().first = lidars[1].transfLidar * allcorrs.back().first;
			}

	PoissonDistribution(allcorrs, n, 100, corr);
	if (corr.size() < n)
		return false;

	usedCorrs3D2D[camID] = corr;

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
	if (res.size() < 4)
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
	for (unsigned int i = 0; i < correspondences3D2D.size(); ++i) {
		n = correspondences3D2D[i].size();
		fwrite(&n, 4, 1, fo);
		fwrite(&*correspondences3D2D[i].begin(), sizeof(Correspondence3D2D), correspondences3D2D[i].size(), fo);
	}
	fclose(fo);
}

void CorrespondencesDetector::load_correspondences(const char* filename) {
	FILE* fo = fopen(filename, "rb");
	int n;
	fread(& n, 4, 1, fo);
	correspondences3D3D.resize(n);
	fread(&* correspondences3D3D.begin(), sizeof(Correspondence3D3D), correspondences3D3D.size(), fo);
	fread(&n, 4, 1, fo);
	correspondences3D2D.resize(n);
	correspondences3D3D_size  = correspondences3D3D.size();

	for (unsigned int i = 0; i < correspondences3D2D.size(); ++i) {
		fread(&n, 4, 1, fo);
		correspondences3D2D[i].resize(n);
		correspondences3D2D_size[i] = correspondences3D2D[i].size();
		fread(&*correspondences3D2D[i].begin(), sizeof(Correspondence3D2D), correspondences3D2D[i].size(), fo);
	}
	fclose(fo);
}