#pragma once
#include <vcg/space/point3.h>
#include <vcg/space/point2.h>
#include <vcg/space/box3.h>


#include "common.h"
#include "detect_3d_marker.h"
#include "camera-markers.h"



class CorrespondencesDetector {
public:
	CorrespondencesDetector() :debug_mode(false) {
		trackingState[0] = trackingState[1] = 0;searchingRadius[0] = searchingRadius[1] = 0.5;
	}


	//correspondence for each camera (0..NUMCAM)
	std::vector<std::vector< Correspondence3D2D > >  correspondences3D2D;
	std::vector<  Correspondence3D3D   >  correspondences3D3D;

	void init(unsigned int _nCams) { nCams = _nCams;correspondences3D2D.resize(nCams);}

	bool region_selection(int lidarID);
	bool detect3D(int lidarID,vcg::Point3f & p);
	
	bool detect2D(int camID, vcg::Point2f& p);

	void detect();


	bool alignCamera(int camID);
	void alignCameras();
	bool alignLidars();
	vcg::Point3f currentP3D[2];
	vcg::Point3f currentN3D[2];

	int trackingState[2]; // 0: no tracking, 1: tracking, 2: searching
	float searchingRadius[2];
	std::vector<vcg::Point3f> trackingHistory[2];


	// debug
	void drawAll2DCorrs(int camID);
	void draw_debug();
	bool debug_mode;
	void save_correspondences(const char*filename);
	void load_correspondences(const char* filename);


private:
	bool lockAll();
	bool unlockAll();
	unsigned int nCams;
	vcg::Box3f correspondences3D3Dbbox;
	vcg::Plane3f p0[2], p1[2], p2[2];
	cameraMarkers  md2d;
	MarkerDetector md;
};
