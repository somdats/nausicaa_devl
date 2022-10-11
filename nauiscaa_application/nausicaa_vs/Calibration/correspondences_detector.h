#pragma once
#include <vcg/space/point3.h>
#include <vcg/space/point2.h>
#include <vcg/space/box3.h>


#include "common.h"
#include "detect_3d_marker.h"
#include "camera-markers.h"

typedef  std::pair< vcg::Point3f, vcg::Point2f> Correspondence3D2D;
typedef std::pair< vcg::Point3f, vcg::Point3f> Correspondence3D3D;

class CorrespondencesDetector {
public:
	CorrespondencesDetector( )  {}
	 
	//correspondence for each camera (0..NUMCAM)
	std::vector<std::vector< Correspondence3D2D > >  correspondences3D2D;
	std::vector<  Correspondence3D3D   >  correspondences3D3D;

	void init(unsigned int _nCams) { nCams = _nCams; }

	bool detect3D(int lidarID,vcg::Point3f & p);
	bool detect2D(int camID, vcg::Point2f& p);
	

	bool detect();


	bool alignCamera(int camID);
	bool alignLidars();

	vcg::Point3f currentP3D[2];

private:
	bool lockAll();
	bool unlockAll();
	unsigned int nCams;
	vcg::Box3f correspondences3D3Dbbox;
	vcg::Plane3f p0[2], p1[2], p2[2];
	cameraMarkers  md2d;
	MarkerDetector md;
};
