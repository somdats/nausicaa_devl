#pragma once
#include <vcg/space/point3.h>
#include <vcg/space/point2.h>
#include "common.h"
#include "detect_3d_marker.h"
#include "camera-markers.h"

struct Correspondence: public std::pair< vcg::Point3f, vcg::Point2f>{
	const vcg::Point3f& p3d() { return first; };
	const vcg::Point2f& p2d() { return second;};
};

class CorrespondencesDetector {
public:
	CorrespondencesDetector() {}
	CorrespondencesDetector(unsigned int numcams):nCams(numcams) {}
	//correspondence for each camera (0..NUMCAM)
	std::vector<std::vector< Correspondence > >  correspondences;

	bool detect3D(int lidarID,vcg::Point3f & p);
	bool detect2D(int camID, vcg::Point2f& p);
	

	bool detect();


	bool alignCamera(int camID);
	bool alignLidars(vcg::Matrix44f & transf);


private:
	bool lockAll();
	bool unlockAll();
	unsigned int nCams;
	MarkerDetector md;
	vcg::Point3f currentP3D[2];
	vcg::Plane3f p0[2], p1[2], p2[2];
};
