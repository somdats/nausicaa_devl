#pragma once
#include <vcg/space/point3.h>
#include <vcg/space/point2.h>
#include "common.h"


struct Correspondence: public std::pair< vcg::Point3f, vcg::Point2f>{
	const vcg::Point3f& p3d() { return first; };
	const vcg::Point2f& p2d() { return second;};
};

class CorrespondencesDetector {
public:
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

};
