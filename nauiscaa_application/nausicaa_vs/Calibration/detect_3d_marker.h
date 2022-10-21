#pragma once
#include <vcg/space/point3.h>
#include <vcg/space/plane3.h>
#include <vector>

//#define PRINTOUT_DEBUG

struct MarkerDetector {

	std::vector <vcg::Point3f> points;

	void _save_plane(vcg::Plane3f plane, const char* filename);

	int evaluate_plane(vcg::Plane3f plane,float th);
	bool fit_plane(vcg::Plane3f&, std::vector <vcg::Point3f> &,int&);
	void close_to_plane(vcg::Plane3f& plane, std::vector<vcg::Point3f>& pts, float th);
	void remove_fitted(vcg::Plane3f plane);
	void remove_unfitted(vcg::Plane3f plane);
	bool find_planes(vcg::Plane3f& p0, vcg::Plane3f& p1, vcg::Plane3f& p2);
	bool detect_corner(vcg::Point3f& corner, vcg::Plane3f& p0, vcg::Plane3f& p1, vcg::Plane3f& p2);
	bool detect_corner(vcg::Point3f& corner);

	bool detect_quad_center(vcg::Point3f& corner);

};