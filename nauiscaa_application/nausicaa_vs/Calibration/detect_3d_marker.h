#pragma once
#include <vcg/space/point3.h>
#include <vcg/space/plane3.h>
#include <vector>

struct MarkerDetector {

	std::vector <vcg::Point3f> points;
	std::vector <vcg::Plane3f> planes;

	void _save_plane(vcg::Plane3f plane, const char* filename);
	//void generate_planes();
	//bool evaluate_plane(vcg::Plane3f plane);
	//bool partition_planes(vcg::Plane3f &p0, vcg::Plane3f& p1, vcg::Plane3f& p2 );
	//bool detect_corner_old(vcg::Point3f& corner);

	int evaluate_plane_1(vcg::Plane3f plane);
	bool fit_plane(vcg::Plane3f&);
	void remove_fitted(vcg::Plane3f plane);
	bool find_planes(vcg::Plane3f& p0, vcg::Plane3f& p1, vcg::Plane3f& p2);
	bool detect_corner(vcg::Point3f& corner);

};