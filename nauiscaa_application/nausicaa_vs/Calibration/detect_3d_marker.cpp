#include "detect_3d_marker.h"
#include "vcg_mesh.h"
#include <vcg/complex/allocate.h>
#include <vcg/space/fitting3.h>
#include <vcg/math/histogram.h>
#include <vcg/space/intersection3.h> 
#include <wrap/io_trimesh/export.h>

//#define PRINTOUT_DEBUG

void _save_points(std::vector<vcg::Point3f> ps,const char * filename) {
#ifdef PRINTOUT_DEBUG
	CMesh m;
	vcg::tri::Allocator<CMesh>::AddVertices(m, ps.size());
	for (int i = 0; i < ps.size();++i)
		m.vert[i].P() = ps[i];
	vcg::tri::io::ExporterPLY<CMesh>::Save(m, filename);
#endif
}
void MarkerDetector::_save_plane(vcg::Plane3f plane, const char* filename) {
#ifdef PRINTOUT_DEBUG
	std::vector<vcg::Point3f> pp;
	for (int i = 0; i < points.size(); ++i) 
		pp.push_back(plane.Projection(points[i]));
	_save_points(pp, filename);
#endif
}


void random_triple(int& i0, int& i1, int& i2,int max_value) {
	i0 = rand() / float(RAND_MAX) * max_value;
	do {
		i1 = rand() / float(RAND_MAX) * max_value;
	} while (i1 == i0);
	do {
		i2 = rand() / float(RAND_MAX) * max_value;
	} while (i2 == i0 || i2 == i1);
}

int MarkerDetector::evaluate_plane(vcg::Plane3f plane, float th) {
	vcg::Distribution<float> d;
	std::vector<float> vec;
	for (unsigned int i = 0; i < points.size(); ++i) {
		vec.push_back((vcg::SignedDistancePlanePoint(plane, points[i])));
		d.Add(vec.back());
	}

	int good_points = 0;
	float dev = d.StandardDeviation(); // quantify a distance-dependent threshold instead
	for (unsigned int i = 0; i < vec.size(); ++i)
		if (fabs(vec[i]) < th)
			++good_points;

	//printf("score %d \n", good_points);
	return good_points;
}


 bool MarkerDetector::fit_plane(vcg::Plane3f & bestplane, std::vector<vcg::Point3f>&pts, int &value) {
	int n_tries = 0;
	std::srand(std::time(nullptr));
	int n_planes = 30; // to be computed
	int i0, i1, i2;

	if (pts.size() < 3)
		return false;
	while (++n_tries < n_planes ) {
		std::vector<vcg::Point3f> triple;
		vcg::Plane3f plane;
		random_triple(i0, i1, i2, pts.size() - 1);
		triple.push_back((pts)[i0]);
		triple.push_back((pts)[i1]);
		triple.push_back((pts)[i2]);

		vcg::FitPlaneToPointSet(triple, plane);
		
		int v = evaluate_plane(plane,0.01);
		if (v > value) {
			value = v;
			bestplane = plane;
		}
	}
	return true;
};

 void MarkerDetector::close_to_plane(vcg::Plane3f& plane, std::vector<vcg::Point3f>& pts, float th) {
	 std::vector<float> vec;
	 pts.clear();
	 for (unsigned int i = 0; i < points.size(); ++i)
		 if (fabs(vcg::SignedDistancePlanePoint(plane, points[i])) < th)
			 pts.push_back(points[i]);
 }


void MarkerDetector::remove_fitted(vcg::Plane3f plane) {
	std::vector<vcg::Point3f> newPoints;
	for (unsigned int i = 0; i < points.size(); ++i) {
		float dst = fabs(vcg::SignedDistancePlanePoint(plane, points[i]));
		if (dst > 0.04)
			newPoints.push_back(points[i]);
	}
	points = newPoints;
}

bool MarkerDetector::find_planes(vcg::Plane3f& p0, vcg::Plane3f& p1, vcg::Plane3f& p2) {
	static int ip = -1;
	++ip;
	int value = 0;
	std::vector <vcg::Point3f> pts;
//	printf("----plane 0 ------- \n");
	close_to_plane(p0, pts, 0.08);
	fit_plane(p0, pts, value);
	if(!fit_plane(p0,points,value)) return false;
	close_to_plane(p0, pts, 0.01);
	vcg::FitPlaneToPointSet(pts, p0);

	 _save_plane(p0, (std::to_string(ip)+"_plane0.ply").c_str());
	remove_fitted(p0);
	value = 0;
	_save_points(points, "after_0.ply");

//	printf("----plane 1 ------- \n");
	close_to_plane(p1, pts, 0.08);
	fit_plane(p1, pts, value);
	if (!fit_plane(p1, points, value)) return false;
	close_to_plane(p1, pts, 0.01);
	vcg::FitPlaneToPointSet(pts, p1);
	_save_plane(p1, (std::to_string(ip) + "_plane1.ply").c_str());

	remove_fitted(p1);
	value = 0;
	_save_points(points, "after_1.ply");

//	printf("----plane 2 ------- \n");
	close_to_plane(p2, pts, 0.08);
	fit_plane(p2, pts, value);
	if (!fit_plane(p2, points, value)) return false;
	close_to_plane(p2, pts, 0.01);
	vcg::FitPlaneToPointSet(pts, p2);
	_save_plane(p2, (std::to_string(ip) + "_plane2.ply").c_str());

	remove_fitted(p2);
	value = 0;
	_save_points(points, "after_2.ply");
	return true;
}


bool MarkerDetector::detect_corner(vcg::Point3f& corner, vcg::Plane3f & p0, vcg::Plane3f& p1, vcg::Plane3f& p2) {
	vcg::Line3f l;

	_save_points(points, "input.ply");
	if (find_planes(p0, p1, p2)) {
		vcg::IntersectionPlanePlane(p0, p1, l);
		vcg::IntersectionLinePlane(l, p2, corner);
		return true;
	}
	else
		return false;
};


bool MarkerDetector::detect_corner(vcg::Point3f& corner) {

	vcg::Plane3f  _p0, _p1, _p2;
	return detect_corner(corner, _p0, _p1, _p2);
};