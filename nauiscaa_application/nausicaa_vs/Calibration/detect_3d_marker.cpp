#include "detect_3d_marker.h"
#include "vcg_mesh.h"
#include <vcg/complex/allocate.h>
#include <vcg/space/fitting3.h>
#include <vcg/math/histogram.h>
#include <vcg/space/intersection3.h> 
#include <wrap/io_trimesh/export.h>

void _save_points(std::vector<vcg::Point3f> ps,const char * filename) {
	CMesh m;
	vcg::tri::Allocator<CMesh>::AddVertices(m, ps.size());
	for (int i = 0; i < ps.size();++i)
		m.vert[i].P() = ps[i];
	vcg::tri::io::ExporterPLY<CMesh>::Save(m, filename);
}
void MarkerDetector::_save_plane(vcg::Plane3f plane, const char* filename) {
	std::vector<vcg::Point3f> pp;
	for (int i = 0; i < points.size(); ++i) 
		pp.push_back(plane.Projection(points[i]));
	_save_points(pp, filename);
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

//bool MarkerDetector::evaluate_plane(vcg::Plane3f plane) {
//	vcg::Distribution<float> d;
//	std::vector<float> vec;
//	for (unsigned int i = 0; i < points.size(); ++i) {
//		vec.push_back( (vcg::SignedDistancePlanePoint(plane, points[i])));
//		d.Add(vec.back());
//	}
//
//	int good_points = 0;
//	float dev = d.StandardDeviation(); // quantify a distance-dependent threshold instead
//	for (unsigned int i = 0; i < vec.size(); ++i)
//		if (fabs(vec[i]) <0.01)
//			++good_points;
//	if (good_points < 20)
//		return false;
//
//	// must be a separating plane
//	int far_plus  = 0;
//	int far_minus = 0;
//	for (unsigned int i = 0; i < vec.size(); ++i) {
//		if (vec[i] > 0.03)
//			far_plus++;else
//			if (vec[i] < -0.03)
//				far_minus++;
//	}
//	if (far_plus < far_minus) std::swap(far_plus, far_minus);
//	float ratio = far_minus / float(far_minus + far_plus);
//	if (ratio > 0.01)
//		return false;
//
//	return true;
//
//}

int MarkerDetector::evaluate_plane_1(vcg::Plane3f plane) {
	vcg::Distribution<float> d;
	std::vector<float> vec;
	for (unsigned int i = 0; i < points.size(); ++i) {
		vec.push_back((vcg::SignedDistancePlanePoint(plane, points[i])));
		d.Add(vec.back());
	}

	int good_points = 0;
	float dev = d.StandardDeviation(); // quantify a distance-dependent threshold instead
	for (unsigned int i = 0; i < vec.size(); ++i)
		if (fabs(vec[i]) < 0.01)
			++good_points;
//	if (good_points < 20)
//		return false;

	// must be a separating plane
	//int far_plus = 0;
	//int far_minus = 0;
	//for (unsigned int i = 0; i < vec.size(); ++i) {
	//	if (vec[i] > 0.03)
	//		far_plus++;else
	//		if (vec[i] < -0.03)
	//			far_minus++;
	//}
	//if (far_plus < far_minus) std::swap(far_plus, far_minus);
	//float ratio = far_minus / float(far_minus + far_plus);
	//if (ratio > 0.01)
	//	return 0;
	printf("score %d \n", good_points);
	return good_points;
}

//void MarkerDetector::generate_planes() {
//	int n_tries = 0;
//	std::srand(std::time(nullptr)); 
//	int n_planes = 500; // to be computed
//	int i0, i1, i2;
//
//	while (planes.size() < n_planes && ++n_tries < n_planes* n_planes) {
//		std::vector<vcg::Point3f> triple;
//		vcg::Plane3f plane;
//		random_triple(i0, i1, i2, points.size()-1);
//		triple.push_back(points[i0]);
//		triple.push_back(points[i1]);
//		triple.push_back(points[i2]);
//
//		vcg::FitPlaneToPointSet(triple,plane);
//
//		if(evaluate_plane(plane))
//			planes.push_back(plane);
//	}
//};

 bool MarkerDetector::fit_plane(vcg::Plane3f & bestplane) {
	int n_tries = 0;
	std::srand(std::time(nullptr));
	int n_planes = 500; // to be computed
	int i0, i1, i2;
	int value = 0;

	if (points.size() < 3)
		return false;
	while (++n_tries < n_planes ) {
		std::vector<vcg::Point3f> triple;
		vcg::Plane3f plane;
		random_triple(i0, i1, i2, points.size() - 1);
		printf("triple (%d) %d %d %d \n", points.size(), i0, i1, i2);
		triple.push_back(points[i0]);
		triple.push_back(points[i1]);
		triple.push_back(points[i2]);

		vcg::FitPlaneToPointSet(triple, plane);
		
		int v = evaluate_plane_1(plane);
		if (v > value) {
			value = v;
			bestplane = plane;
		}
	}
	return true;
};

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
	printf("----plane 0 ------- \n");
	 if(!fit_plane(p0)) return false;
	_save_plane(p0, "plane0.ply");
	remove_fitted(p0);
	_save_points(points, "after_0.ply");

	printf("----plane 1 ------- \n");
	if (!fit_plane(p1)) return false;
	_save_plane(p1, "plane1.ply");
	remove_fitted(p1);
	_save_points(points, "after_1.ply");


	printf("----plane 2 ------- \n");
	if (!fit_plane(p2)) return false;
	_save_plane(p2, "plane2.ply");
	remove_fitted(p2);
	_save_points(points, "after_2.ply");
	return true;
}


//bool MarkerDetector::partition_planes(vcg::Plane3f& p0, vcg::Plane3f& p1, vcg::Plane3f& p2) { 
//	if (planes.size() < 9)
//		return false;
//
//	// k-means (k==3)
//	std::vector<int> cluster[3];
//	vcg::Point3f  centroid[2][3];
//	int centroid_n[3];
//
//	std::vector<vcg::Point3f> normals;
//	for (int ip = 0; ip < planes.size(); ++ip) {
//		_save_plane(planes[ip], (std::string("plane")+ std::to_string(ip)+".ply").c_str());
//
//		normals.push_back(planes[ip].Direction());
//		if (normals.back()[2] < 0)
//			normals.back() = -normals.back();
//	}	
//	
//	_save_points(normals, "normals.ply");
//
//	int id[3];
//	random_triple(id[0], id[1], id[2], normals.size()-1);
//	for (int i = 0; i < 3; ++i) {
//		centroid[0][i] = normals[id[i]];
//		centroid[1][i] = vcg::Point3f(0, 0, 0);
//		centroid_n[i] = 0;
//	}
//
//	int ite = 0;
//	int total_ite = 0;
//	do {
//
//		for (int i = 0; i < normals.size(); ++i) {
//			int cl = 0;
//			float d  = vcg::SquaredDistance<float>(normals[i], centroid[ite][0]);
//			float nd = vcg::SquaredDistance<float>(normals[i], centroid[ite][1]);
//			if (d > nd) { d = nd; cl = 1;}
//			nd = vcg::SquaredDistance<float>(normals[i], centroid[ite][2]);
//			if (d > nd) { d = nd; cl = 2; }
//			centroid[(ite + 1) % 2][cl] += normals[i];
//			centroid_n[cl]++;
//		}
//		for (int i = 0; i < 3; ++i) 
//			if(centroid_n[i]>0)
//				centroid[(ite + 1) % 2][i]/= centroid_n[i] ;
//
//		for (int i = 0; i < 3; ++i) {
//			centroid_n[i] = 0;
//			centroid[ite][i] = vcg::Point3f(0,0,0);
//		}
//
//		ite = (ite + 1) % 2;
//	} while (total_ite++ < 10); // to fix
//
//
//	// take the plane with the normal closest to the centroid
//	int clos[3];
//	for (int i = 0; i < 3; ++i) {
//		float d = vcg::Distance<float>(normals[0], centroid[ite][i]);
//		clos[i] = 0;
//		for (int ip = 1; ip < planes.size(); ++ip) {
//			float nd = vcg::Distance<float>(normals[ip], centroid[ite][i]);
//			if (d > nd) {
//				d = nd;
//				clos[i] = ip;
//			}
//		}
//	}
//
//	if (clos[0] == clos[1] || clos[0] == clos[2] || clos[1] == clos[2])
//		return false;
//
//	p0 = planes[clos[0]];
//	p1 = planes[clos[1]];
//	p2 = planes[clos[2]];
//
//	return true; // handle all the failure cases
//};

//bool MarkerDetector::detect_corner_old(vcg::Point3f& corner) { 
//	planes.clear();
//	vcg::Plane3f  p0, p1, p2;
//	vcg::Line3f l;
// 
//	_save_points(this->points, "input.ply");
//
//	generate_planes();
//
//	if (partition_planes(p0, p1, p2)) {
//		vcg::IntersectionPlanePlane(p0,p1,l);
//		vcg::IntersectionLinePlane(l, p2, corner);
//		
//
//		 
//		return true;
//	}else
//	return false; 
//};

bool MarkerDetector::detect_corner(vcg::Point3f& corner) {

	planes.clear();
	vcg::Plane3f  p0, p1, p2;
	vcg::Line3f l;

	_save_points(points, "input.ply");
	if (find_planes(p0, p1, p2)) {
		vcg::IntersectionPlanePlane(p0, p1, l);
		vcg::IntersectionLinePlane(l, p2, corner);
	}

	return true;
};