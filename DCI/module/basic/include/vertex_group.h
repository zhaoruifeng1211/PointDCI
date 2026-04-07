/*****************************************************************//**
 * \file   vertex_group.h
 * \brief
 *
 * \author Fred
 * \date   August 2024
 *********************************************************************/

#ifndef _VERTEX_GROUP_H_
#define _VERTEX_GROUP_H_

#include <vector>
#include<set>
#include<algorithm>
#include <memory>

 //#include"./point_set.h"
#include"./basic_types.h"

class PointSet;
class Plane3D;
class VertexGroup : public std::vector<unsigned int> {
public:
	typedef std::shared_ptr<VertexGroup> Ptr;
private:
	PointSet* point_set_;
	Plane3D plane_;
public:

	VertexGroup(PointSet* pset_ = 0) : point_set_(pset_) {}

	void set_point_set(PointSet* pset) { point_set_ = pset; }
	void set_plane(Plane3D plane) { plane_ = plane; }
	PointSet* point_set() { return point_set_; }
	Plane3D& plane() { return plane_; }
	~VertexGroup() {
		clear();
	}

	std::vector<int> indices() {
		std::vector<int> indices;
		for (auto i : *this) {
			indices.push_back(i);
		}
		return indices;
	}

	Plane3D calculatePlane(const cloudXYZ* clouds) {
		std::vector<Point_3> points;
		points.reserve(this->size());
		for (unsigned int index : *this) {
			const auto& point = clouds->at(index);
			points.emplace_back(point.x, point.y, point.z);
		}
		if (points.size() < 3) {
			return Plane3D();
		}
		Plane_3 fitted_plane;
		CGAL::linear_least_squares_fitting_3(
			points.begin(), points.end(), fitted_plane, CGAL::Dimension_tag<0>());
		return Plane3D(fitted_plane.a(), fitted_plane.b(), fitted_plane.c(), fitted_plane.d());
	};

	double alphashape(double alpha);
	double fittingError();
};
#endif // _VERTEX_GROUP_H_
