/*****************************************************************//**
 * \file   component.cpp
 * \brief
 *
 * \author Fred
 * \date   December 2024
 *********************************************************************/
#include "component.h"
#include "point_set.h"
#include<iostream>
#include<fstream>

Plane_3 Component::fitPlane(const Component::Ptr comp)
{
	std::vector<Point_3> points;
	for (const auto& i : *comp) {
		points.push_back(Point_3(this->point_set()->points()[i].x, this->point_set()->points()[i].y, this->point_set()->points()[i].z));
	}
	Plane_3 plane;
	CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());

	return plane;
}