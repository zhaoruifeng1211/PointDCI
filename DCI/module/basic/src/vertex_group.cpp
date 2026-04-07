#include "vertex_group.h"
#include"./point_set.h"
#include <Eigen/Dense>
#include <omp.h>
//#include "vertex_group.h"
//#include "vertex_group.h"
//#include <pcl/segmentation/sac_segmentation.h>

//Plane3D VertexGroup::calculatePlane(const cloudXYZ* clouds)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	for (int i = 0; i < this->size(); i++) {
//		cloud->push_back(clouds->at(i));
//	}
//
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setDistanceThreshold(0.01);
//
//	seg.setInputCloud(cloud);
//	seg.segment(*inliers, *coefficients);
//	return Plane3D(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
//}

inline double pointToPlaneDistance_Eigen(const Eigen::Vector3d& point, const Eigen::Vector4d& plane) {
	double numerator = std::abs(plane.head<3>().dot(point) + plane[3]);
	double denominator = plane.head<3>().norm();
	return numerator / denominator;
}

double VertexGroup::alphashape(double alpha)
{
	const auto& originalPoints = this->point_set()->points();
	//pcl::pooint to cgal point
	std::vector<Point_3> points;
	for (int i = 0; i < this->size(); i++) {
		int idx = this->at(i);
		points.push_back(Point_3(originalPoints[idx].x, originalPoints[idx].y, originalPoints[idx].z));
	}
	Plane_3 fitted_plane(this->plane().a(), this->plane().b(), this->plane().c(), this->plane().d());
	//CGAL::linear_least_squares_fitting_3(
	//	points.begin(), points.end(), fitted_plane, CGAL::Dimension_tag<0>());

	// Build an orthonormal basis on the fitted plane.
	Vector_3 normal = fitted_plane.orthogonal_vector();
	Vector_3 base1 = fitted_plane.base1();
	Vector_3 base2 = fitted_plane.base2();

	// Normalize the in-plane basis vectors.
	base1 = base1 / std::sqrt(base1.squared_length());
	base2 = base2 / std::sqrt(base2.squared_length());

	// Project the 3D points onto the plane and build a 2D point set.
	std::vector<Point_2> projected_points;
	Point_3 origin = fitted_plane.projection(points[0]); // Use the first projected point as the local origin.
	for (const Point_3& p : points) {
		Vector_3 vec = p - origin;
		double x = vec * base1;
		double y = vec * base2;
		projected_points.emplace_back(x, y);
	}

	// Build the alpha shape from the projected 2D points.
	try {
		Alpha_shape_2 alpha_shape(projected_points.begin(), projected_points.end(),
			Kernel::FT(alpha), Alpha_shape_2::GENERAL);

		// Extract the alpha-shape boundary and estimate its enclosed area.
		std::vector<Point_2> boundary_points;
		for (auto it = alpha_shape.alpha_shape_edges_begin();
			it != alpha_shape.alpha_shape_edges_end(); ++it) {
			auto segment = alpha_shape.segment(*it);
			boundary_points.push_back(segment.source());
			boundary_points.push_back(segment.target());
		}

		// Remove duplicate boundary points before computing the area.
		std::sort(boundary_points.begin(), boundary_points.end());
		boundary_points.erase(std::unique(boundary_points.begin(), boundary_points.end()),
			boundary_points.end());
		double area = CGAL::polygon_area_2(boundary_points.begin(), boundary_points.end(), Kernel());
		//std::cout << "Alpha Shape Area: " << area << std::endl;

		return std::abs(area);
	}
	catch (...) {
		std::cerr << "Alpha Shape calculation failed!" << std::endl;
		return 0.0;
	}
}

double VertexGroup::fittingError()
{
	double fittingError = 0.0;
	const auto& originalPoints = this->point_set()->points();
	Eigen::Vector4d plane(this->plane().a(), this->plane().b(), this->plane().c(), this->plane().d());

	// Use a reduction to avoid accumulation conflicts across threads.
#pragma omp parallel for reduction(+:fittingError)
	for (int i = 0; i < this->size(); i++) {
		int idx = this->at(i);
		Eigen::Vector3d point(originalPoints[idx].x, originalPoints[idx].y, originalPoints[idx].z);
		double d = pointToPlaneDistance_Eigen(point, plane);
		fittingError += d * d;
	}

	return fittingError;
}
