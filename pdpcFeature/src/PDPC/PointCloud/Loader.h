#pragma once

#include <string>
#include <vector>
#include<CGAL/Simple_cartesian.h>
#include<CGAL/Point_set_3.h>
#include<CGAL/Point_3.h>
#include<CGAL/Plane_3.h>
#include<CGAL/linear_least_squares_fitting_3.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Point_set_3<Point_3> Point_set;
typedef Kernel::Plane_3 Plane_3;

namespace pdpc {

class PointCloud;

class Loader
{
public:
    static bool Load(const std::string& filename, PointCloud& g, bool verbose = true);
    static bool Save(const std::string& filename, const PointCloud& g, bool verbose = true);
    static bool saveVG(const std::string& filenaem,const PointCloud& g, const std::vector<int>& labeling,const int threshold=200,const bool verbose=true);
    static bool saveVGFast(const std::string& filenaem,const PointCloud& g, const std::vector<int>& labeling,const int threshold=200,const bool verbose=true);
    static Plane_3 fitPlane(const PointCloud& g,const std::vector<int>& pointsIndx);

};

} // namespace pdpc
