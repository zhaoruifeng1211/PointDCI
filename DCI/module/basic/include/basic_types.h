/*****************************************************************//**
 * \file   basic_types.h
 * \brief
 *
 * \author Fred
 * \date   July 2024
 *********************************************************************/
#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H

#include <queue>
#include<string>
#include <memory>
#include"./json.hpp"
#include "./pcl_compat.h"
#include<spdlog/spdlog.h>
#include<spdlog/sinks/stdout_color_sinks.h>
#include <mutex>

/** CGAL INCLUDED */
#include<CGAL/Simple_cartesian.h>
#include<CGAL/Point_set_3.h>
#include<CGAL/Point_3.h>
#include<CGAL/Plane_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include<CGAL/Surface_mesh.h>
#include<CGAL/linear_least_squares_fitting_3.h>
#include<CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_3.h>
#include<CGAL/Polygon_2_algorithms.h>
#include <CGAL/Bbox_3.h>
/** CGAL DONE */

/** PCL DEFINED */
typedef pcl::PointCloud<pcl::PointXYZ>				cloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB>			cloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>		cloudXYZRGBNORMAL;
typedef pcl::PointCloud<pcl::PointXYZINormal>		cloudXYZINormal;
typedef pcl::RGB									color;
typedef pcl::Normal									normal;
/** PCL DONE */

/** CGAL DEFINED */
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Point_set_3<Point_3> Point_set;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Vector_3 Vector_3;
typedef CGAL::Alpha_shape_vertex_base_2<Kernel> VertexBase;
typedef CGAL::Alpha_shape_face_base_2<Kernel> FaceBase;
typedef CGAL::Triangulation_data_structure_2<VertexBase, FaceBase> TDS;
typedef CGAL::Delaunay_triangulation_2<Kernel, TDS> Triangulation;
typedef CGAL::Alpha_shape_2<Triangulation> Alpha_shape_2;

typedef CGAL::Bbox_3 Bbox_3;
/** CGAL DONE */

#define M_PI       3.14159265358979323846
class basic_types {
private:
	friend class PointSet;
	basic_types();//Private constructor to prevent instantiation

	std::string cloudBaseName{};
	double Scale{};//denote scale and Bs
	double BI{};
	std::string cloud_path{};
public:

	//static method to get the singleton instance
	static basic_types& getInstance() {
		static basic_types instance;
		return instance;
	}

	inline std::string getCloudBaseName() const {
		return cloudBaseName;
	}

	inline void setCloudBaseName(std::string cloudBaseName_) {
		cloudBaseName = cloudBaseName_;
	}

	inline double getScale() const {
		return Scale;
	}
	inline void setScale(double Scale_) {
		Scale = Scale_;
	}

	inline double getBI() const {
		return BI;
	}
	inline void setBI(double BI_) {
		BI = BI_;
	}

	inline std::string getCloud_path() const {
		return cloud_path;
	}
	inline void setCloud_path(std::string cloud_path_) {
		cloud_path = cloud_path_;
	}

	/**
	 * calculate the difference value betwwen max and min element in queue.
	 *
	 * \param queue_ : the queue for calculate the max difference value
	 * \return : the max difference value in queue
	 */
	static double maxDifference(std::queue<int> queue_);

	/**
	 * queue is stable if all elements are zero.
	 *
	 * \param queue_
	 * \return
	 */
	static bool isQueueStable(std::queue<int> queue_);
};

class Plane3D {
private:
	std::vector<double> coeff;
public:

	Plane3D() {
		coeff.resize(4);
		coeff[0] = 0;
		coeff[1] = 0;
		coeff[2] = 0;
		coeff[3] = 0;
	};
	Plane3D(double a_, double b_, double c_, double d_) {
		coeff.resize(4);
		coeff[0] = a_;
		coeff[1] = b_;
		coeff[2] = c_;
		coeff[3] = d_;
	}

	inline double a() const { return coeff[0]; }
	inline double b() const { return coeff[1]; }
	inline double c() const { return coeff[2]; }
	inline double d() const { return coeff[3]; }
};

//template <typename T>
//class Logger {
//public:
//	static std::shared_ptr<spdlog::logger> getLogger() {
//		static auto logger = spdlog::stdout_color_mt(typeid(T).name());
//		spdlog::set_pattern("[%Y-%m-%d %H:%M:%S] [%n] [%^%l%$] %v");
//		return logger;
//	}
//};
//template <typename T>
//class LoggerFactory {
//public:
//	static std::shared_ptr<spdlog::logger> getLogger() {
//		static auto logger = []() {
//			// Èç¹û logger ÉÐÎ´´´½¨
//			std::string name = typeid(T).name();  // Í¨¹ýÀàÐÍÃû³Æ´´½¨ logger
//			auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
//			// ÉèÖÃÈÕÖ¾¸ñÊ½Ò»´ÎÐÔ³õÊ¼»¯
//			if (!logger_initialized) {
//				sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [" + name + "] %v");
//				logger_initialized = true;
//			}
//
//			auto logger = std::make_shared<spdlog::logger>(name, sink);
//			logger->set_level(spdlog::level::info);
//			spdlog::register_logger(logger);
//			return logger;
//		}();
//		return logger;
//	}
//
//private:
//	static bool logger_initialized;
//};
#endif // !BASIC_TYPES_H
