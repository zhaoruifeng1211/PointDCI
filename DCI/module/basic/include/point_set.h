/*****************************************************************//**
 * \file   point_set.h
 * \brief
 *
 * \author Fred
 * \date   August 2024
 *********************************************************************/
#pragma once
#ifndef _POINT_SET_H_
#define _POINT_SET_H_

 //#include "./basic_types.h"
#include"./vertex_group.h"
#include "./component.h"
#include<set>
#include<map>
#include <utility>

struct levelThreshold {
	double max;
	double min;
	double avg;
	double orignalAverage;
};

struct BoundingBox {
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	double area;
};

typedef std::map<std::string, std::pair<levelThreshold, std::map<std::string, levelThreshold>>> type_levelThresholds;

class VertexGroup;
class Component;
//class basic_types;
class PointSet : public basic_types {
public:
	PointSet()/* :logger(LoggerFactory<PointSet>::getLogger())*/ {
	};
private:
	std::string baseFilePath_;
	std::string baseFileName_;
	//std::shared_ptr<spdlog::logger> logger;

private:
	std::vector<pcl::PointXYZ> points_;
	std::vector<Point_3> cgal_points_;
	std::vector<color>    colors_;
	std::vector<normal>   normals_;
	std::vector<double>   planar_qualities_;

	std::vector<VertexGroup::Ptr> groups_;
	std::vector<Component::Ptr> components_;
	std::vector<std::array<int, 2>> pointsLod_;//store the {lodLevel, pointPers} of each point in original. the pointPers is the pers of biggest component of points exist in.
	std::map<int, std::set<int>> pointsComponents_;//store the component indexs of points
	std::vector<int>pointLabel_;//the final segments result of all points, -1 denote discard
	std::map<std::pair<int, int>, int> segments_;// save each point component in each scale as {<pointIdx,scaleIdx>,component}
	std::vector<std::pair<int, int>> pointsMaxComponent_; // save the max component and corresponding scale of each point as <componentIdx,scale>
	std::vector<int> maxPersValue_;
	type_levelThresholds levelThresholds_;//saved each level's threshold and corresponding threshold
	std::vector<double> stabilityNormalized_;
	std::vector<double> similarityNormalized_;
	BoundingBox bbox;

	/** paramters */
	double point2planeDis = 0.05;
	int outterClusterNum = 3;
	int innerClusterNum = 3;
public:
	int totalComponentSize = 0;
public:
	void setPoints(std::vector<pcl::PointXYZ> points) {
		points_ = points;
	}
	void setColors(const std::vector<color>& colors) { colors_ = colors; }
	void setNormals(const std::vector<normal>& normals) { normals_ = normals; }
	void setCgalPoints(const std::vector<Point_3>& cgal_points) { cgal_points_ = cgal_points; }
	void setgroups(const std::vector<VertexGroup::Ptr>& groups) { groups_ = groups; }
	void setComponents(const std::vector<Component::Ptr>& components) { components_ = components; }
	void setPoinsLod(const std::vector<std::array<int, 2>>& pointsLod) { pointsLod_ = pointsLod; }
	void setlevelThresholds(const type_levelThresholds& levelThre) { levelThresholds_ = levelThre; }
public:
	void fitPlane();

	std::vector<int> getPointNearToPlane(const Plane_3 plane, const std::vector<int> pointIndices);

public:
	/** attribute */
	int num_points() const { return points_.size(); }

	const std::vector<pcl::PointXYZ>& points() { return points_; }
	const std::vector<Point_3>& cgal_points() { return cgal_points_; }
	const std::vector<color>& colors() { return colors_; }
	const std::vector<normal>& normals() { return normals_; }
	const std::vector<double>& planar_qualities() { return planar_qualities_; }
	std::vector<VertexGroup::Ptr>& groups() { return groups_; }
	const std::vector<Component::Ptr>& components() { return components_; }
	const std::vector<std::array<int, 2>>& pointsLod() { return pointsLod_; }
	const std::string baseFilePath() { return baseFilePath_; }
	const std::string baseFileName() { return baseFileName_; }
	const std::vector<int> maxPers() { return maxPersValue_; };
	const type_levelThresholds levelThresholds() { return levelThresholds_; }
	const std::vector<double> stabNormalized() { return this->stabilityNormalized_; }
	const std::vector<double> simiNormalized() { return this->similarityNormalized_; }
	void setOutterNum(int num) { this->outterClusterNum = num; }
	void setInnerNum(int num) { this->innerClusterNum = num; }
	BoundingBox getbbox() { return bbox; }

	/** Tools */
	cloudXYZ::Ptr vg_to_pcl(PointSet* pset);
	void getColor(float value, float& r, float& g, float& b);
	void getPointsLabel();
	void getSelectedThreshold();
	void getPointsStableByCompSize();
	void getPointsStability_final();
	void getPointsStability_finalNew(const std::vector<std::vector<double>>& variation);
	double getLocalMinNum(const std::vector < double >& var, double threshold);
	void getPointsSimilarity_final();
	void generate();
	void generateOverallSegments();
	void genersteCoordinateSegments();
	void getCluster(const int firstLevelNum = 3, const int SecondLevelNum = 2);
	const std::vector<int> getLabelByPers(const int pers);
	std::vector<double> normalized(const std::vector<double> values);
	void trans2PointsLod(const std::vector < std::vector<std::vector<int>>>& cluster);
	void trans2PointsLod_new(const std::vector<std::vector<int>>& cluster);
	Plane_3 fitPlane(const std::vector<int>& pointsIndx);

	/** IO */
	bool save2vg(const std::map<int, std::vector<int>> pointSeg, const std::string filename = "newSegments.vg", int pointNumThre = 200);
	bool savePointcloudsColorByLabel(const std::map<int, std::vector<int>>& pointsSeg, std::string filename = "newSegments.ply", int discardThreshold = 0);
	bool savePointcloudsColorByLabel(const std::vector<int>& label, std::string filename = "newSegments.ply", int threshold = 200);
	bool savePointcloudsWithFeature(const std::vector<std::vector<std::array<double, 5>>>& feature, std::string filePath);
	bool saveIndices2Cloud(const std::vector<int>indices, std::string fileName);
	bool saveComponents2Cloud(const std::set<Component::Ptr> comps, std::string filename = "finalSegments.ply");
	bool saveEachComponent(const std::set<Component::Ptr> comps, std::string folderName = "eachCompnent");
	bool savePointClousColorMap(const std::vector<double>& value, std::string filename);
	bool savePointsWithSpeciColor(const std::string color = "b");
	bool saveSingleColumn(const std::vector<double>& value, std::string filename);
	bool saveThreshold(const type_levelThresholds threshod, std::string filename);
	void readComponent(std::string componentPath);
	void readComponentPoint(std::string componentPath, int pointIdx = -1);
	void readSegment(std::string segmentPath);
	void readPointsLod(std::string pointsLodPath);
	void readPointsLodSep(std::string pointLodPath);
	void readMaxPersLocal(std::string maxPersLocalPath);
	void readVG(std::string pointsVGPath);
	void saveClusterPointClouds(const std::vector<std::vector<int>>& cluster, std::string filename = "cluster");
	std::vector<std::vector<std::array<double, 5>>> readScaleFeatures(const std::string scaleFeaturePath, bool keepZero);

	/**
	 * read a ascii group in vg.
	 *
	 * \param input
	 * \return Vertexgroup::Ptr
	 */
	static VertexGroup::Ptr read_ascii_group(std::ifstream& input);

	/**
	 * load ascii .vg file from local (the detailed .vg file format see PolyFit).
	 *
	 * \param pset
	 * \param filename
	 */
	void load_vg(PointSet* pset, const std::string& filename);

	bool reads(const std::string& filename);

	//void renewPointSet(const cloudXYZ* cloud);
};

#endif // !_POINT_SET_H_
