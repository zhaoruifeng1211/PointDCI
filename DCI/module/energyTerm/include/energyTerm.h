/*****************************************************************//**
 * \file   energyTerm.h
 * \brief
 *
 * \author Fred
 * \date   December 2024
 *********************************************************************/
#pragma once
#define CGAL_USE_EXCEPTION // for catch the exception of cgal
#include<vector>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <Eigen/Dense>
#include "../../basic/include/point_set.h"
#include "../../3rd/dkm/dkm_parallel.hpp"

struct variation {
	double tau;//offset
	std::array<double, 3> eta;
	double kappa;
};

class PointSet;
class energyTerm {
private:
	PointSet* point_set_;
	Bbox_3 bbox_;
	std::vector <std::array<double, 2>> stabilityValues;
	std::map<std::string, std::vector<int>> clusterResult;// the points indices of each cluster level
	std::vector<std::vector<double>> allNormalDiva_;//save the value of normal deviation without and lamda process
	std::vector<std::vector<double>> allCurvatureDiva_;//save the value of curvature deviation without and lamda process
	std::vector<std::vector<double>> variations_;//save the value of each points in each scales v<p,t> value
	std::map < std::string, std::set<Component::Ptr> > levelComponents_;//save the components of whcih share the same points in each level
	std::map<Component::Ptr, std::vector<std::pair<std::string, int>>> compLevelsNumver_;// save the intersection point number between each componeny and each level cluster
	std::map<std::string, std::vector<Component::Ptr>> basicLevelComponent_;// the basic components (the most representive) of each level
	std::map<std::string, std::vector<Component::Ptr>> basicLevelComponentFinal_;// filter by average persisitence from basicLevelComponent_
	std::map<std::string, std::vector<int>> basicLevelIndicesFinal_;// save the indices of all points of each level's basicFinal
	std::map<std::string, int> basicFinalSize_; // save the size of each level's basicFinal points
	double bboxArea = 0.0;
	const double basePersisitence = 2857.0;//TODO: const for now, we need a function to calculate it, if the idea valid
	std::string baseFilePath_;
	double lamda_normal = 1.0;
	double lamda_curvature = 0.5;
	int clusterNumber = 3;
	double basicThreshold = 0.6;
	double basicCompSizeThreshold = 0.8;
	char persThresholdFunc = 'p';
	std::string clusterResultPath_;
	std::string normalDeviationsPath_;
	std::string curvatureDeviationsPath_;
	std::string stabilityValuesPath_;
	std::string componentsEnergyPath_;
	std::string levelComponentsPath_;
private:
	//std::shared_ptr<spdlog::logger> logger;

public:
	std::string baseFilePath() { return baseFilePath_; }
	inline void setPara(double lamda_normal_, double lamda_curvature_, double clusterNumber_) {
		lamda_normal = lamda_normal_;
		lamda_curvature = lamda_curvature_;
		clusterNumber = clusterNumber_;
	};
	inline void setFilePath(std::string clusterResultPath, std::string normalDeviationsPath, std::string curvatureDeviationsPath, std::string stabilityValuesPath, std::string componentsEnergyPath, std::string levelComponentsPath) {
		this->clusterResultPath_ = clusterResultPath;
		this->normalDeviationsPath_ = normalDeviationsPath;
		this->curvatureDeviationsPath_ = curvatureDeviationsPath;
		this->stabilityValuesPath_ = stabilityValuesPath;
		this->componentsEnergyPath_ = componentsEnergyPath;
		this->levelComponentsPath_ = levelComponentsPath;
	}
	inline void setBasicThreshold(const double threshold) {
		basicThreshold = threshold;
	}
	PointSet* getPset() {
		return point_set_;
	}
	std::map < std::string, std::set<Component::Ptr> > getLevelComponents() {
		return levelComponents_;
	}

	void setBasicLevelComponent(std::map<std::string, std::vector<Component::Ptr>> comps) {
		basicLevelComponent_ = comps;
	}
	const std::map<std::string, std::vector<Component::Ptr>> getBasicLevelComp() {
		return basicLevelComponent_;
	}

	void setBasicLevelComponentFinal(std::map<std::string, std::vector<Component::Ptr>> compsFinal) {
		basicLevelComponentFinal_ = compsFinal;
	}
	std::map<std::string, std::vector<Component::Ptr>>  getBasicLevelComponentFinal() {
		return basicLevelComponentFinal_;
	}

	void setBasicLevelIndicesFinal(std::map<std::string, std::vector<int>> indicesFinal) {
		basicLevelIndicesFinal_ = indicesFinal;
	}
	std::map<std::string, std::vector<int>> getBasicLevelIndicesFinal() {
		return basicLevelIndicesFinal_;
	}
	void setBasicFinalSize(std::map<std::string, int> size) {
		basicFinalSize_ = size;
	}
	std::map<std::string, int> getBasicFinalSize() {
		return basicFinalSize_;
	}

	std::vector < std::vector < double >>& variations() {
		return this->variations_;
	}

public:
	energyTerm(PointSet* pset_ = 0) :point_set_(pset_)/*, logger(LoggerFactory<energyTerm>::getLogger())*/ {
		bbox_ = boundingBox(pset_);
		bboxArea = calBboxArea(bbox_);
		this->baseFilePath_ = pset_->baseFilePath();
	};
	void energyData();
	double areaAlphaShapes(Component::Ptr comp);
	void stabilityOfPoints(const std::string featureFilePath);
	std::array<double, 2> sdMean(const std::vector<double>& data);
	double computeStabilityCMA(const std::vector<double>& v);
	std::vector<double> normalized(const std::vector<double> values);
	double MAD(const std::vector<double>& data, const double mean);
	std::array<double, 2> standardDeviation(const std::vector <std::vector<double>>& data);

	std::array<double, 4> standardDeviation(const std::vector <std::array<double, 5>>& data, bool withMean);

	std::vector < double > CV(const std::vector<std::vector<std::array<double, 5>>>& features);
	double computeGeometricVariation(const variation var1, const variation var2, double scaleValue1, double sclaeValue2);
	double computeGeometricVariationModify(const variation var1, const variation var2, const double scaleValue1, const double scaleValue2);
	void stabilityOfPounts_variation(const std::string variationPath, const std::string scalePath, std::string method = "mean");
	void getOnlyVariation(const std::string variationPath, const std::string scalePath);
	void stabilityOfPoints_curvature(const std::string featureFilePath);
	void clusterOfPoints();
	void energyConstraint();
	void getBasic();
	void getFinalBasic();
	void energyToBasicFinal();
	void setComponentsStablity(const std::vector<std::array<double, 3>>& pointStability);
	void setComponentsStablity(const std::vector<double> pointStability);
	void getStabilityTerm();
	void getStabilityTerm(bool isCura);
	void getdisFidelity();
	void process2OPerator(const std::vector < double >stabvalue, const std::vector<double> similarValue);

	/** io */
	std::vector<double> readScale(const std::string scalePath);
	std::vector<std::vector<variation>> readVariation(const std::string variationPath);
	std::vector<std::vector<std::array<double, 5>>> readScaleFeatures(const std::string scaleFeaturePath);
	bool saveStable(const std::vector<double>& stableValue, const std::string filename = "");
	bool saveVariation();
	bool saveDeviation();
	bool saveEnergyTerm();
	bool readStabilityValues(const std::string featureFilePath);
	bool readClusterResult(const std::string clusterResultPath);
	bool readEnergyData(const std::string energyDataPath);
	bool readEnergyData(const std::string energyDataPath, bool isJson);
	bool readEnergyConstraint(const std::string energyConstraintPath);
	bool readLevelComponents(const std::string levelComponentsPath, bool isJson);
	bool readComponentLevelsNumber(const std::string componentLevelsNumPath, bool isJson);
	bool readColormapStablity(const std::string StabilityPath);
	bool readCurvatureStability(const std::string StabilityPath);
	bool saveCompLevelsNumber(const std::map<Component::Ptr, std::vector<std::pair<std::string, int>>>& compLevelsNumver);
	bool savePersAnalysis();

	/** tools */
	Bbox_3 boundingBox(PointSet* pset);
	double calBboxArea(Bbox_3 bbox) {
		double xRange = bbox.xmax() - bbox.xmin();
		double yRange = bbox.ymax() - bbox.ymin();
		double zRange = bbox.zmax() - bbox.zmin();
		return 2 * (xRange * yRange + xRange * zRange + yRange * zRange);
	};
	double normalAngle(const double n1_x, const double n1_y, const double n1_z, const double n2_x, const double n2_y, const double n2_z);
	double DoN(const double n1_x, const double n1_y, const double n1_z, const double n2_x, const double n2_y, const double n2_z);
	double curvatureChange(const double A_k1, const double A_k2, const double	B_k1, const double B_k2);
	double getAvgPersisitence(const std::vector<Component::Ptr> comps);
	double getMedianPersisitence(const std::vector<Component::Ptr> comps);
	double getAllPointsPersistence(const std::vector<Component::Ptr> comps);
	double getThresholdPers(const std::vector<Component::Ptr> comps, char func);
	void disFidelity(const Component::Ptr comp, const std::map<std::string, std::vector<Component::Ptr>> basic);

	/** tools for some ideas pre-check */
	void saveStableComponentOfBasic();
};
