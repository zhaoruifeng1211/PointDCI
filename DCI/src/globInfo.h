#ifndef GLOBINFO_H
#define GLOBINFO_H
#pragma once

#include <string>
#include <iostream>
#include <filesystem>
#include <cxxopts.hpp>
#include "json.hpp"

class GlobInfo
{
public:
	GlobInfo() : options("improved", "DCI point-cloud analysis pipeline") {
		options.add_options()
			("p,pointclouds", "point-cloud file path", cxxopts::value<std::string>())
			("c,components", "component file path", cxxopts::value<std::string>())
			("s,segments", "segment file path", cxxopts::value<std::string>())
			("f,features", "feature file path", cxxopts::value<std::string>())
			("u,clusters", "cluster file path", cxxopts::value<std::string>());
	}
public:
	cxxopts::Options options;

	// Basic input file paths.
	std::string pointcloudsFilePath;
	std::string componentFilePath;
	std::string segmentsFilePath;
	std::string featuresFilePath;
	std::string clustersFilePath;
	std::string stabilityFilePath;
	std::string variationPath; // Stores variation descriptors exported by pdpcFeature.
	std::string scalePath;
	std::string pointsLodFilePath;

	// Output and auxiliary file paths from the JSON config.
	std::string energyDataTermPath;
	std::string stabilityValuesPath;
	std::string clusterResultPath;
	std::string normalDeviationsPath;
	std::string curvatureDeviationPath;
	std::string componentsEnergyPath;
	std::string levelComponentsPath;
	std::string componentLevelsNumberPath;

	//json parameters
	double lamda_normal;
	double lamda_curvature;
	int clusterNumber;
	double basicThreshold;
	double lambda_coverage;
	double lambda_compact;
	double lambda_persistence;
	double lambda_level_0;
	double lambda_level_1;
	double lambda_level_2;
	int componentThreshold;

	std::string processLevel;

public:
	bool parse(int argc, char* argv[]);
};
#endif // !GLOBINFO_H
