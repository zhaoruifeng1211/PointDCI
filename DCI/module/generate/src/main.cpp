#include "../include/generate.h"
#include <iostream>
#include <chrono>

int main(int argc, char* argv[]) {
	std::ostringstream LogBuffer;
	auto start = std::chrono::high_resolution_clock::now();
	auto startSep = std::chrono::high_resolution_clock::now();

	//parse command line arguments
	GlobInfo globInfo;
	//auto logger = globInfo.getLogger();
	if (!globInfo.parse(argc, argv)) {
		//logger->error("Failed to parse command line arguments.");
		std::cout << "Failed to parse command line arguments." << std::endl;
		exit(-1);
	}

	//1.read basic data file

	PointSet* pset = new PointSet();
	pset->reads(globInfo.pointcloudsFilePath);
	//component
	pset->readComponent(globInfo.componentFilePath);
	// pset->readComponentPoint(globInfo.componentFilePath, std::atoi(argv[2]));
	// exit(0);
	////segmentation
	pset->readSegment(globInfo.segmentsFilePath);
	{
		//generate the biggest pers seg
	}
	// energyTerm Data(pset);
	// std::cout << "begin cal v" << std::endl;
	// Data.getOnlyVariation(globInfo.variationPath, globInfo.scalePath);
	//2 process
	////stability
	// pset->getPointsStability_finalNew(Data.variations());
	pset->getPointsStability_final();
	pset->getPointsSimilarity_final();

	auto endSep = std::chrono::high_resolution_clock::now();

	// Measure the runtime of the DCI stage in minutes.
	std::chrono::duration<double, std::ratio<60>> duration_minutesSep = endSep - startSep;

	double minutesSep = duration_minutesSep.count();
	LogBuffer << "DCI runtime: " << minutesSep << " minutes" << "/n";

	startSep = std::chrono::high_resolution_clock::now();
	// 3. Cluster points using the normalized stability and similarity values.
	GenerateCluster* gen = new GenerateCluster(globInfo.clusterNumber, globInfo.clusterNumber);
	const auto& clusters = gen->getCluster(pset->simiNormalized(), pset->stabNormalized(), gen->outterClusterNum, gen->innerClusterNum);
	endSep = std::chrono::high_resolution_clock::now();
	duration_minutesSep = endSep - startSep;
	minutesSep = duration_minutesSep.count();
	LogBuffer << "Cluster runtime: " << minutesSep << " minutes" << "/n";

	pset->setOutterNum(gen->outterClusterNum);
	pset->setInnerNum(gen->innerClusterNum);
	pset->saveClusterPointClouds(clusters, "cluster");
	pset->trans2PointsLod_new(clusters);
	// 4. Build point LOD labels and estimate thresholds.
	//pset->readPointsLodSep(globInfo.pointsLodFilePath);
	startSep = std::chrono::high_resolution_clock::now();
	pset->getSelectedThreshold();
	endSep = std::chrono::high_resolution_clock::now();
	duration_minutesSep = endSep - startSep;
	minutesSep = duration_minutesSep.count();
	LogBuffer << "Threshold runtime: " << minutesSep << " minutes" << "/n";

	startSep = std::chrono::high_resolution_clock::now();
	pset->generate();
	endSep = std::chrono::high_resolution_clock::now();
	duration_minutesSep = endSep - startSep;
	minutesSep = duration_minutesSep.count();
	LogBuffer << "Generate runtime: " << minutesSep << " minutes" << "/n";

	// Record the full pipeline runtime.
	auto end = std::chrono::high_resolution_clock::now();

	// Compute the total runtime in minutes.
	std::chrono::duration<double, std::ratio<60>> duration_minutes = end - start;

	double minutes = duration_minutes.count();
	std::cout << "Program runtime: " << minutes << " minutes" << std::endl;
	LogBuffer << "Program runtime: " << minutes << " minutes" << std::endl;

	// Append the timing summary to the local log file.
	std::ofstream log(pset->baseFilePath() + "runtime_log.txt", std::ios::app);
	if (log.is_open()) {
		log << LogBuffer.str();
		log.close();
	}
	else {
		std::cerr << "Failed to open the log file." << std::endl;
	}
}
