#include <iostream>
#include<filesystem>
#include <fstream>
#include<chrono>
#include "../module/basic/include/point_set.h"
#include "../module/energyTerm/include/energyTerm.h"
#include"./globInfo.h"

using json = nlohmann::json;
int main(int argc, char* argv[]) {
	// Parse command line arguments and load the config file.
	GlobInfo globInfo;
	if (!globInfo.parse(argc, argv)) {
		exit(-1);
	}

	// Main pipeline:
	// 1. read the point cloud
	// 2. read component and segment metadata
	// 3. export stability and similarity values
	PointSet* pset = new PointSet();
	pset->reads(globInfo.pointcloudsFilePath);
	std::cout << "read p done" << std::endl;

	pset->readComponent(globInfo.componentFilePath);
	std::cout << "read c done" << std::endl;

	pset->readSegment(globInfo.segmentsFilePath);
	energyTerm data(pset);
	std::cout << "begin calculating variation-based stability" << std::endl;
	data.stabilityOfPounts_variation(globInfo.variationPath, globInfo.scalePath);
	std::cout << "begin calculating component-based stability and similarity" << std::endl;
	pset->getPointsStability_final();
	pset->getPointsSimilarity_final();
	std::cout << "finished exporting stability and similarity values" << std::endl;

	delete pset;
	return 0;
}
