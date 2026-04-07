#include "evaluate.h"
#include <iostream>
#include<omp.h>
void evaluate::process(PointSet& pset, double alphaValue)
{
	std::ostringstream buffer;
	//iterate all vertex groups
	double totalAlphaArea = 0.0;
	for (const auto& vg : pset.groups()) {
		totalAlphaArea += vg->alphashape(alphaValue);
	}
	double coverage = totalAlphaArea / pset.getbbox().area;
	buffer << "alpha value: " << alphaValue << std::endl;
	buffer << "Total alpha area: " << totalAlphaArea << std::endl;
	buffer << "bbox area:" << pset.getbbox().area << std::endl;
	buffer << "bbox minx,maxx" << pset.getbbox().min_x << ", " << pset.getbbox().max_x << std::endl;
	buffer << "bbox miny,maxy" << pset.getbbox().min_y << ", " << pset.getbbox().max_y << std::endl;
	buffer << "bbox minz,maxz" << pset.getbbox().min_z << ", " << pset.getbbox().max_z << std::endl;
	buffer << "coverage: " << coverage << std::endl;

	this->saveResult("evalue.txt", buffer);

	std::cout << "Total alpha area: " << totalAlphaArea << std::endl;
	std::cout << "coverage: " << coverage << std::endl;
}

bool evaluate::RMSE(PointSet& pset)
{
	std::ostringstream buffer;
	// iterate all vertex groups
	double squaredError = 0.0;
	const auto& groups = pset.groups();
	int totalPoints = 0;
#pragma omp parallel for reduction(+:squaredError)
	for (int i = 0; i < static_cast<int>(groups.size()); ++i) {
		squaredError += groups[i]->fittingError();  // 注意这里不能用 auto& vg，不能并发捕获引用
		totalPoints += groups[i]->size();
	}

	double rmse = std::sqrt(squaredError / totalPoints);
	buffer << "RMSE: " << rmse << std::endl;
	buffer << "group size: " << groups.size() << std::endl;
	buffer << "points size: " << totalPoints << std::endl;

	std::cout << "RMSE: " << rmse << std::endl;
	std::cout << "group size: " << groups.size() << std::endl;
	std::cout << "points size: " << totalPoints << std::endl;
	this->saveResult("RMSE.txt", buffer);
	return true;
}

bool evaluate::saveResult(const std::string& filename, const std::ostringstream& buffer)
{
	const std::string savedPath = this->baseFileName_ + filename;
	std::filesystem::path parentDir = std::filesystem::path(savedPath).parent_path();
	if (!std::filesystem::exists(parentDir)) {
		if (std::filesystem::create_directories(parentDir)) {
			//logger->info("Created directories: {}", parentDir.string());
		}
		else {
			//logger->error("Failed to create directories: {}", parentDir.string());
			return false;
		}
	}
	std::ofstream file(savedPath);
	file << buffer.str();
	std::cout << "save evluate result to " << savedPath << std::endl;
	return true;
}