#include "point_set.h"
#include <fstream>
#include<filesystem>
#include <iostream>
#include <random>
#include <cassert>
#include<limits>
#include <omp.h>

/**
 * fit plane for all components.
 *
 */
void PointSet::fitPlane()
{
	for (auto& comp : this->components()) {
		auto plane_ = comp->fitPlane(comp);
		comp->setPlane(plane_);
	}
}

/**
 * get the point which distance to plane smaller than threshold.
 *
 * \param plane:
 * \param pointIndices
 */
std::vector<int> PointSet::getPointNearToPlane(const Plane_3 plane, const std::vector<int> pointIndices)
{
	std::vector<int> nearPoints;
	size_t n = pointIndices.size();
	double thresholdSquare = point2planeDis * point2planeDis;
	double denominatorSquared = plane.a() * plane.a() + plane.b() * plane.b() + plane.c() * plane.c();
	const auto& A = plane.a();
	const auto& B = plane.b();
	const auto& C = plane.c();
	const auto& D = plane.d();
	// Store near-point candidates in per-thread buffers.
	std::vector<std::vector<int>> localNearPoints(omp_get_max_threads());

#pragma omp parallel for
	for (size_t i = 0; i < n; ++i) {
		int idx = pointIndices[i];
		double numerator = A * points_[idx].x + B * points_[idx].y + C * points_[idx].z + D;
		double distanceSquare = numerator * numerator / denominatorSquared;
		if (distanceSquare < thresholdSquare) {
			int tid = omp_get_thread_num();
			localNearPoints[tid].push_back(idx);
		}
	}

	// Merge the per-thread buffers into the final near-point list.
	for (const auto& local : localNearPoints) {
		nearPoints.insert(nearPoints.end(), local.begin(), local.end());
	}

	return nearPoints;
}

cloudXYZ::Ptr PointSet::vg_to_pcl(PointSet* pset)
{
	cloudXYZ::Ptr clouds(new cloudXYZ);
	const std::vector<pcl::PointXYZ>& points = pset->points();
	for (int i = 0; i < points.size(); i++) {
		clouds->push_back(points[i]);
	}

	return clouds;
}

bool PointSet::save2vg(const std::map<int, std::vector<int>> pointSeg, const std::string filename, int pointNumThre)
{
	std::map<int, std::vector<int>> pointSeg_tmp;
	for (const auto& seg : pointSeg) {
		if (seg.second.size() < pointNumThre) {
			continue;
		}
		pointSeg_tmp[seg.first] = seg.second;
	}
	if (pointSeg_tmp.size() == 0) {
		std::cerr << "no segments to save!" << std::endl;
		return false;
	}
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(0.0, 1.0);
	std::string savePath = baseFilePath_ + filename;
	std::filesystem::path parentDir = std::filesystem::path(savePath).parent_path();

	std::ofstream output(savePath.c_str());
	if (output.fail()) {
		std::cerr << "could not open file\'" << savePath << "\'" << std::endl;
		return false;
	}
	output.precision(16);
	const int pointsNum = this->points_.size();
	output << "num_points: " << this->points_.size() << std::endl;
	for (std::size_t i = 0; i < this->points_.size(); ++i) {
		output << this->points_[i].x << " " << this->points_[i].y << " " << this->points_[i].z << std::endl;
	}
	output << "num_colors: 0" << std::endl;

	if (this->normals_.size() == this->points_.size()) {
		output << "num_normals: " << this->normals_.size() << std::endl;
		for (std::size_t i = 0; i < this->normals_.size(); ++i) {
			output << this->normals_[i].normal_x << " " << this->normals_[i].normal_y << " " << this->normals_[i].normal_y << std::endl;
		}
	}
	else {
		std::cerr << "normals size is not equal to points size!" << "normal size= " << this->normals_.size() << std::endl;
		output << "num_normals: 0" << std::endl;
	}
	output << "num_groups: " << pointSeg_tmp.size() << std::endl;
	for (const auto& seg : pointSeg_tmp) {
		output << "group_type: 0" << std::endl;
		output << "num_group_parameters: 4" << std::endl;
		const auto& plane = fitPlane(seg.second);
		output << "group_parameters: " << plane.a() << " " << plane.b() << " " << plane.c() << " " << plane.d() << std::endl;
		output << "group_label: unknown" << std::endl;
		output << "group_color: " << dis(gen) << " " << dis(gen) << " " << dis(gen) << std::endl;
		output << "num_points: " << seg.second.size() << std::endl;
		for (const auto& idx : seg.second) {
			output << idx << " ";
		}
		output << std::endl;
		output << "num_children: 0" << std::endl;
	}
	output.close();
	return true;
}

bool PointSet::savePointcloudsColorByLabel(const std::map<int, std::vector<int>>& pointsSeg, std::string filename, int discardThreshold)
{
	//save to ply
	cloudXYZRGB::Ptr cloud(new cloudXYZRGB);
	// Generate random RGB colors for each segment.
	std::random_device rd;                            // Random seed source.
	std::mt19937 gen(rd());                          // Random number generator.
	std::uniform_int_distribution<int> dis(0, 255); // Uniform distribution in [0, 255].
	for (const auto& segment : pointsSeg) {
		if (segment.second.size() < discardThreshold) {
			continue;
		}
		//color
		int r = dis(gen);
		int g = dis(gen);
		int b = dis(gen);
		for (const auto& indice : segment.second) {
			pcl::PointXYZRGB p;
			p.x = points_[indice].x;
			p.y = points_[indice].y;
			p.z = points_[indice].z;
			p.r = r;
			p.g = g;
			p.b = b;
			cloud->points.push_back(p);
		}
	}
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->height = 1; // Height 1 indicates an unorganized point cloud.
	std::string savePath = baseFilePath_ + filename;
	std::filesystem::path parentDir = std::filesystem::path(savePath).parent_path();
	// Ensure the parent directory exists before saving.
	if (!std::filesystem::exists(parentDir)) {
		if (std::filesystem::create_directories(parentDir)) {
			//logger->info("Created directories: {}", parentDir.string());
		}
		else {
			//logger->error("Failed to create directories: {}", parentDir.string());
			return false; // Abort on directory creation failure.
		}
	}
	if (pcl::io::savePLYFile(savePath, *cloud) == -1) {
		//logger->error("Failed to save PLY file.");
		return false;
	}
	else {
		return true;
	}
}

bool PointSet::savePointcloudsColorByLabel(const std::vector<int>& label, std::string filename, int threshold)
{
	std::map<int, std::vector<int>> pointsSeg;
	for (int i = 0; i < label.size(); i++) {
		if (label[i] == -1) {
			continue;
		}
		pointsSeg[label[i]].push_back(i);
	}
	this->savePointcloudsColorByLabel(pointsSeg, filename, threshold);
	// save2vg(pointsSeg, filename + ".vg", 200);
	return true;
}

bool PointSet::savePointcloudsWithFeature(const std::vector<std::vector<std::array<double, 5>>>& feature, std::string filePath)
{
	const auto& points = this->points();
	int scale = 50;
	for (int i = 0; i < scale; i++) {
		cloudXYZINormal::Ptr cloud(new cloudXYZINormal);
		int j = 0;
		for (const auto& pointFea : feature) {
			pcl::PointXYZINormal p;
			p.x = points[j].x;
			p.y = points[j].y;
			p.z = points[j].z;
			p.intensity = 0.0;
			p.normal_x = pointFea[i][0];
			p.normal_y = pointFea[i][1];
			p.normal_z = pointFea[i][2];
			cloud->push_back(p);
			j++;
		}
		cloud->width = static_cast<uint32_t>(cloud->points.size());
		cloud->height = 1; // Height 1 indicates an unorganized point cloud.
		cloud->is_dense = true;
		std::string savePath = filePath + std::to_string(i) + "normal.ply";
		std::filesystem::path parentDir = std::filesystem::path(savePath).parent_path();

		// Ensure the parent directory exists before saving.
		if (!std::filesystem::exists(parentDir)) {
			if (std::filesystem::create_directories(parentDir)) {
				//logger->info("Created directories: {}", parentDir.string());
			}
			else {
				//logger->error("Failed to create directories: {}", parentDir.string());
				return 1; // Abort on directory creation failure.
			}
		}
		if (pcl::io::savePLYFile(savePath, *cloud) == -1) {
			//logger->error("Failed to save PLY file.");
			break;
		}
		else {
			//logger->info("Save PLY file successfully: {}", savePath);
		}
	}
	return true;
}

bool PointSet::saveIndices2Cloud(const std::vector<int> indices, std::string fileName)
{
	const std::string savedPath = this->baseFilePath_ + fileName;
	//save to ply
	cloudXYZ::Ptr cloud(new cloudXYZ);
	for (const auto& i : indices) {
		pcl::PointXYZ p;
		p.x = points_[i].x;
		p.y = points_[i].y;
		p.z = points_[i].z;
		cloud->push_back(p);
	}
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->height = 1; // Height 1 indicates an unorganized point cloud.
	cloud->is_dense = true;
	std::filesystem::path parentDir = std::filesystem::path(savedPath).parent_path();

	// Ensure the parent directory exists before saving.
	if (!std::filesystem::exists(parentDir)) {
		if (std::filesystem::create_directories(parentDir)) {
			//logger->info("Created directories: {}", parentDir.string());
		}
		else {
			//logger->error("Failed to create directories: {}", parentDir.string());
			return false; // Abort on directory creation failure.
		}
	}
	if (pcl::io::savePLYFile(savedPath, *cloud) == -1) {
		//logger->error("Failed to save PLY file.");
		return false;
	}
	else {
		//logger->info("Save PLY file successfully: {}", savedPath);
		return true;
	}
}

/**
 * save vector<component> to local point segmentation clouds, and labeling each point by component with largest persisitence.
 *
 * \param comps
 * \param filename
 * \return
 */
bool PointSet::saveComponents2Cloud(const std::set<Component::Ptr> comps, std::string filename)
{
	std::map<Component::Ptr, std::vector<int>> pointsComponents;
	std::map<int, Component::Ptr> pointsMaxComponents;
	if (comps.size() < 1) {
		//logger->error("components is empty!");
		return false;
	}
	cloudXYZRGB::Ptr cloud(new cloudXYZRGB);
	for (const auto& comp : comps) {
		for (const auto& idx : comp->indices()) {
			if (pointsMaxComponents.find(idx) == pointsMaxComponents.end()) {
				pointsMaxComponents[idx] = comp;
			}
			else {
				if (comp->attri().energyterm.persistence > pointsMaxComponents[idx]->attri().energyterm.persistence) {
					pointsMaxComponents[idx] = comp;
				}
			}
			//pcl::PointXYZ p;
			//p.x = points_[idx].x;
			//p.y = points_[idx].y;
			//p.z = points_[idx].z;
			//cloud->points.push_back(p);
		}
	}
	for (const auto& pointMaxC : pointsMaxComponents) {
		pointsComponents[pointMaxC.second].push_back(pointMaxC.first);
	}

	// Generate random RGB colors for each label.
	std::random_device rd;                            // Random seed source.
	std::mt19937 gen(rd());                          // Random number generator.
	std::uniform_int_distribution<int> dis(0, 255); // Uniform distribution in [0, 255].
	for (const auto& segment : pointsComponents) {
		//color
		int r = dis(gen);
		int g = dis(gen);
		int b = dis(gen);
		for (const auto& indice : segment.second) {
			pcl::PointXYZRGB p;
			p.x = points_[indice].x;
			p.y = points_[indice].y;
			p.z = points_[indice].z;
			p.r = r;
			p.g = g;
			p.b = b;
			cloud->points.push_back(p);
		}
	}
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->height = 1; // Height 1 indicates an unorganized point cloud.
	cloud->is_dense = true;
	std::string savePath = baseFilePath_ + filename;
	std::filesystem::path parentDir = std::filesystem::path(savePath).parent_path();

	// Ensure the parent directory exists before saving.
	if (!std::filesystem::exists(parentDir)) {
		if (std::filesystem::create_directories(parentDir)) {
			//logger->info("Created directories: {}", parentDir.string());
		}
		else {
			//logger->error("Failed to create directories: {}", parentDir.string());
			return 1; // �����˳�
		}
	}
	if (pcl::io::savePLYFile(savePath, *cloud) == -1) {
		//logger->error("Failed to save PLY file.");
		return false;
	}
	else {
		//logger->info("Save PLY file successfully: {}", savePath);
		return true;
	}
}
bool PointSet::saveEachComponent(const std::set<Component::Ptr> comps, std::string folderName)
{
	for (const auto& comp : comps) {
		std::string filename = folderName + "/" + std::to_string(comp->index()) + "_" + std::to_string(comp->attri().persistence) + ".ply";
		saveIndices2Cloud(comp->indices(), filename);
	}

	return true;
}

void PointSet::getPointsLabel()
{
	std::map<int, std::vector<int>> pointsSeg;
	this->pointLabel_.assign(this->points_.size(), -1);
	std::map<int, std::vector<int>> lodPoints;
	std::map<int, int>pointsMinPers;
	std::map<int, int>pointsAvgPers;

	//get comp threshold
	for (int i = 0; i < pointsLod_.size(); i++) {
		int lodLevel = pointsLod_[i][0];
		int pointsMaxPers = pointsLod_[i][1];
		lodPoints[lodLevel].push_back(i);
		pointsAvgPers[lodLevel] += pointsMaxPers;
		pointsAvgPers[lodLevel] /= i + 1;
		if (pointsMinPers.find(lodLevel) == pointsMinPers.end()) {
			pointsMinPers[lodLevel] = pointsMaxPers;
		}
		else {
			if (pointsMaxPers < pointsMinPers[lodLevel]) {
				pointsMinPers[lodLevel] = pointsMaxPers;
			}
		}
	}
	const auto& points = this->points();
	//iter all points
	for (int i = 0; i < points.size(); i++) {
		//judge the lod level
		int pointsLevel = pointsLod_[i][0];
		int pointsMaxPers = pointsLod_[i][1];
		int persThreshold = pointsMinPers[pointsLevel];
		int doo = false;
		for (const auto& pointComp : this->pointsComponents_[i]) {
			if (this->components_[pointComp]->attri().persistence > persThreshold) {
				this->pointLabel_[i] = pointComp;
				pointsSeg[pointComp].push_back(i);
				doo = true;
				break;
			}
		}
		if (!doo && !this->pointsComponents_[i].empty()) {
			const auto& pointComp = *this->pointsComponents_[i].rbegin();
			this->pointLabel_[i] = pointComp;
			pointsSeg[pointComp].push_back(i);
		}
	}
	this->savePointcloudsColorByLabel(pointsSeg, "newsegments.ply", 200);

	for (const auto& level : lodPoints) {
		std::map<int, std::vector<int>> pointsLabelLevel;
		for (const auto& p : level.second) {
			if (this->pointLabel_[p] != -1) {
				pointsLabelLevel[this->pointLabel_[p]].push_back(p);
			}
		}
		this->savePointcloudsColorByLabel(pointsLabelLevel, std::to_string(level.first) + "newsegments.ply", 50);
	}
}

void PointSet::getSelectedThreshold()
{
	type_levelThresholds levelThresholds;//saved each level's threshold and corresponding threshold
	std::map<std::string, std::map<std::string, std::vector<int>>> pointsLevelPers;
	std::map<std::string, std::map<std::string, std::vector<int>>> levelPointsIdx;

	std::string outterLevel = "";
	std::string innerLevel = "";
	int idx = 0;
	for (const auto& point : this->pointsLod()) {
		int lodLevel = point[0];
		if (lodLevel < 0) {
			continue;
		}
		int pointsMaxPers = point[1];
		outterLevel = std::to_string(lodLevel / this->outterClusterNum);
		innerLevel = outterLevel + std::to_string(lodLevel % this->innerClusterNum);
		if (pointsLevelPers.find(outterLevel) == pointsLevelPers.end()) {
			pointsLevelPers[outterLevel] = std::map<std::string, std::vector<int>>();
			levelPointsIdx[outterLevel] = std::map<std::string, std::vector<int>>();
		}
		if (pointsLevelPers[outterLevel].find(innerLevel) == pointsLevelPers[outterLevel].end()) {
			pointsLevelPers[outterLevel][innerLevel] = std::vector<int>();
			levelPointsIdx[outterLevel][innerLevel] = std::vector<int>();
		}
		pointsLevelPers[outterLevel][innerLevel].push_back(pointsMaxPers);
		levelPointsIdx[outterLevel][innerLevel].push_back(idx);
		idx++;
	}

	//check
	for (const auto& level : pointsLevelPers) {
		for (const auto& inner : level.second) {
			std::cout << "outter: " << level.first << ", inner" << inner.first << std::endl;
		}
	}

	//get threshold
	for (const auto& outter : pointsLevelPers) {
		std::string outterLevel = outter.first;
		int outterMin = std::numeric_limits<int>::max(), outterMax = std::numeric_limits<int>::min();
		double outterAvg = 0;
		int outterSize = 0;
		for (const auto& inner : outter.second) {
			std::string innerLevel = inner.first;
			std::vector<int> pointsMaxPers = inner.second;
			int minPers = *std::min_element(pointsMaxPers.begin(), pointsMaxPers.end());
			int maxPers = *std::max_element(pointsMaxPers.begin(), pointsMaxPers.end());
			int sum = std::accumulate(pointsMaxPers.begin(), pointsMaxPers.end(), 0);
			double avgPers = sum / pointsMaxPers.size();
			levelThreshold thresholdInner;
			thresholdInner.min = minPers;
			thresholdInner.max = maxPers;
			thresholdInner.avg = avgPers;
			levelThresholds[outterLevel].second[innerLevel] = thresholdInner;

			outterAvg += sum;
			outterSize += pointsMaxPers.size();
			outterMin = std::min(outterMin, minPers);
			outterMax = std::max(outterMax, maxPers);
		}
		outterAvg /= outterSize;
		levelThreshold thresholdOutter;
		thresholdOutter.min = outterMin;
		thresholdOutter.max = outterMax;
		thresholdOutter.avg = outterAvg;
		levelThresholds[outterLevel].first = thresholdOutter;
	}

	for (const auto& outter : levelPointsIdx) {
		std::string outterLevel = outter.first;
		double outterSum = 0;
		int outterPointsSize = 0;
		for (const auto& inner : outter.second) {
			std::string innerLevel = inner.first;
			std::vector<int> pointsIdx = inner.second;
			double sum = 0;
			int deletePoint = 0;
			for (const auto& idx : pointsIdx) {
				if (this->pointsComponents_[idx].size() == 0) {
					deletePoint++;
					continue;
				}
				int minPers = this->components_[*this->pointsComponents_[idx].begin()]->attri().persistence;
				sum += minPers;
				outterSum += minPers;
			}
			double avgPers = sum / (pointsIdx.size() - deletePoint);
			outterPointsSize += (pointsIdx.size() - deletePoint);
			levelThresholds[outterLevel].second[innerLevel].orignalAverage = avgPers;
		}
		double outterAvg = outterSum / outterPointsSize;
		levelThresholds[outterLevel].first.orignalAverage = outterAvg;
	}

	//check
	for (const auto& outter : levelThresholds) {
		for (const auto& inner : outter.second.second) {
			std::cout << std::endl;
			std::cout << "outter: " << outter.first << ", inner" << inner.first << std::endl;
			std::cout << "min: " << outter.second.first.min << ", max: " << outter.second.first.max << ", avg: " << outter.second.first.avg << ", originalAvg: " << outter.second.first.orignalAverage << std::endl;
			std::cout << "inner min: " << inner.second.min << ", max: " << inner.second.max << ", avg: " << inner.second.avg << ", originalAvg: " << inner.second.orignalAverage << std::endl;
		}
	}

	//save above output to local

	this->saveThreshold(levelThresholds, "levelThreshold.txt");
	this->setlevelThresholds(levelThresholds);
}

void PointSet::getPointsStableByCompSize()
{
	std::vector<double> pointStable;
	const auto& points = this->points();
	for (int i = 0; i < points.size(); i++) {
		double stable = 0;
		for (const auto& pointComp : this->pointsComponents_[i]) {
			int persistence = this->components_[pointComp]->attri().persistence;
			if (persistence > 0) {
				stable += (50.0 - persistence);
			}
		}
		pointStable.push_back(stable);
	}

	this->savePointClousColorMap(pointStable, "compNumStable.ply");

	std::vector<double> normalizedValue;
	float min_value = *std::min_element(pointStable.begin(), pointStable.end());
	float max_value = *std::max_element(pointStable.begin(), pointStable.end());

	for (size_t i = 0; i < pointStable.size(); ++i) {
		float normalized_value = (pointStable[i] - min_value) / (max_value - min_value);
		normalizedValue.push_back(normalized_value);
	}
	std::string savedFilename = baseFilePath_ + "compNumStableValue.txt";
	std::ofstream stableFile(savedFilename);
	if (!stableFile.is_open()) {
		throw std::runtime_error("stableFile file not found!");
		exit(-1);
	}
	for (const auto& stable : normalizedValue) {
		stableFile << std::fixed << std::setprecision(7) << stable << std::endl;
	}
	stableFile.close();
}

void PointSet::getPointsStability_final()
{
	std::vector<double> pointStable(this->points().size(), 0.0); // Preallocate the output buffer.
	const auto& points = this->points();

#pragma omp parallel for
	for (int i = 0; i < points.size(); i++) {
		double stable = 0;
		int compNum = 0;
		for (const auto& pointComp : this->pointsComponents_[i]) {
			int persistence = this->components_[pointComp]->attri().persistence;
			if (persistence > 2) {
				//stable += (50.0 - persistence);
				compNum++;

				stable++;
				//stable += persistence;
			}
		}
		//if (compNum > 0)
		//	pointStable[i] = stable / compNum;
		//else
		//	pointStable[i] = 0.0; // Or another default value if needed.
		pointStable[i] = stable;
	}

	const auto& stabNormalized_tmp = this->normalized(pointStable);

	std::vector<double> stabNormalized(stabNormalized_tmp.size());

#pragma omp parallel for
	for (int i = 0; i < stabNormalized_tmp.size(); ++i) {
		stabNormalized[i] = 1 - stabNormalized_tmp[i];
	}

	saveSingleColumn(stabNormalized, "compNumStableValue.txt");
	saveSingleColumn(pointStable, "compNumStableValueOriginal.txt");

	this->savePointClousColorMap(stabNormalized, "compNumStable.ply");
	this->stabilityNormalized_ = stabNormalized;
}

void PointSet::getPointsStability_finalNew(const std::vector<std::vector<double>>& variation)
{
	std::vector<double > stab_tmp(variation.size(), 0);
	//get variation
#pragma omp parallel for
	for (int i = 0; i < variation.size() - 22; i++) {
		double frequency = getLocalMinNum(variation[i], 0.005);
		stab_tmp[i] = frequency;
	}
	const auto& stabNormalized_tmp = this->normalized(stab_tmp);
	std::vector<double> stabNormalized(stabNormalized_tmp.size());

#pragma omp parallel for
	for (int i = 0; i < stabNormalized_tmp.size(); ++i) {
		stabNormalized[i] = 1 - stabNormalized_tmp[i];
	}

	saveSingleColumn(stabNormalized, "compNumStableValue.txt");
	saveSingleColumn(stab_tmp, "compNumStableValueOriginal.txt");

	this->savePointClousColorMap(stabNormalized, "compNumStable.ply");
	this->stabilityNormalized_ = stabNormalized;
}

double PointSet::getLocalMinNum(const std::vector<double>& var, double threshold)
{
	int count = 0;
	int n = var.size();
	for (int i = 1; i < n - 1; ++i) {
		double leftDiff = var[i - 1] - var[i];
		double rightDiff = var[i + 1] - var[i];

		if (leftDiff > threshold && rightDiff > threshold) {
			++count;
		}
	}
	return count;
}

void PointSet::getPointsSimilarity_final()
{
	const auto& points = this->points();
	const auto& comps = this->components_;
	const int N = points.size();

	std::vector<double> maxPointsPers(N, 0.0); // Preallocate storage to avoid push_back races.

#pragma omp parallel for
	for (int i = 0; i < N; ++i) {
		if (this->pointsMaxComponent_[i].first >= 0) {
			int maxPointPers = comps[this->pointsMaxComponent_[i].first]->attri().persistence;
			maxPointsPers[i] = maxPointPers;
		}
		// Otherwise the default value 0.0 is already correct.
	}

	this->savePointClousColorMap(maxPointsPers, "compNumSimilarity.ply");
	const auto& simiNormalized = this->normalized(maxPointsPers);
	saveSingleColumn(simiNormalized, "compNumSimilarityValue.txt");
	this->similarityNormalized_ = simiNormalized;
}

void PointSet::generate()
{
	generateOverallSegments();
	genersteCoordinateSegments();
}

void PointSet::generateOverallSegments()
{
	std::map<int, std::vector<int>> pointsSeg;
	const auto& points = this->points();
	//iter all points
	for (int i = 0; i < points.size(); i++) {
		//judge the lod level
		std::string pointsLevel = std::to_string(int(pointsLod_[i][0] / this->outterClusterNum));
		int persThreshold = this->levelThresholds().at(pointsLevel).first.avg;
		int doo = false;
		for (const auto& pointCompidx : this->pointsComponents_[i]) {
			if (this->components_[pointCompidx]->attri().persistence > persThreshold) {
				pointsSeg[pointCompidx].push_back(i);
				doo = true;
				break;
			}
		}
		if (!doo && !this->pointsComponents_[i].empty()) {
			const auto& pointCompidx = *this->pointsComponents_[i].rbegin();
			pointsSeg[pointCompidx].push_back(i);
		}
	}
	this->savePointcloudsColorByLabel(pointsSeg, "newsegments.ply", 200);
	// save2vg(pointsSeg, "newsegments.vg", 200);
}

void PointSet::genersteCoordinateSegments()
{
	//const auto& points = this->points();
	//for (int i = 0; i < points.size(); i++) {
	//	//judge the level
	//	std::string pointsOutterLevel = std::to_string(int(pointsLod_[i][0] / 2));
	//	std::string pointsInnerLevel = pointsOutterLevel + std::to_string(pointsLod_[i][0] % 2);
	//	int persThreshold = this->levelThresholds().at(pointsOutterLevel).second.at(pointsInnerLevel).avg;
	//	int doo = false;
	//	for (const auto& pointCompidx : this->pointsComponents_[i]) {
	//		if (this->components_[pointCompidx]->attri().persistence > persThreshold) {
	//			this->pointLabel_[i] = pointCompidx;
	//			doo = true;
	//			break;
	//		}
	//	}
	//	if (!doo && !this->pointsComponents_[i].empty()) {
	//		const auto& pointCompidx = *this->pointsComponents_[i].rbegin();
	//		this->pointLabel_[i] = pointCompidx;
	//	}

	//}

	for (const auto& outter : this->levelThresholds()) {
		std::string outterLevel = outter.first;
		for (const auto& inner : outter.second.second) {
			std::string innerLevel = inner.first;
			int persThreshold = this->levelThresholds().at(outterLevel).second.at(innerLevel).avg;
			const auto& pointsLabel = this->getLabelByPers(persThreshold);
			this->savePointcloudsColorByLabel(pointsLabel, outterLevel + "_" + innerLevel + "newsegments.ply", 100);
		}
		int persThreshold = this->levelThresholds().at(outterLevel).first.avg;
		const auto& pointsLabel = this->getLabelByPers(persThreshold);
		this->savePointcloudsColorByLabel(pointsLabel, outterLevel + "_" + "newsegments.ply", 100);
	}
}

void PointSet::getCluster(const int firstLevelNum, const int SecondLevelNum)
{
	//process feature vector
	const auto& simi = this->similarityNormalized_;
	const  auto& stab = this->stabilityNormalized_;
	//1st clustering

	//2nd clustering
}

const std::vector<int> PointSet::getLabelByPers(const int pers)
{
	std::vector<int> pointsLabel;
	for (int i = 0; i < this->points_.size(); i++) {
		int label = -1;
		bool doo = false;
		for (const auto& pointCompidx : this->pointsComponents_[i]) {
			if (this->components_[pointCompidx]->attri().persistence >= pers) {//we need the minimal persistence component when statisfied persThreshold
				label = pointCompidx;
				doo = true;
			}
		}
		if (!doo) {
			label = *this->pointsComponents_[i].begin();
		}
		pointsLabel.push_back(label);
	}

	return pointsLabel;
}

std::vector<double> PointSet::normalized(const std::vector<double> values)
{
	std::vector<double> normalizedValue;
	float min_value = *std::min_element(values.begin(), values.end());
	float max_value = *std::max_element(values.begin(), values.end());

	for (size_t i = 0; i < values.size(); ++i) {
		float normalized_value = (values[i] - min_value) / (max_value - min_value);
		normalizedValue.push_back(normalized_value);
	}

	return normalizedValue;
}

void PointSet::trans2PointsLod(const std::vector<std::vector<std::vector<int>>>& cluster)
{
	int level = 0;
	std::vector<std::array<int, 2>> pointsLevel;
	pointsLevel.resize(this->points().size());
	const auto& comps = this->components_;
	for (const auto& outter : cluster) {
		for (const auto& inner : outter) {
			for (const auto& idx : inner) {
				int maxPers = comps[this->pointsMaxComponent_[idx].first]->attri().persistence;
				pointsLevel[idx] = { level, maxPers };
			}
			level++;
		}
	}
	this->pointsLod_ = pointsLevel;
}

void PointSet::trans2PointsLod_new(const std::vector<std::vector<int>>& cluster)
{
	int level = 0;
	std::vector<std::array<int, 2>> pointsLevel;
	pointsLevel.resize(this->points().size());
	const auto& comps = this->components_;
	for (const auto& outter : cluster) {
		for (const auto& idx : outter) {
			int componentIdx = this->pointsMaxComponent_[idx].first;

			if (componentIdx < 0) {
				pointsLevel[idx] = { -1, 0 };
			}
			else {
				int maxPers = comps[componentIdx]->attri().persistence;
				pointsLevel[idx] = { level, maxPers };
			}
		}
		level++;
	}
	this->pointsLod_ = pointsLevel;
}

Plane_3 PointSet::fitPlane(const std::vector<int>& pointsIndx)
{
	const auto& points = this->points();
	std::vector<Point_3> pointsFitting;
	for (const auto& i : pointsIndx) {
		pointsFitting.push_back(Point_3(points[i].x, points[i].y, points[i].z));
	}
	Plane_3 plane;
	CGAL::linear_least_squares_fitting_3(pointsFitting.begin(), pointsFitting.end(), plane, CGAL::Dimension_tag<0>());

	return plane;
}

void PointSet::getColor(float value, float& r, float& g, float& b)
{
	// Jet-style color ramp from low values (blue) to high values (red).
	float colors[6][3] = {
		{0, 0, 1}, // Blue
		{0, 1, 1}, // Cyan
		{0, 1, 0}, // Green
		{1, 1, 0}, // Yellow
		{1, 0.5, 0}, // Orange
		{1, 0, 0} // Red
	};
	int idx1 = static_cast<int>(value * 5); // Select the lower color interval.
	int idx2 = (idx1 + 1) % 6;
	float frac = value * 5 - idx1; // Linear interpolation factor.
	r = colors[idx1][0] * (1 - frac) + colors[idx2][0] * frac;
	g = colors[idx1][1] * (1 - frac) + colors[idx2][1] * frac;
	b = colors[idx1][2] * (1 - frac) + colors[idx2][2] * frac;
}
bool PointSet::savePointClousColorMap(const std::vector<double>& value, std::string filename)
{
	if (value.empty()) {
		return false;
	}
	float min_value = *std::min_element(value.begin(), value.end());
	float max_value = *std::max_element(value.begin(), value.end());
	const float range = max_value - min_value;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	colored_cloud->width = static_cast<uint32_t>(value.size());
	colored_cloud->height = 1;
	colored_cloud->points.resize(colored_cloud->width * colored_cloud->height);

	for (size_t i = 0; i < value.size(); ++i) {
		// Copy point coordinates.
		colored_cloud->points[i].x = this->points_[i].x;
		colored_cloud->points[i].y = this->points_[i].y;
		colored_cloud->points[i].z = this->points_[i].z;

		// Normalize the scalar value to [0, 1].
		float normalized_value = 0.0f;
		if (std::abs(range) > 1e-12f) {
			normalized_value = (value[i] - min_value) / range;
		}

		// Map the normalized value to RGB colors.
		float r, g, b;
		getColor(normalized_value, r, g, b);

		//Fred: low value->high value: blue(cold)->red(hot)
		// Store the mapped color in the output point cloud.
		colored_cloud->points[i].r = static_cast<uint8_t>(r * 255);
		colored_cloud->points[i].g = static_cast<uint8_t>(g * 255);
		colored_cloud->points[i].b = static_cast<uint8_t>(b * 255);
	}
	std::string savePath = baseFilePath_ + filename;
	std::filesystem::path parentDir = std::filesystem::path(savePath).parent_path();

	// Ensure the parent directory exists before saving.
	if (!std::filesystem::exists(parentDir)) {
		if (std::filesystem::create_directories(parentDir)) {
			//logger->info("Created directories: {}", parentDir.string());
		}
		else {
			//logger->error("Failed to create directories: {}", parentDir.string());
			return 1; // Abort on directory creation failure.
		}
	}
	if (pcl::io::savePLYFile(savePath, *colored_cloud) == -1) {
		//logger->error("Failed to save PLY file.");
		return false;
	}
	else {
		//logger->info("Save PLY file successfully: {}", savePath);
		return true;
	}
}
bool PointSet::savePointsWithSpeciColor(const std::string color)
{
	std::vector<int> aimPointsIndices;
	std::vector<int> aimPointsIndices_r;
	std::vector<int> aimPointsIndices_g;

	if (this->points_.size() == 0) {
		/*logger->error("0 points exits!");*/
		return false;
	}
	if (this->points_.size() != this->colors_.size()) {
		//logger->error("color size != points size");
	}

	int index = 0;
	for (const auto& color : this->colors_) {
		if (color.b > color.r && color.b > color.g && color.r < 100 && color.g < 100) {
			aimPointsIndices.push_back(index);
		}
		else if (color.r > color.g && color.r > color.b && color.g < 100 && color.b < 100) {
			aimPointsIndices_r.push_back(index);
		}
		else if (color.g > color.r && color.g > color.b && color.r < 100 && color.b < 100) {
			aimPointsIndices_g.push_back(index);
		}
		index++;
	}

	this->saveIndices2Cloud(aimPointsIndices, "bluePoints.ply");
	this->saveIndices2Cloud(aimPointsIndices_r, "redPoints.ply");
	this->saveIndices2Cloud(aimPointsIndices_g, "greenPoints.ply");

	{
		const std::string savedPath = this->baseFilePath_ + "bluePointsIndices.txt";
		const std::string savedPath_r = this->baseFilePath_ + "redPointsIndices.txt";
		const std::string savedPath_g = this->baseFilePath_ + "greenPointsIndices.txt";

		std::filesystem::path parentDir = std::filesystem::path(savedPath).parent_path();

		// Ensure the parent directory exists before saving.
		if (!std::filesystem::exists(parentDir)) {
			if (std::filesystem::create_directories(parentDir)) {
				//logger->info("Created directories: {}", parentDir.string());
			}
			else {
				//logger->error("Failed to create directories: {}", parentDir.string());
				return false; // Abort on directory creation failure.
			}
		}
		std::ofstream file(savedPath);
		if (!file.is_open()) {
			std::cerr << "Failed to open file for writing!" << std::endl;
			return 1; // Return an error code.
		}
		for (const auto& i : aimPointsIndices) {
			file << std::to_string(i) << std::endl;
		}
		file.close();

		std::ofstream file_r(savedPath_r);
		if (!file_r.is_open()) {
			std::cerr << "Failed to open file for writing!" << std::endl;
			return 1; // Return an error code.
		}
		for (const auto& i : aimPointsIndices_r) {
			file_r << std::to_string(i) << std::endl;
		}
		file_r.close();

		std::ofstream file_g(savedPath_g);
		if (!file_g.is_open()) {
			std::cerr << "Failed to open file for writing!" << std::endl;
			return 1; // Return an error code.
		}
		for (const auto& i : aimPointsIndices_g) {
			file_g << std::to_string(i) << std::endl;
		}
		file_g.close();
	}
	return true;
}
bool PointSet::saveSingleColumn(const std::vector<double>& value, std::string filename)
{
	const std::string savedPath = this->baseFilePath_ + filename;
	std::filesystem::path parentDir = std::filesystem::path(savedPath).parent_path();
	// Ensure the parent directory exists before saving.
	if (!std::filesystem::exists(parentDir)) {
		if (std::filesystem::create_directories(parentDir)) {
			//logger->info("Created directories: {}", parentDir.string());
		}
		else {
			//logger->error("Failed to create directories: {}", parentDir.string());
			return false; // Abort on directory creation failure.
		}
	}
	std::ostringstream buffer;
	std::ofstream file(savedPath);
	if (!file.is_open()) {
		std::cerr << "Failed to open file for writing!" << std::endl;
		return false; // Return an error code.
	}
	for (const auto& i : value) {
		buffer << std::fixed << std::setprecision(7) << i << "\n";
	}
	file << buffer.str();
	file.close();
	//logger->info("save {} to {}", filename, savedPath);
	return true;
}
bool PointSet::saveThreshold(const type_levelThresholds threshod, std::string filename)
{
	//save type_levelThresholds to local json
	const std::string savedPath = this->baseFilePath_ + filename;
	std::filesystem::path parentDir = std::filesystem::path(savedPath).parent_path();
	// Ensure the parent directory exists before saving.
	if (!std::filesystem::exists(parentDir)) {
		if (std::filesystem::create_directories(parentDir)) {
			//logger->info("Created directories: {}", parentDir.string());
		}
		else {
			//logger->error("Failed to create directories: {}", parentDir.string());
			return false; // Abort on directory creation failure.
		}
	}
	std::ofstream file(savedPath);
	if (!file.is_open()) {
		std::cerr << "Failed to open file for writing!" << std::endl;
		return false; // Return an error code.
	}
	for (const auto& outter : threshod) {
		file << outter.first << std::endl;
		file << outter.second.first.min << " " << outter.second.first.max << " " << outter.second.first.avg << " " << outter.second.first.orignalAverage << std::endl;
		for (const auto& inner : outter.second.second) {
			file << inner.first << " " << inner.second.min << " " << inner.second.max << " " << inner.second.avg << " " << inner.second.orignalAverage << std::endl;
		}
	}
	file.close();
	return true;
}
/**
 * .
 *
 * \param componentPath
 */
void PointSet::readComponent(std::string componentPath)
{
	int totalCompSize = 0;
	std::vector<Component::Ptr> components;
	std::ifstream input(componentPath);
	if (!input.is_open()) {
		//logger->error("Error: cannot open file {}", componentPath);
		exit(-1);
	}
	std::string line;
	int compIndex = 0;
	while (std::getline(input, line)) {
		Component::Ptr comp(new Component);
		comp->set_point_set(this);
		std::istringstream iss(line);
		int birth;
		int death;
		iss >> birth >> death;
		comp->set_Attri(birth, death);
		int index;
		while (iss >> index) {
			comp->push_back(index);
			//TODO add the points belong  component here?
				//TODO its segments!!!!!
		}
		comp->setIndex(compIndex);
		components.push_back(comp);
		totalCompSize += comp->size();
		compIndex++;
	}
	if (components.size() < 1) {
		//logger->error("components is empty!");
		/*exit(-1);*/
	}
	this->setComponents(components);
	input.close();
	this->totalComponentSize = totalCompSize;
	this->fitPlane();

	//logger->info("read {} components from {}", this->components().size(), componentPath);
}

void PointSet::readComponentPoint(std::string componentPath, int pointIdx)
{
	int totalCompSize = 0;
	std::set<Component::Ptr> componentsPoint;
	std::set<int> componentsPointIdx;
	std::vector<Component::Ptr> components;

	std::ifstream input(componentPath);
	if (!input.is_open()) {
		//logger->error("Error: cannot open file {}", componentPath);
		exit(-1);
	}
	std::string line;
	int compIndex = 0;
	while (std::getline(input, line)) {
		Component::Ptr comp(new Component);
		comp->set_point_set(this);
		std::istringstream iss(line);
		int birth;
		int death;
		int idx;
		iss >> birth >> death;
		comp->set_Attri(birth, death);
		int index;
		while (iss >> index) {
			if (index == pointIdx) {
				componentsPoint.insert(comp);
				componentsPointIdx.insert(compIndex);
			}
			comp->push_back(index);
		}
		comp->setIndex(compIndex);
		components.push_back(comp);
		totalCompSize += comp->size();
		compIndex++;
	}
	if (components.size() < 1) {
		//logger->error("components is empty!");
		/*exit(-1);*/
	}

	std::cout << "Points: " << pointIdx << " corresponding comp is :";
	for (const auto& idx : componentsPointIdx) {
		std::cout << idx << " ";
	}
	std::cout << std::endl;
}

/**
 * get the corresponding component in each scale of every point.
 *
 * \param segmentPath
 */
void PointSet::readSegment(std::string segmentPath)
{
	std::vector<int> pointsMaxPersLabel;//label the points by max pers component
	std::ifstream input(segmentPath);
	if (!input.is_open()) {
		throw std::runtime_error("Error: cannot open file " + segmentPath);
		exit(-1);
	}
	std::string line;
	int pointNum = 0, scaleNum = 0;
	std::getline(input, line);
	std::istringstream iss(line);
	iss >> pointNum >> scaleNum;
	if (pointNum <= 0 || scaleNum <= 0) {
		throw std::runtime_error("Feature file have nothing!");
		exit(-1);
	}
	int pointIndex = 0;
	while (std::getline(input, line)) {
		std::istringstream iss(line);
		int componentIdx = -1;
		int maxComponentIdx = -1;
		int scaleIdx = -1;
		int maxScaleIdx = -1;
		int pointMaxPers = 0;
		while (iss >> componentIdx) {
			if (componentIdx == -1) { // -1 means the point is unlabeled at this scale.
				continue;
			}
			if (components_[componentIdx]->attri().persistence > pointMaxPers) {
				maxComponentIdx = componentIdx;
				maxScaleIdx = scaleIdx;
				pointMaxPers = components_[componentIdx]->attri().persistence;
			}
			segments_[{pointIndex, scaleIdx}] = componentIdx;
			pointsComponents_[pointIndex].insert(componentIdx);
			scaleIdx++;
		}
		pointIndex++;
		pointsMaxComponent_.push_back({ maxComponentIdx,maxScaleIdx });
		pointsMaxPersLabel.push_back(maxComponentIdx);
	}
	input.close();
	assert(pointsMaxComponent_.size() == pointNum);

	//save
	this->savePointcloudsColorByLabel(pointsMaxPersLabel, "maxPersSeg.ply", 50);
}

void PointSet::readPointsLod(std::string pointsLodPath)
{
	//check file exist
	std::ifstream pointLodFile(pointsLodPath);
	if (!pointLodFile.is_open()) {
		//logger->error("Error: cannot open file {}", pointsLodPath);
		exit(-1);
	}
	std::vector<std::array<int, 2>> pointsLod;
	std::string line;
	int pointNum = 0;
	while (std::getline(pointLodFile, line)) {
		std::istringstream iss(line);
		int lodLevel = 0;
		int pointPers = 0;
		iss >> lodLevel >> pointPers;
		pointsLod.push_back({ lodLevel,pointPers });
	}

	if (pointsLod.size() != points_.size()) {
		//logger->error("Error: pointsLod size is not equal to points size");
		exit(-1);
	}
	this->setPoinsLod(pointsLod);
	pointLodFile.close();
	//logger->info("read {} pointsLod from {}", pointsLod.size(), pointsLodPath);
}

void PointSet::readPointsLodSep(std::string pointsLodPath)
{
	//check file exist
	std::ifstream pointLodFile(pointsLodPath);
	if (!pointLodFile.is_open()) {
		//logger->error("Error: cannot open file {}", pointsLodPath);
		exit(-1);
	}
	std::vector<std::array<int, 2>> pointsLod;
	std::string line;
	int pointNum = 0;
	while (std::getline(pointLodFile, line)) {
		std::istringstream iss(line);
		int pointsIdx = 0;
		int lodLevel = 0;
		iss >> pointsIdx >> lodLevel;
		const auto& comp = this->components_[this->pointsMaxComponent_[pointsIdx].first];
		int Maxpers = comp->attri().persistence;
		pointsLod.push_back({ lodLevel,Maxpers });
	}

	if (pointsLod.size() != points_.size()) {
		//logger->error("Error: pointsLod size is not equal to points size");
		exit(-1);
	}
	this->setPoinsLod(pointsLod);
	pointLodFile.close();
	//logger->info("read {} pointsLod from {}", pointsLod.size(), pointsLodPath);
}

void PointSet::readVG(std::string pointsVGPath)
{
	double min_x = std::numeric_limits<double>::max();
	double min_y = std::numeric_limits<double>::max();
	double min_z = std::numeric_limits<double>::max();
	double max_x = std::numeric_limits<double>::lowest();
	double max_y = std::numeric_limits<double>::lowest();
	double max_z = std::numeric_limits<double>::lowest();

	std::ifstream input(pointsVGPath.c_str());
	if (input.fail()) {
		std::cerr << "could not open file\'" << pointsVGPath << "\'" << std::endl;
		return;
	}

	std::string dumy;
	std::size_t num;

	input >> dumy >> num;
	std::vector<pcl::PointXYZ>& points = this->points_;
	points.resize(num);
	double x, y, z;
	for (int i = 0; i < num; ++i) {
		input >> x >> y >> z;
		points[i].x = x;
		points[i].y = y;
		points[i].z = z;
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_y = std::min(min_y, y);
		max_y = std::max(max_y, y);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}

	bbox.max_x = max_x;
	bbox.max_y = max_y;
	bbox.max_z = max_z;
	bbox.min_x = min_x;
	bbox.min_y = min_y;
	bbox.min_z = min_z;
	double dx = max_x - min_x;
	double dy = max_y - min_y;
	double dz = max_z - min_z;
	bbox.area = 2 * (dx * dy + dx * dz + dy * dz);

	input >> dumy >> num;
	std::vector<color>& colors = this->colors_;
	colors.resize(num);
	for (int i = 0; i < num; ++i) {
		input >> colors[i].r >> colors[i].g >> colors[i].b;
	}

	input >> dumy >> num;
	std::vector<normal>& normals = this->normals_;
	normals.resize(num);
	for (int i = 0; i < num; ++i) {
		input >> normals[i].normal_x >> normals[i].normal_y >> normals[i].normal_z;
	}

	//////////////////////////////////////////////////////////////////////////

	std::size_t num_groups = 0;
	input >> dumy >> num_groups;
	for (int i = 0; i < num_groups; ++i) {
		VertexGroup::Ptr g = read_ascii_group(input);

		if (!g->empty()) {
			g->set_point_set(this);
			this->groups().push_back(g);
		}

		int num_children = 0;
		input >> dumy >> num_children;
		for (int j = 0; j < num_children; ++j) {
			//VertexGroup::Ptr chld = read_ascii_group(input);
			//if (!chld->empty()) {
			//	chld->set_point_set(this);
			//	g->add_child(chld);
			//}
		}

		std::filesystem::path filepath(pointsVGPath);
		std::string directory = filepath.parent_path().string();   // Directory containing the file.
		std::string filenameNoExt = filepath.stem().string();      // Filename without extension.

		this->baseFilePath_ = directory + "/" + filenameNoExt + "/" + filenameNoExt;
	}
}

void PointSet::saveClusterPointClouds(const std::vector<std::vector<int>>& cluster, std::string filename)
{
	int levelIdx = 0;
	const auto& points = this->points();
	for (const auto& level : cluster) {
		std::string outterLevel = std::to_string(levelIdx / this->outterClusterNum);
		std::string innerLevel = outterLevel + std::to_string(int(levelIdx % this->innerClusterNum));
		std::string savePath = filename + outterLevel + "_" + innerLevel + ".ply";
		this->saveIndices2Cloud(level, savePath);
		levelIdx++;
	}
}
void PointSet::readMaxPersLocal(std::string maxPersLocalPath)
{
	std::ifstream maxPersLocalFile(maxPersLocalPath);
	if (!maxPersLocalFile.is_open()) {
		//logger->error("Error: cannot open file {}", pointsLodPath);
		exit(-1);
	}

	std::vector<int> maxPers;
	std::string line;
	int pointNum = 0;
	int scale = 0;
	while (std::getline(maxPersLocalFile, line)) {
		std::istringstream iss(line);
		int maxPersValue = 0;
		iss >> maxPersValue;
		maxPers.push_back(maxPersValue);
	}
	if (maxPers.size() != points_.size()) {
		//logger->error("Error: maxPers size is not equal to points size");
		exit(-1);
	}
	this->maxPersValue_ = maxPers;
	maxPersLocalFile.close();
}

/**
 * read feature without any process, keep zero.
 *
 * \param scaleFeaturePath
 * \param keepZero
 * \return each point feature of all scale in a vector
 */
std::vector<std::vector<std::array<double, 5>>> PointSet::readScaleFeatures(const std::string scaleFeaturePath, bool keepZero)
{
	std::vector<std::vector<std::array<double, 5>>> scaleFeatures;
	//read feature from file
	std::ifstream featureFile(scaleFeaturePath);
	if (!featureFile.is_open()) {
		throw std::runtime_error("Feature file not found!");
	}
	//read line
	int pointNum = 0;
	int scaleNum = 0;
	const int scaleFeatureIdx = 5;//each point have {nx, ny ,nz, k1, k2} features in every scale
	std::string line;
	std::vector<std::string> feature;
	if (std::getline(featureFile, line)) {
		std::istringstream iss(line);
		iss >> pointNum >> scaleNum;
	}
	if (pointNum <= 0 || scaleNum <= 0) {
		throw std::runtime_error("Feature file have nothing!");
	}
	int nosiyPoint = 0;
	while (std::getline(featureFile, line)) {
		std::vector<std::array<double, 5>> eachPointFeatures;
		double nx, ny, nz, k1, k2;
		std::istringstream iss(line);
		for (int iter = 0; iter < scaleNum; iter++) {
			iss >> nx >> ny >> nz >> k1 >> k2;
			eachPointFeatures.push_back({ nx, ny, nz, k1, k2 });
		}
		if (eachPointFeatures.empty()) {
			nosiyPoint++;
			throw std::runtime_error("have nosiy pointLine!!!");
			continue;
		}
		scaleFeatures.push_back(eachPointFeatures);
	}
	featureFile.close();

	return scaleFeatures;
}

VertexGroup::Ptr PointSet::read_ascii_group(std::ifstream& input)
{
	std::string dumy;
	int type;
	input >> dumy >> type;

	int num;
	input >> dumy >> num;
	assert(num == 4);
	std::vector<double> para(num);
	input >> dumy;
	for (int i = 0; i < num; i++) {
		input >> para[i];
	}

	std::string label;
	input >> dumy >> label;
	float r, g, b;
	input >> dumy >> r >> g >> b;

	int num_points;
	input >> dumy >> num_points;

	VertexGroup::Ptr grp(new VertexGroup);
	grp->set_plane(Plane3D(para[0], para[1], para[2], para[3]));
	for (int i = 0; i < num_points; i++) {
		int idx;
		input >> idx;
		grp->push_back(idx);
	}

	return grp;
}

void PointSet::load_vg(PointSet* pset, const std::string& filename)
{
	std::string ext = filename.substr(filename.find_last_of(".") + 1);
	if (ext != "vg") {
		//logger->error("file extension is not vg");
		exit(1);
	}
	std::ifstream input(filename.c_str());
	if (input.fail()) {
		//logger->error("cannot open file {}", filename);
		exit(1);
	}

	std::string dumy;
	std::size_t num;

	input >> dumy >> num;
	std::vector<pcl::PointXYZ> points;
	points.resize(num);
	for (std::size_t i = 0; i < num; i++) {
		input >> points[i].x >> points[i].y >> points[i].z;
	}

	input >> dumy >> num;
	std::vector<color> colors;
	colors.resize(num);
	for (int i = 0; i < num; i++) {
		input >> colors[i].rgb;
	}

	input >> dumy >> num;
	std::vector<normal> normals;
	normals.resize(num);
	for (int i = 0; i < num; i++) {
		input >> normals[i].normal_x >> normals[i].normal_y >> normals[i].normal_z;
	}
	pset->setPoints(points);
	pset->setColors(colors);
	pset->setNormals(normals);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::vector<VertexGroup::Ptr> groups;
	std::size_t num_groups = 0;
	input >> dumy >> num_groups;
	for (int i = 0; i < num_groups; i++) {
		VertexGroup::Ptr g = read_ascii_group(input);

		if (!g->empty()) {
			g->set_point_set(pset);
			groups.push_back(g);
		}

		int num_children = 0;
		input >> dumy >> num_children;
		for (int j = 0; j < num_children; j++) {
		}
	}
	pset->setgroups(groups);
}

bool PointSet::reads(const std::string& filename)
{
	std::ifstream in(filename.c_str());
	if (in.fail())
	{
		//logger->error("Cannot open file {}", filename);
		return false;
	}
	in.close();

	//logger->info("Reading point set from {}", filename);

	std::string ext = filename.substr(filename.find_last_of(".") + 1);
	if (ext == "ply") {
		cloudXYZRGBNORMAL::Ptr P(new cloudXYZRGBNORMAL);
		if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(filename, *P) == -1) {
			//logger->error("Failed to read PLY file");
			return false;
		}
		else {
			//pcl::point to pset
			std::vector<Point_3> cgalPoints_tmp;
			std::vector<pcl::PointXYZ> points_tmp;
			std::vector<color> colors_tmp;
			std::vector<normal> normals_tmp;
			for (auto p = P->points.begin(); p != P->points.end(); p++) {
				Point_3 cgal_point = Point_3(p->x, p->y, p->z);
				pcl::PointXYZ point = pcl::PointXYZ(p->x, p->y, p->z);
				color c = color(p->r, p->g, p->b);
				normal n = pcl::Normal(p->normal_x, p->normal_y, p->normal_z, p->curvature);
				cgalPoints_tmp.push_back(cgal_point);
				points_tmp.push_back(point);
				colors_tmp.push_back(c);
				normals_tmp.push_back(n);
			}
			setPoints(points_tmp);
			setColors(colors_tmp);
			setNormals(normals_tmp);
			setCgalPoints(cgalPoints_tmp);
			//logger->info("Loaded {} data points from PLY file", P->width * P->height);
		}
	}
	else if (ext == "vg") {
		load_vg(this, filename);
	}

	if (this->num_points() < 1) {
		//logger->error("reading file failed (no data exist)");
		return false;
	}
	std::filesystem::path filepath(filename);
	std::string directory = filepath.parent_path().string();   // Directory containing the file.
	std::string filenameNoExt = filepath.stem().string();      // Filename without extension.

	this->baseFilePath_ = directory + "/" + filenameNoExt + "/" + filenameNoExt;

	pointLabel_.assign(this->num_points(), -1);
	return true;
}

//void PointSet::renewPointSet(const cloudXYZ* cloud)
//{
//	points_.clear();
//	colors_.clear();
//	normals_.clear();
//	planar_qualities_.clear();
//
//	for (int i = 0; i < cloud->size(); i++) {
//		points_.push_back(cloud->points[i]);
//	}
//}
