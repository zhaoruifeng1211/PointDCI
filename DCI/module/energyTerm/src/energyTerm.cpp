#include "energyTerm.h"
#include <string>
#include<filesystem>
#include <unordered_map>
#include <unordered_set>
#include<ranges>
#ifndef NDEBUG
#include<cassert>
#include <omp.h>
#include<array>
#include <cmath>
#endif // DEBUG
using json = nlohmann::json;

void energyTerm::energyData()
{
	std::vector<std::array<double, 3>> componentsEnergy;
	if (point_set_->points().size() == 0) {
		//logger->error("Point set is empty!");
	}
	int totalPointNumber = point_set_->points().size();
	for (auto comp : point_set_->components()) {
		//coverage
		//double coverage = 1 - static_cast<double>(comp->size()) / totalPointNumber;
		double coverage = static_cast<double>(comp->size()) / totalPointNumber;

		//compact
		double alphaShape = 0.0;
		if (comp->size() > 30) {
			alphaShape = std::abs(areaAlphaShapes(comp));
		}
		double compact = 1 - alphaShape / bboxArea;

		//persistence
		double persistence = 1 - comp->attri().persistence / basePersisitence;

		comp->setEnergyTerm(coverage, compact, persistence);
		componentsEnergy.push_back({ coverage, compact, persistence });
	}

#if 0
	//save componentsEnergy to local
	std::string componentsEnergyPath = point_set_->baseFilePath() + "_3_componentsEnergy.txt";
	std::ofstream componentsEnergyFile(componentsEnergyPath);
	if (!componentsEnergyFile.is_open()) {
		throw std::runtime_error("ComponentsEnergy file not found!");
	}
	componentsEnergyFile << componentsEnergy.size() << " " << "3" << std::endl;
	componentsEnergyFile << "coverage compact persistence" << std::endl;
	for (const auto& componentEnergy : componentsEnergy) {
		componentsEnergyFile << componentEnergy[0] << " " << componentEnergy[1] << " " << componentEnergy[2] << std::endl;
	}
#endif
#ifndef NDEBUG
	assert(componentsEnergy.size() == point_set_->components().size());
#endif // !NDEBUG
	//logger->info("Energy data term calculation done!");
}

double energyTerm::areaAlphaShapes(Component::Ptr comp)
{
	//pcl::pooint to cgal point
	std::vector<Point_3> points;
	for (int i = 0; i < comp->size(); i++) {
		points.push_back(Point_3(comp->point_set()->points()[comp->at(i)].x, comp->point_set()->points()[comp->at(i)].y, comp->point_set()->points()[comp->at(i)].z));
	}
	Plane_3 fitted_plane;
	CGAL::linear_least_squares_fitting_3(
		points.begin(), points.end(), fitted_plane, CGAL::Dimension_tag<0>());

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
			Kernel::FT(0.05), Alpha_shape_2::GENERAL);

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

void energyTerm::stabilityOfPoints(const std::string featureFilePath)
{
	//read feature from file
	std::ifstream featureFile(featureFilePath);
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
		std::vector<std::vector<double>> eachPointFeatures;
		double nx, ny, nz, k1, k2;
		std::istringstream iss(line);
		for (int iter = 0; iter < scaleNum; iter++) {
			iss >> nx >> ny >> nz >> k1 >> k2;
			if (nx == 0 && ny == 0 && nz == 0 && k1 == 0 && k2 == 0) {
				continue;
			}
			else {
				eachPointFeatures.push_back({ nx, ny, nz, k1, k2 });
			}
		}
		if (eachPointFeatures.empty()) {
			nosiyPoint++;
			throw std::runtime_error("have nosiy pointLine!!!");
			continue;
		}
#ifndef NDEBUG
		assert(scaleNum == 50);

		//assert(eachPointFeatures.size() == scaleNum);
#endif //DEBUG
		std::array<double, 2> standardDeviationValue = standardDeviation(eachPointFeatures);
		//double stabilityValue = standardDeviationValue.first + standardDeviationValue.second;//TODO: SUM() for now
		this->stabilityValues.push_back(standardDeviationValue);
	}
	featureFile.close();

	//save deviation to local
	saveDeviation();

	//save stabilityValues to local
	std::string stabilityValuesPath = baseFilePath_ + stabilityValuesPath_;
	std::ofstream stabilityFile(stabilityValuesPath);
	if (!stabilityFile.is_open()) {
		throw std::runtime_error("Stability file not found!");
	}
	for (const auto& stabilityValue : stabilityValues) {
		stabilityFile << stabilityValue[0] << " " << stabilityValue[1] << std::endl;
	}

	if (nosiyPoint > 0) {
		std::cout << "ERROR: noisy point count: " << nosiyPoint << std::endl;
	}

#ifndef NDEBUG
	assert(stabilityValues.size() == pointNum);
#endif //DEBUG
}

/**
 * compute the variance and mean of input data vector<double>.
 *
 * \param data
 * \return [0]-variance, [1]-mean
 */
std::array<double, 2> energyTerm::sdMean(const std::vector<double>& data)
{
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance, boost::accumulators::tag::mean>> acc;
	for (const auto& value : data) {
		acc(value);
	}
	double vari = std::sqrt(boost::accumulators::variance(acc));
	double Mean = boost::accumulators::mean(acc);
	return { vari,Mean };
}

double energyTerm::computeStabilityCMA(const std::vector<double>& v)
{
	const int N = v.size();
	if (N == 0) return 0.0;

	std::vector<double> M(N);  // Cumulative moving averages.
	M[0] = v[0];
	double M_prev = v[0];

	// 1. Compute the cumulative moving average sequence M_k.
	for (int k = 1; k < N; ++k) {
		M_prev += (v[k] - M_prev) / (k + 1); // k starts from 0, so the divisor is k + 1.
		M[k] = M_prev;
	}

	// 2. Compute the mean of the cumulative averages.
	double M_mean = 0.0;
	for (int k = 0; k < N; ++k) {
		M_mean += M[k];
	}
	M_mean /= N;

	// 3. Compute the variance of the cumulative averages.
	double variance = 0.0;
	for (int k = 0; k < N; ++k) {
		double diff = M[k] - M_mean;
		variance += diff * diff;
	}
	variance /= N; // Use N - 1 instead if an unbiased estimator is required.

	return variance;
}

std::vector<double> energyTerm::normalized(const std::vector<double> values)
{
	std::vector<double> normalizedValue;
	if (values.empty()) {
		return normalizedValue;
	}
	float min_value = *std::min_element(values.begin(), values.end());
	float max_value = *std::max_element(values.begin(), values.end());
	if (std::abs(max_value - min_value) < 1e-12f) {
		normalizedValue.assign(values.size(), 0.0);
		return normalizedValue;
	}

	for (size_t i = 0; i < values.size(); ++i) {
		float normalized_value = (values[i] - min_value) / (max_value - min_value);
		normalizedValue.push_back(normalized_value);
	}

	return normalizedValue;
}

double energyTerm::MAD(const std::vector<double>& data, const double mean)
{
	double totalDeviation = 0.0;
	for (double x : data) {
		totalDeviation += std::abs(x - mean);
	}
	double mad = totalDeviation / data.size();
	return mad;
}

/**
 * calculate the standardDeviation value of normal and curvature of each point along all smooth scales.
 *
 * \param data
 * \return
 */
std::array<double, 2> energyTerm::standardDeviation(const std::vector<std::vector<double>>& data)
{
	std::vector<double> normalDivaVector;
	std::vector<double> curvatureDivaVector;

	/** using SCALE 0 as reference */
	for (int i = 1; i < data.size(); i++) {
		//normal divation
		double normalDiva = normalAngle(data[0][0], data[0][1], data[0][2], data[i][0], data[i][1], data[i][2]);
		double curvatureDiva = curvatureChange(data[0][3], data[0][4], data[i][3], data[i][4]);
		normalDivaVector.push_back(normalDiva);
		curvatureDivaVector.push_back(curvatureDiva);
	}
#if 0
	/** use SCALE PREVIOUS as refernece */
	for (int i = 0; i < data.size() - 1; i++) {
		//normal divation
		double normalDiva = normalAngle(data[i][0], data[i][1], data[i][2], data[i + 1][0], data[i + 1][1], data[i + 1][2]);
		double curvatureDiva = curvatureChange(data[i][3], data[i][4], data[i][3], data[i + 1][4]);
		normalDivaVector.push_back(normalDiva);
		curvatureDivaVector.push_back(curvatureDiva);
	}
#endif
	allNormalDiva_.push_back(normalDivaVector);
	allCurvatureDiva_.push_back(curvatureDivaVector);
	//get Standard Deviation
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance>> normalAcc;
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance>> curvatureAcc;
	for (const auto& normal : normalDivaVector) {
		normalAcc(normal);
	}
	for (const auto& curvature : curvatureDivaVector) {
		curvatureAcc(curvature);
	}
	double normalDivaFinal = std::sqrt(boost::accumulators::variance(normalAcc));
	double curvatureDivaFinal = std::sqrt(boost::accumulators::variance(curvatureAcc));

	return std::array<double, 2>{normalDivaFinal, curvatureDivaFinal};
}

/**
 * .
 *
 * \param data
 * \param withMean
 * \return {normalDivaFinal,normalMean, curvatureDivaFinal,curvatureMean}
 */
std::array<double, 4> energyTerm::standardDeviation(const std::vector<std::array<double, 5>>& data, bool withMean)
{
	std::vector<double> normalDivaVector;
	std::vector<double> curvatureDivaVector;

	///** using SCALE 0 as reference */
	//for (int i = 1; i < data.size(); i++) {
	//	//normal divation
	//	double normalDiva = normalAngle(data[0][0], data[0][1], data[0][2], data[i][0], data[i][1], data[i][2]);
	//	double curvatureDiva = curvatureChange(data[0][3], data[0][4], data[i][3], data[i][4]);
	//	normalDivaVector.push_back(normalDiva);
	//	curvatureDivaVector.push_back(curvatureDiva);
	//}
#if 1
	/** use SCALE PREVIOUS as refernece */
	for (int i = 0; i < data.size() - 1; i++) {
		//normal divation
		double normalDiva = DoN(data[i][0], data[i][1], data[i][2], data[i + 1][0], data[i + 1][1], data[i + 1][2]);//TODO @20250226: we using DoN for now
		double curvatureDiva = curvatureChange(data[i][3], data[i][4], data[i][3], data[i + 1][4]);
		normalDivaVector.push_back(normalDiva);
		curvatureDivaVector.push_back(curvatureDiva);
	}
#endif
	allNormalDiva_.push_back(normalDivaVector);
	allCurvatureDiva_.push_back(curvatureDivaVector);
	//get Standard Deviation
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance, boost::accumulators::tag::mean>> normalAcc;
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance, boost::accumulators::tag::mean>> curvatureAcc;
	for (const auto& normal : normalDivaVector) {
		normalAcc(normal);
	}
	for (const auto& curvature : curvatureDivaVector) {
		curvatureAcc(curvature);
	}
	double normalDivaFinal = std::sqrt(boost::accumulators::variance(normalAcc));
	double curvatureDivaFinal = std::sqrt(boost::accumulators::variance(curvatureAcc));
	double normalMean = boost::accumulators::mean(normalAcc);
	double curvatureMean = boost::accumulators::mean(curvatureAcc);

	return std::array<double, 4>{normalDivaFinal, normalMean, curvatureDivaFinal, curvatureMean};
}

std::vector<double> energyTerm::CV(const std::vector<std::vector<std::array<double, 5>>>& features)
{
	return std::vector<double>();
}

/**
 * compute the geoVar (change rate) of each scale, reference original paper of 2012growing-equation(5).
 *
 * \param var1
 * \param var2
 * \param scaleValue1
 * \param sclaeValue2
 * \return
 */
double energyTerm::computeGeometricVariation(const variation var1, const variation var2, double scaleValue1, double sclaeValue2)
{
	double delta_t = sclaeValue2 - scaleValue1;
	double dtau_dt = (var2.tau - var1.tau) / delta_t;
	Eigen::Vector3d eta1(var1.eta[0], var1.eta[1], var1.eta[2]);
	Eigen::Vector3d eta2(var2.eta[0], var2.eta[1], var2.eta[2]);
	Eigen::Vector3d deta_dt = (eta2 - eta1) / delta_t;
	double dkappa_dt = (var2.kappa - var1.kappa) / delta_t;

	//calculate the term in the formulation (5) in 2012 growing paper
	double term1 = std::pow(dtau_dt, 2);
	double term2 = std::pow(scaleValue1 * deta_dt.norm(), 2);
	double term3 = std::pow(scaleValue1 * scaleValue1 * dkappa_dt, 2);
	//double term3 = 0.0;
	return term1 + term2 + term3;
}

/**
 * compute the changeValue in each scale, modify from the paper 2012Growing-equation(5).
 *
 * \param var
 * \param scaleValue
 * \return
 */
double energyTerm::computeGeometricVariationModify(const variation var1, const variation var2, const double scaleValue1, const double scaleValue2)
{
	double delta_t = scaleValue2 - scaleValue1;
	double tau = var1.tau;
	Eigen::Vector3d eta1(var1.eta[0], var1.eta[1], var1.eta[2]);
	Eigen::Vector3d eta2(var2.eta[0], var2.eta[1], var2.eta[2]);
	Eigen::Vector3d deta = (eta2 - eta1) /*/ delta_t*/;
	double dkappa = (var2.kappa - var1.kappa) /*/ delta_t*/;
	double term1 = std::pow(tau / scaleValue1, 2);
	double term2 = std::pow(/*scaleValue1 **/ deta.norm(), 2);
	double term3 = std::pow(/*scaleValue1 * scaleValue1 **/ dkappa * scaleValue1, 2);
	return term1 + term2 + term3;
}

void energyTerm::stabilityOfPounts_variation(const std::string variationPath, const std::string scalePath, std::string method)
{
	//logger->info("Start to calculate the stability of points using variation data.");
	std::vector<std::vector<double>> variationStable;// each points v(p,t) in each scale
	std::vector<double> finalStable;
	std::vector<double> finalStableMca;
	const auto& variations = readVariation(variationPath);
	const auto& scales = readScale(scalePath);
	std::cout << "read v,s done" << std::endl;

	int pointIndex = 0;
	for (const auto& points : variations) {
		std::vector<double> pointsValue;
		std::vector<double> pointsValueSingle;

		double value = 0.0;
		double single = 0.0;
		for (int i = 0; i < points.size() - 20; i++) {
			single = computeGeometricVariation(points[i], points[i + 1], scales[i], scales[i + 1]);
			value += single;
			if (value > 200) {
				value = 200;
			}

			//if (value > 20.0) {
			//	logger->warn("value:{}.point{} in scale{}: tau={}-{},sclale1={},scale2={}  ", value, pointIndex, i, points[i].tau, points[i + 1].tau, scales[i], scales[i + 1]);
			//}
			pointsValue.push_back(value);
			pointsValueSingle.push_back(single);
		}

		//finalStable.push_back(MAD(pointsValue, sdMean(pointsValue)[1]));
		std::array<double, 2> sdMeanValue = sdMean(pointsValue);
		finalStable.push_back(sdMeanValue[0]);
		double final = computeStabilityCMA(pointsValueSingle);
		if (final > 100) {
			final = 100;
		}
		finalStableMca.push_back(final);

		variationStable.push_back(pointsValue);
		pointIndex++;
	}
	std::cout << "cal v done" << std::endl;

	this->variations_ = variationStable;
	this->getPset()->savePointClousColorMap(finalStable, "stableColorMap_final.ply");
	this->getPset()->savePointClousColorMap(finalStableMca, "stableColorMap_newV_final_mean.ply");
	this->saveStable(finalStable, "stableValue.txt");
	this->saveStable(finalStableMca, "stableValue_mca.txt");

	this->saveVariation();

	//std::vector<double> similarityValue;
	//const auto& comp = this->getPset()->components();
	//for (const auto& point : this->getPset()->maxPers()) {
	//	int compIdx = point.first;
	//	double pers = comp[compIdx]->attri().persistence;
	//	similarityValue.push_back(pers);
	//}
	const auto& aa = this->getPset()->maxPers();
	if (aa.size() == finalStableMca.size()) {
		std::vector<double> doubleVec(aa.begin(), aa.end());
		this->process2OPerator(finalStableMca, doubleVec);
	} else {
		std::cout << "skip 2-operator export because max-persistence values are unavailable in the public main flow" << std::endl;
	}
}

void energyTerm::getOnlyVariation(const std::string variationPath, const std::string scalePath)
{
	std::vector<std::vector<double>> variationsValue;
	const auto& variations = readVariation(variationPath);
	const auto& scales = readScale(scalePath);
	std::cout << "read v,s done" << std::endl;
	for (const auto& points : variations) {
		std::vector<double> pointsValue;

		double value = 0.0;
		double single = 0.0;
		for (int i = 0; i < points.size() /*- 20*/; i++) {
			single = computeGeometricVariation(points[i], points[i + 1], scales[i], scales[i + 1]);
			pointsValue.push_back(single);
		}
		variationsValue.push_back(pointsValue);
	}

	this->variations_ = variationsValue;
	std::string savedFilename;
	savedFilename = baseFilePath_ + "variationValues.txt";

	std::ofstream stableFile(savedFilename);
	if (!stableFile.is_open()) {
		throw std::runtime_error("stableFile file not found!");
	}
	int pointIdx = 0;
	for (const auto& points : variationsValue) {
		stableFile << pointIdx << " ";
		for (const auto& value : points) {
			stableFile << std::fixed << std::setprecision(7) << value << " ";
		}
		stableFile << std::endl;
		pointIdx++;
	}
}

void energyTerm::stabilityOfPoints_curvature(const std::string featureFilePath)
{
	double epsilon = 1e-6;
	std::vector<double> pointsVc_normal, pointsVc_curvature;
	const auto& features = readScaleFeatures(featureFilePath);
	for (const auto& point : features) {
		const auto& stab = standardDeviation(point, true);
		double normalCV = 0.0, curvatureCV = 0.0;
		if (std::abs(stab[1]) < epsilon) {
			normalCV = stab[0];
		}
		else {
			normalCV = stab[0] / (stab[1] + epsilon);
		}
		if (std::abs(stab[3]) < epsilon) {
			curvatureCV = stab[2];
		}
		else {
			curvatureCV = stab[2] / (stab[3] + epsilon);
		}

		pointsVc_normal.push_back(stab[0]);
		pointsVc_curvature.push_back(stab[2]);
	}

	this->getPset()->savePointClousColorMap(pointsVc_normal, "normalStable.ply");
	this->getPset()->savePointClousColorMap(pointsVc_curvature, "curvatureStable.ply");
	saveDeviation();
}

void energyTerm::clusterOfPoints()
{
	std::unordered_map<std::string, int> frequency;
	std::unordered_map<std::string, std::vector<int>> pointcloudClusters;
	//lamda process

	auto clusters = dkm::kmeans_lloyd_parallel(stabilityValues, clusterNumber);
	std::cout << "Means:" << std::endl;
	for (const auto& mean : std::get<0>(clusters)) {
		std::cout << "\t(" << mean[0] << "," << mean[1] << ")" << std::endl;
	}
	int idx = 0;
	for (const auto& label : std::get<1>(clusters)) {
		frequency[std::to_string(label)]++;
		pointcloudClusters[std::to_string(label)].push_back(idx);
		idx++;
	}
	for (const auto& pair : frequency) {
		std::cout << "Character: " << pair.first << " Frequency: " << pair.second << std::endl;
	}

	//save cluster label to local
	std::string clusterResultPath = this->baseFilePath_ + clusterResultPath_;
	std::ofstream clusterResultFile(clusterResultPath);
	if (!clusterResultFile.is_open()) {
		throw std::runtime_error("ClusterResult file not found!");
	}
	clusterResultFile << "ClusterNum: " << frequency.size() << std::endl;
	for (const auto& mean : std::get<0>(clusters)) {
		clusterResultFile << mean[0] << " " << mean[1] << std::endl;
	}
	for (const auto& pair : frequency) {
		clusterResultFile << pair.first << " " << pair.second << std::endl;
	}
	for (const auto& label : std::get<1>(clusters)) {
		clusterResultFile << label << std::endl;
	}

	//save cluster result as pointcloud

	for (const auto& cluster : pointcloudClusters) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::string savePath = baseFilePath_ + "_4-2_cluster_" + cluster.first + ".ply";
		//std::ofstream clusterFile(savePath);
		for (int i = 0; i < cluster.second.size(); i++) {
			pcl::PointXYZ p;
			p.x = point_set_->points()[cluster.second[i]].x;
			p.y = point_set_->points()[cluster.second[i]].y;
			p.z = point_set_->points()[cluster.second[i]].z;
			cloud->points.push_back(p);
		}
		cloud->width = static_cast<uint32_t>(cloud->points.size());
		cloud->height = 1; // Height 1 indicates an unorganized point cloud.
		cloud->is_dense = true;
		if (pcl::io::savePLYFile(savePath, *cloud) == -1) {
			std::cerr << "Failed to save PLY file." << std::endl;
		}
		else {
			std::cout << "Save PLY file successfully: " << savePath << std::endl;
		}
	}
}
/**
 * .
 *
 */
void energyTerm::energyConstraint()
{
	std::vector<Component::Ptr> components = point_set_->components();
	std::map<std::string, std::vector<int>> sortedClusterIndices;
	for (const auto& cluster : clusterResult) {
		std::vector<int> sortedCluster = cluster.second;
		std::sort(sortedCluster.begin(), sortedCluster.end());
		sortedClusterIndices[cluster.first] = sortedCluster;
	}
#pragma omp parallel for
	for (const auto& comp : components) {
		std::map < std::string, std::vector<int>> levelIndices;
		int maxNum = 0;
		std::string maxLevel;
		std::vector<int> maxIndices;
		std::vector<int> indices = comp->indices();
		std::vector<int> compIndices(indices.begin(), indices.end());
		std::sort(compIndices.begin(), compIndices.end());
		for (const auto& cluster : clusterResult) {
			std::vector<int> sortedCluster = sortedClusterIndices[cluster.first];
			std::vector<int> intersection;
			//std::set<int> orderSet(cluster.second.begin(), cluster.second.end());
			std::ranges::set_intersection(compIndices, sortedCluster, std::back_inserter(intersection));
			compLevelsNumver_[comp].push_back({ cluster.first, static_cast<int>(intersection.size()) });

			//if (static_cast<int>(intersection.size()) > maxNum) {
			//	maxNum = static_cast<int>(intersection.size());
			//	maxLevel = cluster.first;
			//	maxIndices = std::move(intersection); // Use move semantics to reduce copying.
			//}
			maxLevel = cluster.first;
			maxIndices = intersection;
			levelIndices[maxLevel] = std::move(maxIndices);
		}
		// Use a critical section to avoid concurrent inserts into the shared map.
#pragma omp critical
		{
			levelComponents_[maxLevel].insert(comp);
		}
		comp->setLevelIndices(std::move(levelIndices));
	}
	saveCompLevelsNumber(compLevelsNumver_);
	//2.calcualte the energy constraint term
//openmp
#pragma omp parallel for
	for (const auto& comp : components) {
		std::map<std::string, double> energyConstraintTerm;
		for (const auto& level : comp->attri().levelIndices) {
			double energyConstraint = 1.0 - (level.second.size() / static_cast<double>(comp->size()));
			energyConstraintTerm[level.first] = energyConstraint;
		}
		comp->setEnergyConstraintTerm(energyConstraintTerm);
	}

	//logger->info("Energy constraint term calculation done!");
}

/**
 * compute the basic component of each levels for energy optimization.
 *
 */
void energyTerm::getBasic()
{
	std::map<std::string, std::vector<Component::Ptr>> basicLevelComponent;

	double averageCompSize = this->getPset()->totalComponentSize / static_cast<double>(this->getPset()->components().size());
	//logger->info("Average component size: {}", averageCompSize);

#pragma omp parallel
	{
		// Use a thread-local temporary map to avoid write conflicts.
		std::map<std::string, std::vector<Component::Ptr>> localLevelComponent;

#pragma omp for nowait
		for (size_t i = 0; i < point_set_->components().size(); ++i) {
			const auto& comp = point_set_->components()[i];
			for (const auto& level : comp->attri().levelIndices) {
				if (comp->size() < averageCompSize) {
					continue;
				}
				if (level.second.size() < comp->size() * basicThreshold) {
					continue;
				}
				localLevelComponent[level.first].push_back(comp);
			}
		}

		// Merge each thread-local result into the global map.
#pragma omp critical
		{
			for (const auto& [key, vec] : localLevelComponent) {
				basicLevelComponent[key].insert(
					basicLevelComponent[key].end(),
					vec.begin(),
					vec.end()
				);
			}
		}
	}
	{
		//process level 1 and level 2
		basicLevelComponent["1"].insert(basicLevelComponent["1"].end(), basicLevelComponent["0"].begin(), basicLevelComponent["0"].end());
		std::set<Component::Ptr> uniqueSet(
			basicLevelComponent["1"].begin(),
			basicLevelComponent["1"].end()
		);
		basicLevelComponent["1"].assign(uniqueSet.begin(), uniqueSet.end());

		basicLevelComponent["2"].insert(basicLevelComponent["2"].end(), basicLevelComponent["1"].begin(), basicLevelComponent["1"].end());
		std::set<Component::Ptr> uniqueSet2(
			basicLevelComponent["2"].begin(),
			basicLevelComponent["2"].end()
		);
		basicLevelComponent["2"].assign(uniqueSet2.begin(), uniqueSet2.end());
	}
	this->setBasicLevelComponent(basicLevelComponent);
	//save basic level component to local
	for (const auto& level : basicLevelComponent) {
		std::string basicLevelComponentPath = "_5_basicLevel_" + level.first + ".ply";
		std::set<Component::Ptr> levelSet(level.second.begin(), level.second.end());
		getPset()->saveComponents2Cloud(levelSet, basicLevelComponentPath);
		//getPset()->saveEachComponent(levelSet, "basicLevel_" + level.first);
	}

	// get basicFinal By the way
	getFinalBasic();
	{
		//save finalBasic Component
		for (const auto& level : this->getBasicLevelComponentFinal()) {
			std::set<Component::Ptr> compSet(level.second.begin(), level.second.end());
			this->getPset()->saveEachComponent(compSet, "basicLevelFinal" + level.first);
		}
		//logger->info("save basicFinal components Done!");
	}
}

/**
 * filter each component of basic by averagePersistence.
 *
 */
void energyTerm::getFinalBasic()
{
	std::map<std::string, std::vector<Component::Ptr>> basicLevelComponentFinal;
	auto basic = this->getBasicLevelComp();
	// get average persistence
	std::map<std::string, double> avgPers;
	for (const auto& level : basic) {
		double avgPers = this->getThresholdPers(level.second, persThresholdFunc);
		//filter
		std::vector<Component::Ptr> basicFinalTmp;
		for (const auto& comp : level.second) {
			if (comp->attri().persistence * comp->size() >= avgPers * 0.2) {
				basicFinalTmp.push_back(comp);
			}
		}
		basicLevelComponentFinal[level.first] = basicFinalTmp;
		{
			//save to local for check
			std::set<Component::Ptr> levelSet(basicFinalTmp.begin(), basicFinalTmp.end());
			this->getPset()->saveComponents2Cloud(levelSet, "basicFinal_" + level.first + ".ply");

			//logger->info("ThresholdPers of level {} is: {}", level.first, avgPers);
		}
	}

	this->setBasicLevelComponentFinal(basicLevelComponentFinal);
}

/**
 * get the energy value of every component to each level basicFinal.
 *
 */
void energyTerm::energyToBasicFinal()
{
	//logger->info("Start to calculate toBasicFinal energy...");
	const auto& basicFinal = this->getBasicLevelComponentFinal();
	// get all points of each basic final
	std::map<std::string, std::vector<int>> basicIndices;// save the all pointsIndices of each level's basicFinal

	// Phase 1: merge point indices for each final basic level.
#pragma omp parallel
	{
		std::map<std::string, std::vector<int>> localBasicIndices;

#pragma omp for nowait
		for (const auto& level : basicFinal) {
			for (const auto& comp : level.second) {
				auto indices = comp->indices();
				localBasicIndices[level.first].insert(
					localBasicIndices[level.first].end(),
					indices.begin(),
					indices.end()
				);
			}
		}

		// Merge each thread-local result into the global basicIndices map.
#pragma omp critical
		{
			for (const auto& [key, vec] : localBasicIndices) {
				basicIndices[key].insert(
					basicIndices[key].end(),
					vec.begin(),
					vec.end()
				);
			}
		}
	}
	setBasicLevelIndicesFinal(basicIndices);
	std::map<std::string, int> Size;
	for (const auto& level : basicIndices) {
		Size[level.first] = level.second.size();
	}
	this->setBasicFinalSize(Size);
	// Phase 2: compute the toBasicFinal energy term.
	int n = this->getPset()->components().size();
	const auto& component = this->getPset()->components();
	const auto& pset = this->getPset();
	const auto& basicLevelComponentFinal = this->getBasicLevelComponentFinal();
#pragma omp parallel for
	{
		for (const auto& level : basicLevelComponentFinal) {
			//double basicPers = getThresholdPers(level.second, persThresholdFunc);
			double basicPers = getAvgPersisitence(level.second);
			//logger->info("BasicPers of level {} is: {}", level.first, basicPers);
			for (size_t i = 0; i < n; ++i) {
				const auto& comp = component[i];
				int compPers = comp->attri().persistence;
				const auto& compPlane = comp->attri().plane;
				std::map<std::string, toBasicFinalEnergy> toBasicFinal;

				double persDifference = compPers - basicPers;

				int basicFinalNearIndices = pset->getPointNearToPlane(
					compPlane, basicIndices[level.first]
				).size();

				//toBasicFinal[level.first] = { persDifference, basicFinalNearIndices };
				comp->setToBasicFinal(level.first, { persDifference, basicFinalNearIndices });
			}
		}
	}
}

void energyTerm::setComponentsStablity(const std::vector<std::array<double, 3>>& pointStability)
{
	const auto& comps = this->getPset()->components();

#pragma omp parallel for
	for (int i = 0; i < comps.size(); i++) {
		int compSize = comps[i]->size();
		std::array<double, 3> compStability = { 0.0, 0.0, 0.0 };

		// Accumulate per-point stability values for this component.
#pragma omp parallel for reduction(+:compStability)
		for (int j = 0; j < compSize; j++) {
			compStability[0] += pointStability[comps[i]->at(j)][0];
			compStability[1] += pointStability[comps[i]->at(j)][1];
			compStability[2] += pointStability[comps[i]->at(j)][2];
		}

		// Normalize by the component size.
		compStability[0] /= compSize;
		compStability[1] /= compSize;
		compStability[2] /= compSize;

		// Store the final component stability value.
		comps[i]->setStability(compStability);
	}
}

void energyTerm::setComponentsStablity(const std::vector<double> pointStability)
{
	const auto& comps = this->getPset()->components();
	for (int i = 0; i < comps.size(); i++) {
		const auto& comp = comps[i];
		int compSize = comp->size();
		double compStability = 0.0;
		// Accumulate per-point stability values for this component.
		for (int j = 0; j < compSize; j++) {
			compStability += pointStability[comp->at(j)];
		}
		// Normalize by the component size.
		compStability /= compSize;
		// Store the final component stability value.
		comps[i]->setStability(compStability);
	}
}

/**
 * read stability value(curave for now) of points from local jason file.
 *
 * \param colormapStabilityPath
 * \return
 */
bool energyTerm::readColormapStablity(const std::string colormapStabilityPath)
{
	//check file exist
	if (!std::filesystem::exists(colormapStabilityPath)) {
		//logger->error("ColormapStability file not found!");
		return false;
	}
	std::ifstream colormapStabilityFile(colormapStabilityPath);
	if (!colormapStabilityFile.is_open()) {
		//logger->error("ColormapStability file not found!");
		return false;
	}
	//read json
	json colormapStability = json::parse(colormapStabilityFile);
	std::vector<std::array<double, 3>> colormapStabilityData_points;

#pragma omp parallel
	{
		// Per-thread temporary buffer.
		std::vector<std::array<double, 3>> local_data;

#pragma omp for
		for (size_t i = 0; i < colormapStability.size(); ++i) {
			std::array<double, 3> color;
			color[0] = colormapStability[i][0];
			color[1] = colormapStability[i][1];
			color[2] = colormapStability[i][2];

			// Append to the local buffer first.
			local_data.push_back(color);
		}

		// Merge into the shared output buffer under a critical section.
#pragma omp critical
		colormapStabilityData_points.insert(
			colormapStabilityData_points.end(),
			local_data.begin(),
			local_data.end()
		);
	}

	// set stability for each component
	setComponentsStablity(colormapStabilityData_points);
	//logger->info("read colormapStability done");
	return true;
}

bool energyTerm::readCurvatureStability(const std::string StabilityPath)
{
	//check file exist
	if (!std::filesystem::exists(StabilityPath)) {
		//logger->error("Stability file not found!");
		return false;
	}
	std::ifstream StabilityFile(StabilityPath);
	if (!StabilityFile.is_open()) {
		//logger->error("Stability file not found!");
		return false;
	}
	//read json
	json Stability = json::parse(StabilityFile);
	std::vector<double> curavtureStabilityData_points;
	for (const auto& i : Stability) {
		//logger->warn("type of stability value is {}",std::stod(i.value()));
		curavtureStabilityData_points.push_back(i);
	}
	if (curavtureStabilityData_points.size() != this->getPset()->points().size()) {
		//logger->error("Stability data size not match with points number!");
		return false;
	}

	//calculate the stable value of each component
	this->setComponentsStablity(curavtureStabilityData_points);
	return true;
}

void energyTerm::getStabilityTerm() //TODO last time coding to here!
{
	//pre: get the stability of basic and basicFinal
	std::map<std::string, std::array<double, 3>> basicStability;
	std::map<std::string, std::array<double, 3>> basicFinalStability;

	const auto& basicComp = this->getBasicLevelComp();
	const auto& basicFinalComp = this->getBasicLevelComponentFinal();

	for (const auto& level : basicComp) {
		std::array<double, 3> stability = { 0.0, 0.0, 0.0 };
		for (const auto& comp : level.second) {
			const auto& compStability = comp->stability();
			stability[0] += compStability[0];
			stability[1] += compStability[1];
			stability[2] += compStability[2];
		}
		stability[0] /= level.second.size();
		stability[1] /= level.second.size();
		stability[2] /= level.second.size();
		basicStability[level.first] = stability;
	}

	for (const auto& level : basicFinalComp) {
		std::array<double, 3> stability = { 0.0, 0.0, 0.0 };
		for (const auto& comp : level.second) {
			const auto& compStability = comp->stability();
			stability[0] += compStability[0];
			stability[1] += compStability[1];
			stability[2] += compStability[2];
		}
		stability[0] /= level.second.size();
		stability[1] /= level.second.size();
		stability[2] /= level.second.size();
		basicFinalStability[level.first] = stability;
	}

	//get stability values of component
	const auto& components = this->getPset()->components();
	for (const auto& comp : components) {
		const auto& compStability = comp->stability();
		for (const auto& level : this->basicLevelComponent_) {
			std::array<double, 2> compStabilityEnergy = { 0.0,0.0 };
			//distance
			compStabilityEnergy[0] = std::sqrt(
				std::pow(compStability[0] - basicStability[level.first][0], 2) +
				std::pow(compStability[1] - basicStability[level.first][1], 2) +
				std::pow(compStability[2] - basicStability[level.first][2], 2)
			);
			compStabilityEnergy[1] = std::sqrt(
				std::pow(compStability[0] - basicFinalStability[level.first][0], 2) +
				std::pow(compStability[1] - basicFinalStability[level.first][1], 2) +
				std::pow(compStability[2] - basicFinalStability[level.first][2], 2)
			);

			comp->setStabilityEnergy(level.first, compStabilityEnergy);
		}
	}
}

void energyTerm::getStabilityTerm(bool isCura)
{
	//pre: get the stability of basic and basicFinal
	std::map<std::string, double> basicStability;
	std::map<std::string, double> basicFinalStability;

	const auto& basicComp = this->getBasicLevelComp();
	const auto& basicFinalComp = this->getBasicLevelComponentFinal();

	for (const auto& level : basicComp) {
		double stability = 0.0;
		for (const auto& comp : level.second) {
			stability += comp->stability(true);
		}
		stability /= level.second.size();
		basicStability[level.first] = stability;
	}

	for (const auto& level : basicFinalComp) {
		double stability = 0.0;
		for (const auto& comp : level.second) {
			stability += comp->stability(true);
		}
		stability /= level.second.size();
		basicFinalStability[level.first] = stability;
	}

	//get stability values of component
	const auto& components = this->getPset()->components();
	for (const auto& comp : components) {
		const auto& compStability = comp->stability(true);
		for (const auto& level : this->basicLevelComponent_) {
			std::array<double, 2> compStabilityEnergy = { 0.0,0.0 };
			//distance
			compStabilityEnergy[0] = basicStability[level.first] - compStability;
			compStabilityEnergy[1] = basicFinalStability[level.first] - compStability;

			comp->setStabilityEnergy(level.first, compStabilityEnergy);
		}
	}
}

void energyTerm::getdisFidelity()
{
	//const auto& basicComponent = this->basicLevelComponent_;
	//const auto& basicFinalComponent = this->basicLevelComponentFinal_;

	//const auto& components = this->getPset()->components();
	//for (const auto& comp : components) {
	//	for (const auto& level : basicComponent) {
	//		double disFidelity = 0.0;
	//		for (const auto& basicComp : level.second) {
	//			disFidelity += comp->distanceFidelity(basicComp);
	//		}
	//		disFidelity /= level.second.size();
	//		comp->setDisFidelity(level.first, disFidelity);
	//	}
	//}
}

void energyTerm::process2OPerator(const std::vector<double> stabvalue, const std::vector<double> similarValue)
{
	std::map<int, std::vector<int>> cluster;
	const auto& stableNormalize = this->normalized(stabvalue);
	const auto& similarityNormalize = this->normalized(similarValue);

	if (stableNormalize.size() != similarityNormalize.size()) {
		std::cerr << "Stable and Similarity size not match!" << std::endl;
		exit(-1);
	}
	int N = stableNormalize.size();
	for (int i = 0; i < N; i++) {
		double stable = stableNormalize[i];
		double similarity = similarityNormalize[i];
		if (stable > 0.5 && similarity > 0.5) {
			cluster[1].push_back(i);
		}
		else if (stable < 0.5 && similarity > 0.5) {
			cluster[2].push_back(i);
		}
		else if (stable > 0.5 && similarity < 0.5) {
			cluster[3].push_back(i);
		}
		else {
			cluster[4].push_back(i);
		}
	}
	std::cout << "1-4size: " << cluster[1].size() << " " << cluster[2].size() << " " << cluster[2].size() << " " << cluster[4].size() << " " << std::endl;
	this->getPset()->savePointcloudsColorByLabel(cluster, "stableColorMap.ply", 0);
	for (const auto& level : cluster) {
		this->getPset()->saveIndices2Cloud(level.second, "stableColorMap_" + std::to_string(level.first) + ".ply");
	}

	//save normalize to local
	std::string stableNormalizePath = baseFilePath_ + "stableNormalize.txt";
	std::ofstream stableNormalizeFile(stableNormalizePath);
	if (!stableNormalizeFile.is_open()) {
		throw std::runtime_error("stableNormalize file not found!");
	}
	for (const auto& stable : stableNormalize) {
		stableNormalizeFile << std::fixed << std::setprecision(7) << stable << std::endl;
	}
	std::string similarityNormalizePath = baseFilePath_ + "similarityNormalize.txt";
	std::ofstream similarityNormalizeFile(similarityNormalizePath);
	if (!similarityNormalizeFile.is_open()) {
		throw std::runtime_error("similarityNormalize file not found!");
	}
	for (const auto& stable : similarityNormalize) {
		similarityNormalizeFile << std::fixed << std::setprecision(7) << stable << std::endl;
	}
}

std::vector<double> energyTerm::readScale(const std::string scalePath)
{
	std::vector<double> scaleValues;
	//read scale from file
	std::ifstream scaleFile(scalePath);
	if (!scaleFile.is_open()) {
		throw std::runtime_error("Scale file not found!");
	}
	//read line
	std::string line;
	if (std::getline(scaleFile, line)) {
		std::istringstream iss(line);
		double scale;
		while (iss >> scale) {
			scaleValues.push_back(scale);
		}
	}
	scaleFile.close();
	return scaleValues;
}

std::vector<std::vector<variation>> energyTerm::readVariation(const std::string variationPath)
{
	std::vector<std::vector<variation>> variations;
	//read feature from file
	std::ifstream featureFile(variationPath);
	if (!featureFile.is_open()) {
		throw std::runtime_error("Feature file not found!");
	}
	//read line
	int pointNum = 0;
	int scaleNum = 0;
	const int scaleFeatureIdx = 5;//each point have {tau,eta0, eat1, eat2, kappa} features in every scale
	std::string line;
	if (std::getline(featureFile, line)) {
		std::istringstream iss(line);
		iss >> pointNum >> scaleNum;
	}
	if (pointNum <= 0 || scaleNum <= 0) {
		throw std::runtime_error("Feature file have nothing!");
	}
	int nosiyPoint = 0;
	while (std::getline(featureFile, line)) {
		std::vector<variation> variationVector;
		double tau, eta0, eta1, eta2, kappa;
		std::istringstream iss(line);
		for (int iter = 0; iter < scaleNum; iter++) {
			iss >> tau >> eta0 >> eta1 >> eta2 >> kappa;
			if (/*nx == 0 && ny == 0 && nz == 0 && k1 == 0 && k2 == 0*/0) {//TODO: need proces the 0 ?
				continue;
			}
			else {
				variationVector.push_back({ tau, {eta0,eta1,eta2},kappa });
			}
		}
		//if (variationVector.empty()) {
		//	nosiyPoint++;
		//	throw std::runtime_error("have nosiy pointLine!!!");
		//	continue;
		//}
		variations.push_back(variationVector);
	}
	featureFile.close();
	return variations;
}

std::vector<std::vector<std::array<double, 5>>> energyTerm::readScaleFeatures(const std::string scaleFeaturePath)
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
			if (/*nx == 0 && ny == 0 && nz == 0 && k1 == 0 && k2 == 0*/0) {//TODO: need proces the 0 ?
				continue;
			}
			else {
				eachPointFeatures.push_back({ nx, ny, nz, k1, k2 });
			}
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

bool energyTerm::saveStable(const std::vector<double>& stableValue, const std::string filename)
{
	std::string savedFilename;
	if (filename == "") {
		savedFilename = baseFilePath_ + "stability.txt";
	}
	else {
		savedFilename = baseFilePath_ + filename;
	}
	std::ofstream stableFile(savedFilename);
	if (!stableFile.is_open()) {
		throw std::runtime_error("stableFile file not found!");
		return false;
	}
	for (const auto& stable : stableValue) {
		stableFile << std::fixed << std::setprecision(7) << stable << std::endl;
	}
	return true;
}

bool energyTerm::saveVariation()
{
	//save deviation to local
	std::string variationPath = baseFilePath_ + "variations_ori.txt";
	std::ofstream variationFile(variationPath);
	if (!variationFile.is_open()) {
		throw std::runtime_error("Variation file not found!");
		return false;
	}
	for (const auto& points : variations_) {
		for (const auto& var : points) {
			variationFile << var << " ";
		}
		variationFile << std::endl;
	}
	variationFile.close();
	return true;
}

/**
 * save the original normal and curvature value without lamda para process to local.
 *
 * \return
 */
bool energyTerm::saveDeviation()
{
	//save deviation to local
	std::string normalDeviationPath = baseFilePath_ + "normalDeviations.txt";
	std::ofstream normalDeviationFile(normalDeviationPath);
	if (!normalDeviationFile.is_open()) {
		throw std::runtime_error("NormalDeviation file not found!");
		return false;
	}
	for (const auto& normalDiva : allNormalDiva_) {
		for (const auto& diva : normalDiva) {
			normalDeviationFile << diva << " ";
		}
		normalDeviationFile << std::endl;
	}
	std::string curvatureDeviationPath = baseFilePath_ + "curvatureDeviations.txt";
	std::ofstream curvatureDeviationFile(curvatureDeviationPath);
	if (!curvatureDeviationFile.is_open()) {
		throw std::runtime_error("CurvatureDeviation file not found!");
		return false;
	}
	for (const auto& curvatureDiva : allCurvatureDiva_) {
		for (const auto& diva : curvatureDiva) {
			curvatureDeviationFile << std::fixed << std::setprecision(7) << diva << " ";
		}
		curvatureDeviationFile << std::endl;
	}
	return true;
}

/**
 * save the energyTerm infomation to local json file.
 * each component{coverage,compact, persistence, the indices in each level, the value of energyConstraint which is the percent of pointNum in levelCluster of component points }
 * , and the component belong to each levelCluster
 *
 * \return true if save successed
 */
bool energyTerm::saveEnergyTerm()
{
	json energyTerm;
	// save basicFinal Component index
	json basicFinalComponent;
	for (const auto& level : this->getBasicLevelComponentFinal()) {
		std::vector<int> componentIndex;
		for (const auto& comp : level.second) {
			componentIndex.push_back(comp->index());
		}
		basicFinalComponent[level.first] = componentIndex;
	}
	energyTerm["basicFinalComponent"] = basicFinalComponent;
	//save toBasicFinal size for optimization
	json toBasicFinalSize;
	for (const auto& level : this->getBasicLevelIndicesFinal()) {
		toBasicFinalSize[level.first] = level.second.size();
	}
	energyTerm["toBasicFinalSize"] = toBasicFinalSize;
	//save component energy
	std::string componentsEnergyPath = baseFilePath_ + componentsEnergyPath_;
	json componentEnergy;
	for (const auto& comp : point_set_->components()) {
		json component;
		component["coverage"] = comp->energyterm().coverage;
		component["compact"] = comp->energyterm().compact;
		component["persistence"] = comp->energyterm().persistence;
		//toBasicFianl
		for (const auto& level : comp->energyterm().toBasicFinal) {
			json toBasicFinal;
			toBasicFinal["persDifference"] = level.second.persDifference;
			//json indices;
			//for (const auto& idx : level.second.supportPoints) {
			//	indices.push_back(idx);
			//}
			int indices = level.second.supportPoints;
			toBasicFinal["basicFinalEnergy"] = indices;
			component["toBasicFinal"][level.first] = toBasicFinal;
		}

		//stabilityValue

		for (const auto& level : comp->energyterm().stability) {
			json stability;
			stability["basicStable"] = level.second[0];
			stability["basicFinalStable"] = level.second[1];
			component["stability"][level.first] = stability;
		}

		//json levelIndices;
		//for (const auto& level : comp->attri().levelIndices) {
		//	json indices;
		//	for (const auto& idx : level.second) {
		//		indices.push_back(idx);
		//	}
		//	levelIndices[level.first] = indices;
		//}
		//component["levelIndices"] = levelIndices;
		json energyConstraintTerm;
		for (const auto& level : comp->attri().energyConstraintTerm) {
			energyConstraintTerm[level.first] = level.second;
		}
		component["energyConstraintTerm"] = energyConstraintTerm;

		componentEnergy[std::to_string(comp->index())] = component;
	}
	energyTerm["componentsEnergy"] = componentEnergy;
	std::ofstream file(componentsEnergyPath);
	if (file.is_open()) {
		// Write the JSON data to disk.
		file << energyTerm.dump(4); // dump(4) for format output
		file.close();

		//logger->info("componentEnergy.json already save to :{}", componentsEnergyPath);
	}
	else {
		//logger->error("error when open json file: {}", componentsEnergyPath);
		return false;
	}

	//save level corresponding component
	const std::string levelComponentsPath = baseFilePath_ + levelComponentsPath_;
	json levelComponents;
	for (const auto& lc : this->getLevelComponents()) {
		std::vector<int> componentIndex;
		for (const auto& comp : lc.second) {
			componentIndex.push_back(comp->index());
		}
		levelComponents[lc.first] = componentIndex;
	}
	std::ofstream LCfile(levelComponentsPath);
	if (LCfile.is_open()) {
		// Write the JSON data to disk.
		LCfile << levelComponents.dump(4); // dump(4) for format output
		LCfile.close();

		//logger->info("levelComponents.json already save to : {}", levelComponentsPath);
	}
	else {
		//logger->error("error when open json file: {}", levelComponentsPath);
		return false;
	}

	return true;
}

bool energyTerm::readStabilityValues(const std::string featureFilePath)
{
	if (!std::filesystem::exists(featureFilePath)) {
		throw std::runtime_error("local StabilityValues file not found!");
		return false;
	}
	std::ifstream stabilityFile(featureFilePath);
	if (!stabilityFile.is_open()) {
		throw std::runtime_error("StabilityValues file not found!");
		return false;
	}
	std::string line;
	while (std::getline(stabilityFile, line)) {
		std::istringstream iss(line);
		std::array<double, 2> stabilityValue;
		iss >> stabilityValue[0] >> stabilityValue[1];
		stabilityValues.push_back(stabilityValue);
	}
	return true;
}

bool energyTerm::readClusterResult(const std::string clusterResultPath)
{
	if (!std::filesystem::exists(clusterResultPath)) {
		//logger->error("local ClusterResult file not found!");
		return false;
	}
	std::ifstream clusterResultFile(clusterResultPath);
	if (!clusterResultFile.is_open()) {
		//logger->error("ClusterResult file cannot open!");
		return false;
	}
	std::string dump;
	int clusertNum = 0;
	std::string line;
	std::getline(clusterResultFile, line);
	std::istringstream iss(line);
	iss >> dump >> clusertNum;
	if (clusertNum < 1) {
		//logger->error("Cluster number is invalid!");
		return false;
	}
	for (int i = 0; i < 2 * clusertNum; i++) {
		std::getline(clusterResultFile, line);
		//TODO: discard cluster info for now
	}
	int pointIdx = 0;
	std::map<std::string, std::vector<int>> clusterResultTmp;
	while (std::getline(clusterResultFile, line)) {
		std::istringstream iss(line);
		std::string label;
		iss >> label;
		clusterResultTmp[label].push_back(pointIdx);
		pointIdx++;
	}
	// merge low level to high level

	clusterResult["0"].insert(clusterResult["0"].end(), clusterResultTmp["0"].begin(), clusterResultTmp["0"].end());

	//clusterResult["1"].insert(clusterResult["1"].end(), clusterResultTmp["0"].begin(), clusterResultTmp["0"].end());
	clusterResult["1"].insert(clusterResult["1"].end(), clusterResultTmp["1"].begin(), clusterResultTmp["1"].end());

	//clusterResult["2"].insert(clusterResult["2"].end(), clusterResultTmp["0"].begin(), clusterResultTmp["0"].end());
	//clusterResult["2"].insert(clusterResult["2"].end(), clusterResultTmp["1"].begin(), clusterResultTmp["1"].end());
	clusterResult["2"].insert(clusterResult["2"].end(), clusterResultTmp["2"].begin(), clusterResultTmp["2"].end());

	//logger->info("ClusterResult read done!");
	for (const auto& cluster : clusterResult) {
		//logger->info("Cluster {} size: {} ", cluster.first, cluster.second.size());
		this->getPset()->saveIndices2Cloud(cluster.second, "levelCluster_" + cluster.first + ".ply");
	}

	//logger->info("read cluster result from {} done", clusterResultPath);
	return true;
}

bool energyTerm::readEnergyData(const std::string energyDataPath)
{
	std::ifstream componentsEnergyFile(energyDataPath);
	if (!componentsEnergyFile.is_open()) {
		throw std::runtime_error("ComponentsEnergyData file not found!");
	}
	int componentSize, componentDim;
	std::string line;
	std::getline(componentsEnergyFile, line);
	std::istringstream iss(line);
	iss >> componentSize >> componentDim;
	std::getline(componentsEnergyFile, line);//{coverage,compact,persistence}--discard
#ifndef NDEBUG
	assert(componentSize == point_set_->components().size());
#endif // !NDEBUG
	for (int i = 0; i < componentSize; i++) {
		std::getline(componentsEnergyFile, line);
		std::istringstream iss(line);
		double coverage, compact, persistence;
		iss >> coverage >> compact >> persistence;
		point_set_->components()[i]->setEnergyTerm(coverage, compact, persistence);
	}
	return true;
}

/**
 * read energy data term and constraint term from local json file.
 *
 * \param energyDataPath
 * \param json. any bool value for overloaded function readEnergyData(){} to read energyterm from json
 * \return
 */
bool energyTerm::readEnergyData(const std::string energyDataPath, bool isJson)
{
	//logger->info("read energyData from {}", energyDataPath);
	if (!std::filesystem::exists(energyDataPath)) {
		throw std::runtime_error("local EnergyData.json file not found!");
		return false;
	}
	std::ifstream jsonFile(energyDataPath);
	if (!jsonFile.is_open()) {
		std::cout << "json file not found!";
		return -1;
	}
	json j = json::parse(jsonFile);
	//judge exits
	if (j.find("toBasicFinalSize") == j.end()) {
		throw std::runtime_error("toBasicFinalSize not found in json file!");
		return false;
	}
	std::map<std::string, int> Size;
	for (const auto& level : j["toBasicFinalSize"].items()) {
		Size[level.key()] = level.value();
	}
	this->setBasicFinalSize(Size);

	//read basicFinalComponent
	if (j.find("basicFinalComponent") == j.end()) {
		throw std::runtime_error("basicFinalComponent not found in json file!");
		return false;
	}
	std::map<std::string, std::vector<Component::Ptr>> basicFinalComponent;
	for (const auto& level : j["basicFinalComponent"].items()) {
		std::vector<Component::Ptr> components;
		for (const auto& compidx : level.value()) {
			auto& comp = point_set_->components()[compidx];
			if (comp->index() == compidx) {
				components.push_back(comp);
			}
			else {
				//logger->error("json;s component idx != comp->idx");
				exit(-1);
			}
		}

		basicFinalComponent[level.key()] = components;
	}
	this->setBasicLevelComponentFinal(basicFinalComponent);

	//read energy data term
	j = j["componentsEnergy"];
	for (const auto& comp : point_set_->components()) {
		std::map<std::string, toBasicFinalEnergy> toBasicFinal;
		for (const auto& level : j[std::to_string(comp->index())]["toBasicFinal"].items()) {
			toBasicFinal[level.key()] = { level.value()["persDifference"], level.value()["basicFinalEnergy"] };
		}
		comp->setEnergyTerm(j[std::to_string(comp->index())]["coverage"], j[std::to_string(comp->index())]["compact"], j[std::to_string(comp->index())]["persistence"], toBasicFinal);
		std::map<std::string, double> energyConstraintTerm;
		for (const auto& level : j[std::to_string(comp->index())]["energyConstraintTerm"].items()) {
			energyConstraintTerm[level.key()] = level.value();
		}
		//std::map<std::string, std::vector<int>> levelIndices;
		//for (const auto& level : j[std::to_string(comp->index())]["levelIndices"].items()) {
		//	std::vector<int> indices;
		//	for (const auto i : level.value()) {
		//		indices.push_back(/*std::atoi(std::string(i).c_str())*/i);
		//	}
		//	levelIndices[level.key()] = indices;
		//}
		//comp->setLevelIndices(levelIndices);
		comp->setEnergyConstraintTerm(energyConstraintTerm);

		//stability
		std::map<std::string, std::array<double, 2>> stability;
		for (const auto& level : j[std::to_string(comp->index())]["stability"].items()) {
			stability[level.key()] = { level.value()["basicStable"], level.value()["basicFinalStable"] };
		}
		comp->setStability(stability);
	}

	return true;
}

bool energyTerm::readEnergyConstraint(const std::string energyConstraintPath)
{
	std::ifstream componentsEnergyFile(energyConstraintPath);
	if (!componentsEnergyFile.is_open()) {
		throw std::runtime_error("ComponentsEnergyData file not found!");
		return false;
	}
	std::vector<Component::Ptr> components = point_set_->components();
	std::string line;
	std::getline(componentsEnergyFile, line);//{coverage compact,persistence,}--discard
	int componentIndex = 0;
	while (std::getline(componentsEnergyFile, line)) {
		std::istringstream iss(line);
		double coverage, compact, persistence;
		iss >> coverage >> compact >> persistence;
		std::map<std::string, double> energyConstraintTerm;
		std::string levelLabel;
		double value;
		while (iss >> levelLabel >> value) {
			energyConstraintTerm[levelLabel] = value;
		}
		components[componentIndex]->setEnergyTerm(coverage, compact, persistence);
		components[componentIndex]->setEnergyConstraintTerm(energyConstraintTerm);
		componentIndex++;
	}
	return true;
}

bool energyTerm::readLevelComponents(const std::string levelComponentsPath, bool isJson)
{
	//logger->info("read levelComponent from {}", levelComponentsPath);
	if (!std::filesystem::exists(levelComponentsPath)) {
		throw std::runtime_error("local levelComponentsPath.json file not found!");
		return false;
	}
	std::ifstream jsonFile(levelComponentsPath);
	if (!jsonFile.is_open()) {
		std::cout << "json file not found!";
		return -1;
	}
	json j = json::parse(jsonFile);
	std::map<int, Component::Ptr> compsIndex;
	for (const auto& comp : getPset()->components()) {
		compsIndex[comp->index()] = comp;
	}
	std::map < std::string, std::set<Component::Ptr> > levelComponents;
	for (const auto& level : j.items()) {
		std::string levelLabel = level.key();
		for (const auto& indices : level.value()) {
			levelComponents[levelLabel].insert(compsIndex[indices]);
		}
	}
	levelComponents_ = levelComponents;
	return true;
}

bool energyTerm::readComponentLevelsNumber(const std::string componentLevelsNumPath, bool isJson)
{
	if (!std::filesystem::exists(componentLevelsNumPath)) {
		throw std::runtime_error("local componentLevelsNumPath.json file not found!");
		return false;
	}
	json j;
	std::ifstream jsonFile(componentLevelsNumPath);
	if (!jsonFile.is_open()) {
		std::cout << "json file not found!";
		return false;
	}
	j = json::parse(jsonFile);
	for (const auto& component : j.items()) {
		Component::Ptr comp = point_set_->components()[std::atoi(component.key().c_str())];
		std::vector<std::pair<std::string, int>> levelIndices;
		for (const auto& level : component.value().items()) {
			levelIndices.push_back({ level.key(), level.value() });
		}
		compLevelsNumver_[comp] = levelIndices;
	}
	return true;
}

Bbox_3 energyTerm::boundingBox(PointSet* pset)
{
	if (pset->cgal_points().empty()) {
		throw std::runtime_error("Point set is empty!");
	}

	Bbox_3 bbox = pset->cgal_points()[0].bbox();
	for (const auto& point : pset->cgal_points()) {
		bbox = bbox + point.bbox();
	}
	return bbox;
}

double energyTerm::normalAngle(const double n1_x, const double n1_y, const double n1_z, const double n2_x, const double n2_y, const double n2_z)
{
	double dot = n1_x * n2_x + n1_y * n2_y + n1_z * n2_z;

	/** !!hypothesis the normal was normalized!! */
	double magnitude_1 = std::sqrt(n1_x * n1_x + n1_y * n1_y + n1_z * n1_z);
	double magnitude_2 = std::sqrt(n2_x * n2_x + n2_y * n2_y + n2_z * n2_z);

	if (magnitude_1 == 0 || magnitude_2 == 0) {
		return M_PI;
		//throw std::runtime_error("normal vector is zero!");
	}
	double cosTheta = dot / (magnitude_1 * magnitude_2);
	// Clamp the cosine value to avoid floating-point overflow outside [-1, 1].
	if (cosTheta > 1.0) cosTheta = 1.0;
	if (cosTheta < -1.0) cosTheta = -1.0;

	return std::acos(cosTheta);
}

double energyTerm::curvatureChange(const double A_k1, const double A_k2, const double B_k1, const double B_k2)
{
#if 0
	/** using  Gaussian Curvature*/
	double A_GC = A_k1 * A_k2;
	double B_GC = B_k1 * B_k2;
#endif
	/** using Average Curvature */
	double A_GC = (A_k1 + A_k2) / 2.0;
	double B_GC = (B_k1 * B_k2) / 2.0;
	return B_GC - A_GC;
}

double energyTerm::getAvgPersisitence(const std::vector<Component::Ptr> comps)
{
#if 0
	{
		std::vector<int> persisitences;
		for (const auto& comp : comps) {
			persisitences.push_back(comp->attri().persistence);
		}
		std::sort(persisitences.begin(), persisitences.end());
		std::cout << "each persiistence:";
		for (auto pers : persisitences) {
			std::cout << pers << " ";
		}
		std::cout << std::endl;
	}
#endif
	int totalPers = 0;
	//openmp
#pragma omp parallel for reduction(+:totalPers)
	for (const auto& comp : comps) {
		int pers = comp->attri().persistence;
		totalPers += pers;
	}
	return totalPers / static_cast<double>(comps.size());
}

/**
 * .
 *
 * \param comps
 * \return
 */
double energyTerm::getMedianPersisitence(const std::vector<Component::Ptr> comps)
{
	if (comps.empty()) {
		throw std::invalid_argument("Components is empty!");
	}

	std::vector<int> persisitences;
	for (const auto& comp : comps) {
		persisitences.push_back(comp->attri().persistence);
	}
	std::sort(persisitences.begin(), persisitences.end());
	int size = persisitences.size();
	if (size % 2 == 0) {
		return (persisitences[size / 2 - 1] + persisitences[size / 2]) / 2.0;
	}
	else {
		return persisitences[size / 2];
	}
}

double energyTerm::getAllPointsPersistence(const std::vector<Component::Ptr> comps)
{
	int totalPers = 0;
	for (const auto& comp : comps) {
		totalPers += comp->attri().persistence * comp->size();
	}
	return totalPers / static_cast<double>(comps.size());
}

/**
 * select the function of how to calculate the persThreshold.
 *
 * \param comps
 * \param func: 'm' for median, 'a' for average
 * \return
 */
double energyTerm::getThresholdPers(const std::vector<Component::Ptr> comps, char func)
{
	switch (func)
	{
	case 'm':
		return getMedianPersisitence(comps);
		break;
	case 'a':
		return getAvgPersisitence(comps);
		break;
	case 'p':
		return getAllPointsPersistence(comps);
		break;
	default:
		std::cerr << "Invalid function type!" << std::endl;
		return 0.0;
		break;
	}
}

void energyTerm::disFidelity(const Component::Ptr comp, const std::map<std::string, std::vector<Component::Ptr>> basic)
{
	//calculate the avg dis from comp to each level basic
}

void energyTerm::saveStableComponentOfBasic()
{
	//logger->debug("saveStableComponentOfBasic begin...");
	std::map < std::string, std::array < std::map<double, std::vector<Component::Ptr>>, 2 > > compStaility;//{level,{basic/basicFinal,{stability,{comp::ptr}}}}
	const auto& comps = this->getPset()->components();
	for (const auto& comp : comps) {
		for (const auto& level : this->basicLevelComponent_) {
			double stability = std::abs(comp->attri().energyterm.stability[level.first][0]);
			compStaility[level.first][0][stability].push_back(comp);
		}
		for (const auto& level : this->basicLevelComponentFinal_) {
			double stability = std::abs(comp->attri().energyterm.stability[level.first][1]);
			compStaility[level.first][1][stability].push_back(comp);
		}
	}

	int numThreshold = 10;
	for (const auto& level : compStaility) {
		int count = 0;
		for (const auto comps : level.second[1]) {
			//if (count == 100) {
			//	break;
			//}
			for (const auto& comp : comps.second) {
				std::cout << "level: " << level.first << " basicFinal: " << comps.first << " comp: " << comp->index() << std::endl;
				this->getPset()->saveIndices2Cloud(comp->indices(), "/checkbasic/basicFinal/level" + level.first + "_" + std::to_string(comp->index()) + std::to_string(comps.first) + ".ply");
			}
			count++;
		}
	}
	//logger->debug("saveStableComponentOfBasic done...");
}

bool energyTerm::saveCompLevelsNumber(const std::map<Component::Ptr, std::vector<std::pair<std::string, int>>>& compLevelsNumver)
{
	std::string compLevelsNumberPath = baseFilePath_ + "_4-1_compLevelsNumber.json";
	json compLevelsNumber;
	for (const auto& comp : compLevelsNumver) {
		json component;
		for (const auto& level : comp.second) {
			component[level.first] = level.second;
		}
		compLevelsNumber[std::to_string(comp.first->index())] = component;
	}
	std::ofstream file(compLevelsNumberPath);
	if (file.is_open()) {
		// Write the JSON data to disk.
		file << compLevelsNumber.dump(4); // dump(4) for format output
		file.close();
		std::cout << "compLevelsNumber.json already save to : " << compLevelsNumberPath << std::endl;
	}
	else {
		std::cerr << "error when open json file: " << compLevelsNumberPath << std::endl;
		return false;
	}
	return true;
}

bool energyTerm::savePersAnalysis()
{
	std::string persAnalysisPath = baseFilePath_ + "_4-3_persAnalysis.json";
	json persAnalysis;
	json basic;
	json basicFinal;
	for (const auto& level : this->getBasicLevelComp()) {
		json levelPers;
		for (const auto& comp : level.second) {
			levelPers[std::to_string(comp->index())] = { comp->attri().persistence ,comp->attri().birth,comp->attri().death };
		}
		basic[level.first] = levelPers;
	}
	for (const auto& level : this->getBasicLevelComponentFinal()) {
		json levelPers;
		for (const auto& comp : level.second) {
			levelPers[std::to_string(comp->index())] = { comp->attri().persistence ,comp->attri().birth,comp->attri().death };
		}
		basicFinal[level.first] = levelPers;
	}
	persAnalysis["basic"] = basic;
	persAnalysis["basicFinal"] = basicFinal;
	std::ofstream file(persAnalysisPath);
	if (file.is_open()) {
		// Write the JSON data to disk.
		file << persAnalysis.dump(4); // dump(4) for format output
		file.close();
		//logger->info("persAnalysis.json already save to : {}", persAnalysisPath);
	}
	else {
		//logger->error("error when open json file: {}", persAnalysisPath);
		return false;
	}
	return true;
}

double energyTerm::DoN(const double n1_x, const double n1_y, const double n1_z, const double n2_x, const double n2_y, const double n2_z)
{
	Eigen::Vector3d n1(n1_x, n1_y, n1_z);
	Eigen::Vector3d n2(n2_x, n2_y, n2_z);
	Eigen::Vector3d DoN = n2 - n1;
	double DoN_norm = DoN.norm();
	return DoN_norm;
}
