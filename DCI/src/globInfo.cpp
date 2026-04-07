#include "globInfo.h"
#include <fstream>

namespace {
std::string get_required_string(const nlohmann::json& object, const char* key)
{
	const auto it = object.find(key);
	if (it == object.end() || !it->is_string() || it->get<std::string>().empty()) {
		throw std::runtime_error(std::string("Missing required string field: ") + key);
	}
	return it->get<std::string>();
}

std::string get_optional_string(const nlohmann::json& object, const char* key, const std::string& default_value = {})
{
	const auto it = object.find(key);
	if (it == object.end() || it->is_null()) {
		return default_value;
	}
	if (!it->is_string()) {
		throw std::runtime_error(std::string("Expected string field: ") + key);
	}
	return it->get<std::string>();
}
}

bool GlobInfo::parse(int argc, char* argv[])
{
	using json = nlohmann::json;
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " <config.json>" << std::endl;
		return false;
	}

	const std::string config_path = argv[1];
	std::ifstream jsonFile(config_path);
	if (!jsonFile.is_open()) {
		std::cerr << "Config file not found: " << config_path << std::endl;
		return false;
	}
	try
	{
		json j = json::parse(jsonFile);
		const auto& inputPath = j.at("inputPath");
		const auto filePathIt = j.find("filePath");
		const auto parametersIt = j.find("parameters");
		const auto lambdaIt = j.find("lambda");
		const json empty_object = json::object();
		const auto& filePath = (filePathIt != j.end() && filePathIt->is_object()) ? *filePathIt : empty_object;
		const auto& parameters = (parametersIt != j.end() && parametersIt->is_object()) ? *parametersIt : empty_object;
		const auto& lambda = (lambdaIt != j.end() && lambdaIt->is_object()) ? *lambdaIt : empty_object;

		// Input file paths required by the public DCI pipeline.
		pointcloudsFilePath = get_required_string(inputPath, "pointcloudsPath");
		componentFilePath = get_required_string(inputPath, "componentPath");
		segmentsFilePath = get_required_string(inputPath, "segmentsPath");
		variationPath = get_required_string(inputPath, "variationPath");
		scalePath = get_required_string(inputPath, "scalePath");

		// Legacy or extended-pipeline fields kept optional for compatibility.
		featuresFilePath = get_optional_string(inputPath, "featuresPath");
		clustersFilePath = get_optional_string(inputPath, "clustersPath");
		stabilityFilePath = get_optional_string(inputPath, "stabilityPath");
		pointsLodFilePath = get_optional_string(inputPath, "pointsLod");

		energyDataTermPath = get_optional_string(filePath, "energyDataTermPath");
		stabilityValuesPath = get_optional_string(filePath, "stabilityValuesPath");
		clusterResultPath = get_optional_string(filePath, "clusterResultPath");
		normalDeviationsPath = get_optional_string(filePath, "normalDeviationsPath");
		curvatureDeviationPath = get_optional_string(filePath, "curvatureDeviationsPath");
		componentsEnergyPath = get_optional_string(filePath, "componentsEnergyPath");
		levelComponentsPath = get_optional_string(filePath, "levelComponents");
		componentLevelsNumberPath = get_optional_string(filePath, "componentLevelsNumber");

		lamda_normal = parameters.value("lamda_normal", 1.0);
		lamda_curvature = parameters.value("lamda_curvature", 0.5);
		clusterNumber = parameters.value("clusterNumber", 4);
		basicThreshold = parameters.value("basicThreshold", 0.6);
		componentThreshold = parameters.value("componentThreshold", 200);

		lambda_coverage = lambda.value("lambda_coverage", 1.0);
		lambda_compact = lambda.value("lambda_compact", 1.0);
		lambda_persistence = lambda.value("lambda_persistence", 1.0);
		lambda_level_0 = lambda.value("lambda_level_0", 0.0);
		lambda_level_1 = lambda.value("lambda_level_1", 0.0);
		lambda_level_2 = lambda.value("lambda_level_2", 1.0);

		processLevel = get_optional_string(j, "processLevel", "0");
	}
	catch (const std::exception& e)
	{
		std::cerr << "Config parsing error: " << e.what() << std::endl;
		return false;
	}
	return true;
}
