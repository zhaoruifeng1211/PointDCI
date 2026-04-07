#include "core/PointCloud.h"
#include "core/KDTree.h"
#include "core/FPSSampler.h"
#include "io/PlyIO.h"
#include "io/ObjIO.h"
#include "weighting/WeightStrategy.h"
#include "weighting/ColorIntensityWeight.h"
#include "weighting/CurvatureWeight.h"
#include "weighting/DensityWeight.h"
#include "weighting/PercentileGatedWeight.h"
#include "weighting/UniformWeight.h"
#include "weighting/ConsistencyStabilityWeight.h"
#include "weighting/ConsistencyWeight.h"
#include "weighting/StabilityWeight.h"
#include "downsampling/Downsampler.h"
#include "downsampling/OctreeDownsampler.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <cstdlib>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <cstdint>
#include <cxxopts.hpp>

// 强制引用各个策略，触发静态注册
namespace ForceRef {
    static WeightStrategy* weightRefs[] = {
        new ColorIntensityWeight(),
        new CurvatureWeight(),
        new DensityWeight(),
        new UniformWeight(),
        new ConsistencyStabilityWeight(),
        new ConsistencyWeight(),
        new StabilityWeight()
    };
    static Downsampler* downsamplerRefs[] = {
        new OctreeDownsampler()
    };
}

namespace {

std::string deriveSidecarFilename(const std::string& outputFile,
                                  const std::string& suffix,
                                  const std::string& defaultBaseName) {
    if (!outputFile.empty() && outputFile != "output.ply") {
        size_t dotPos = outputFile.rfind('.');
        if (dotPos != std::string::npos) {
            return outputFile.substr(0, dotPos) + suffix + outputFile.substr(dotPos);
        }
        return outputFile + suffix + ".ply";
    }
    return defaultBaseName + suffix + ".ply";
}

std::vector<size_t> collectPositiveMaskIndices(const std::vector<double>& mask) {
    std::vector<size_t> indices;
    indices.reserve(mask.size());
    for (size_t i = 0; i < mask.size(); ++i) {
        if (mask[i] > 0.0) {
            indices.push_back(i);
        }
    }
    return indices;
}

struct PercentRange {
    bool enabled = false;
    double start = 0.0;
    double end = 1.0;
    std::string label;
};

PercentRange parsePercentRange(const std::string& percentArg) {
    PercentRange range;
    if (percentArg.empty()) {
        return range;
    }

    const size_t dashPos = percentArg.find('-');
    if (dashPos == std::string::npos || dashPos == 0 || dashPos == percentArg.size() - 1) {
        throw std::runtime_error("percent must be provided as start-end, for example 0-0.3");
    }

    range.start = std::stod(percentArg.substr(0, dashPos));
    range.end = std::stod(percentArg.substr(dashPos + 1));
    range.enabled = true;
    range.label = percentArg;
    return range;
}

void assignRedLowBlueHighColors(PointCloud& cloud, const std::vector<double>& scores) {
    if (scores.size() != cloud.size() || scores.empty()) {
        return;
    }

    auto [minIt, maxIt] = std::minmax_element(scores.begin(), scores.end());
    const double minScore = *minIt;
    const double maxScore = *maxIt;
    const double range = maxScore - minScore;

    cloud.red.resize(cloud.size());
    cloud.green.resize(cloud.size());
    cloud.blue.resize(cloud.size());

    for (size_t i = 0; i < cloud.size(); ++i) {
        double t = 0.5;
        if (range > 1e-12) {
            t = (scores[i] - minScore) / range;
        }

        cloud.red[i] = static_cast<std::uint8_t>(255.0 * (1.0 - t));
        cloud.green[i] = 0;
        cloud.blue[i] = static_cast<std::uint8_t>(255.0 * t);
    }
}

void assignBlueLowRedHighColors(PointCloud& cloud, const std::vector<double>& scores) {
    if (scores.size() != cloud.size() || scores.empty()) {
        return;
    }

    auto [minIt, maxIt] = std::minmax_element(scores.begin(), scores.end());
    const double minScore = *minIt;
    const double maxScore = *maxIt;
    const double range = maxScore - minScore;

    cloud.red.resize(cloud.size());
    cloud.green.resize(cloud.size());
    cloud.blue.resize(cloud.size());

    for (size_t i = 0; i < cloud.size(); ++i) {
        double t = 0.5;
        if (range > 1e-12) {
            t = (scores[i] - minScore) / range;
        }

        cloud.red[i] = static_cast<std::uint8_t>(255.0 * t);
        cloud.green[i] = 0;
        cloud.blue[i] = static_cast<std::uint8_t>(255.0 * (1.0 - t));
    }
}

void assignStrategyColors(PointCloud& cloud,
                          const std::string& strategyName,
                          const std::vector<double>& scores) {
    if (strategyName == "consistency") {
        assignBlueLowRedHighColors(cloud, scores);
        return;
    }
    assignRedLowBlueHighColors(cloud, scores);
}

void exportColoredSidecar(const PointCloud& sourceCloud,
                          const std::vector<double>& scores,
                          const std::string& strategyName,
                          const std::string& outputFile,
                          const std::string& suffix,
                          const std::string& defaultBaseName,
                          bool asBinary,
                          const std::string& label) {
    PointCloud coloredCloud = sourceCloud;
    assignStrategyColors(coloredCloud, strategyName, scores);
    const std::string filename = deriveSidecarFilename(outputFile, suffix, defaultBaseName);
    std::cout << "Exporting " << label << " to " << filename << "...\n";
    PlyIO::write(coloredCloud, filename, asBinary);
}

bool isColorMappedStrategy(const std::string& strategyName, const PointCloud& cloud) {
    if (strategyName == "curvature") {
        return true;
    }
    if (strategyName == "color-intensity") {
        return cloud.hasIntensity();
    }
    if (strategyName == "consistency") {
        return cloud.hasConsistency();
    }
    if (strategyName == "stability") {
        return cloud.hasStability();
    }
    return false;
}

std::vector<double> computeColorScores(const std::string& strategyName,
                                       const PointCloud& cloud,
                                       const KDTree& tree,
                                       int k) {
    if (strategyName == "curvature") {
        CurvatureWeight curvature(k);
        return curvature.compute(cloud, tree);
    }
    if (strategyName == "color-intensity" && cloud.hasIntensity()) {
        return cloud.intensity;
    }
    if (strategyName == "consistency" && cloud.hasConsistency()) {
        return cloud.consistency;
    }
    if (strategyName == "stability" && cloud.hasStability()) {
        return cloud.stability;
    }
    return {};
}

std::string fullCloudColorSuffix(const std::string& strategyName) {
    if (strategyName == "curvature") return "_curvature_full";
    if (strategyName == "color-intensity") return "_intensity_full";
    if (strategyName == "consistency") return "_consistency_full";
    if (strategyName == "stability") return "_stability_full";
    return "";
}

std::string fullCloudColorLabel(const std::string& strategyName) {
    if (strategyName == "curvature") return "full curvature-colored cloud";
    if (strategyName == "color-intensity") return "full intensity-colored cloud";
    if (strategyName == "consistency") return "full consistency-colored cloud";
    if (strategyName == "stability") return "full stability-colored cloud";
    return "full colored cloud";
}

}  // namespace

void printHelp(const cxxopts::Options& opts) {
    std::cout << "Usage: pointSample [options]\n\n";
    std::cout << "Available weighting strategies:\n";
    for (const auto& name : WeightRegistry::instance().list()) {
        auto creator = WeightRegistry::instance().get(name);
        if (creator) {
            auto s = creator();
            std::cout << "  " << std::left << std::setw(24) << name
                      << s->description() << "\n";
        }
    }
    std::cout << "\n";
    std::cout << "Available downsampling methods:\n";
    for (const auto& name : DownsamplerRegistry::instance().list()) {
        auto creator = DownsamplerRegistry::instance().get(name);
        if (creator) {
            auto d = creator();
            std::cout << "  " << std::left << std::setw(24) << name
                      << d->description() << "\n";
        }
    }
    std::cout << "\n";
    std::cout << opts.help() << "\n";
}

int main(int argc, char** argv) {
    try {
        cxxopts::Options opts("pointSample", "Weighted Farthest Point Sampling for point clouds");

        opts.add_options()
            ("i,input",         "Input point cloud or mesh file",      cxxopts::value<std::string>())
            ("o,output",        "Output PLY file",                      cxxopts::value<std::string>()->default_value("output.ply"))
            ("n,num",           "Number of points to sample",           cxxopts::value<size_t>()->default_value("4096"))
            ("k",               "K for KNN (curvature/density)",       cxxopts::value<int>()->default_value("20"))
            ("s,strategy",      "Weighting strategy (see below)",       cxxopts::value<std::string>()->default_value("uniform"))
            ("a,alpha",         "Weight intensity (0 = FPS only)",      cxxopts::value<double>()->default_value("1.0"))
            ("percent",         "Percentile range for gating, e.g. 0-0.3 or 0.3-0.6",
                                cxxopts::value<std::string>()->default_value(""))
            ("r,radius",        "Radius for density estimation",        cxxopts::value<double>()->default_value("0.1"))
            ("flip-yz",          "Flip Y and Z coordinates",             cxxopts::value<bool>()->default_value("false"))
            ("binary",          "Write output as binary PLY",           cxxopts::value<bool>()->default_value("true"))
            ("colorMesh",       "Interpret input as OBJ mesh vertices with RGB and use color-intensity workflow",
                                cxxopts::value<bool>()->default_value("false"))

            // 降采样参数
            ("d,downsample",    "Downsampling method (octree/none)",   cxxopts::value<std::string>()->default_value("none"))
            ("l,octree-level",  "Octree subdivision level (6-12)",     cxxopts::value<int>()->default_value("6"))
            ("m,octree-mode",   "Octree selection mode (center/first)",cxxopts::value<std::string>()->default_value("center"))

            // 权重数据文件
            ("c,consistency",   "Consistency values file (txt)",       cxxopts::value<std::string>()->default_value(""))
            ("t,stability",     "Stability values file (txt)",         cxxopts::value<std::string>()->default_value(""))
            ("g,gamma",         "Gamma for (1+|C-S|)^gamma scoring",   cxxopts::value<double>()->default_value("1.0"))

            ("h,help",          "Show this help",                      cxxopts::value<bool>()->default_value("false"));

        auto parsed = opts.parse(argc, argv);

        if (parsed["help"].as<bool>() || !parsed.count("input")) {
            printHelp(opts);
            return 0;
        }

        std::string inputFile     = parsed["input"].as<std::string>();
        std::string outputFile    = parsed["output"].as<std::string>();
        size_t numTarget          = parsed["num"].as<size_t>();
        int k                     = parsed["k"].as<int>();
        std::string strategyName  = parsed["strategy"].as<std::string>();
        double alpha              = parsed["alpha"].as<double>();
        PercentRange percentRange = parsePercentRange(parsed["percent"].as<std::string>());
        double radius             = parsed["radius"].as<double>();
        bool flipYZ               = parsed["flip-yz"].as<bool>();
        bool asBinary             = parsed["binary"].as<bool>();
        bool colorMesh            = parsed["colorMesh"].as<bool>();

        std::string downsampleMethod = parsed["downsample"].as<std::string>();
        int octreeLevel          = parsed["octree-level"].as<int>();
        std::string octreeMode   = parsed["octree-mode"].as<std::string>();
        std::string consistencyFile = parsed["consistency"].as<std::string>();
        std::string stabilityFile  = parsed["stability"].as<std::string>();
        double gamma             = parsed["gamma"].as<double>();

        if (colorMesh) {
            strategyName = "color-intensity";
            if (downsampleMethod == "none") {
                downsampleMethod = "octree";
            }
        }

        // 参数验证
        if (octreeLevel < 6 || octreeLevel > 12) {
            std::cerr << "Error: octree-level must be between 6 and 12\n";
            return 1;
        }
        if (percentRange.enabled &&
            (percentRange.start < 0.0 || percentRange.end > 1.0 || percentRange.start >= percentRange.end)) {
            std::cerr << "Error: percent range must satisfy 0 <= start < end <= 1\n";
            return 1;
        }
        if (octreeMode != "center" && octreeMode != "first") {
            std::cerr << "Error: octree-mode must be 'center' or 'first'\n";
            return 1;
        }

        // ========== 1. 读取点云 ==========
        std::cout << "Reading " << inputFile << "...\n";
        auto startRead = std::chrono::high_resolution_clock::now();
        PointCloud cloud;
        if (colorMesh) {
            ObjIO::ParseOptions objOpts;
            objOpts.flipYZ = flipYZ;
            cloud = ObjIO::read(inputFile, objOpts);
        } else {
            PlyIO::ParseOptions plyOpts;
            plyOpts.flipYZ = flipYZ;
            cloud = PlyIO::read(inputFile, plyOpts);
        }
        auto endRead = std::chrono::high_resolution_clock::now();

        if (cloud.size() == 0) {
            std::cerr << "Error: Failed to read input file or file has no usable vertices/points\n";
            return 1;
        }

        std::cout << "  Loaded " << cloud.size() << " points"
                  << " (" << std::chrono::duration<double>(endRead - startRead).count() << "s)\n";
        std::cout << "  Has normals: " << (cloud.hasNormals ? "yes" : "no") << "\n";
        if (colorMesh) {
            std::cout << "  Has color intensity: " << (cloud.hasIntensity() ? "yes" : "no") << "\n";
        }

        // ========== 2. 读取权重数据 ==========
        if (!consistencyFile.empty()) {
            std::cout << "Reading consistency from " << consistencyFile << "...\n";
            cloud.consistency = PlyIO::readWeights(consistencyFile);
            if (cloud.consistency.size() != cloud.size()) {
                std::cerr << "Error: Consistency data size (" << cloud.consistency.size()
                          << ") != point count (" << cloud.size() << ")\n";
                return 1;
            }
            std::cout << "  Loaded " << cloud.consistency.size() << " values\n";
        }

        if (!stabilityFile.empty()) {
            std::cout << "Reading stability from " << stabilityFile << "...\n";
            cloud.stability = PlyIO::readWeights(stabilityFile);
            if (cloud.stability.size() != cloud.size()) {
                std::cerr << "Error: Stability data size (" << cloud.stability.size()
                          << ") != point count (" << cloud.size() << ")\n";
                return 1;
            }
            std::cout << "  Loaded " << cloud.stability.size() << " values\n";
        }

        if (isColorMappedStrategy(strategyName, cloud)) {
            std::cout << "Computing " << strategyName << " colors for full cloud...\n";
            auto startFullColor = std::chrono::high_resolution_clock::now();
            KDTree fullTree(cloud);
            const std::vector<double> fullColorScores =
                computeColorScores(strategyName, cloud, fullTree, k);
            auto endFullColor = std::chrono::high_resolution_clock::now();
            std::cout << "  Full-cloud " << strategyName << " scores computed in "
                      << std::chrono::duration<double>(endFullColor - startFullColor).count()
                      << "s\n";
            exportColoredSidecar(
                cloud, fullColorScores, strategyName, outputFile, fullCloudColorSuffix(strategyName),
                "output", asBinary, fullCloudColorLabel(strategyName));
        }

        // ========== 3. 降采样 ==========
        PointCloud workingCloud = cloud;  // 默认使用原始点云
        std::vector<size_t> downsampleIndices;  // 降采样索引映射

        if (downsampleMethod != "none") {
            if (!DownsamplerRegistry::instance().has(downsampleMethod)) {
                std::cerr << "Error: Unknown downsampling method '" << downsampleMethod << "'\n";
                std::cerr << "Available: ";
                for (const auto& n : DownsamplerRegistry::instance().list()) {
                    std::cerr << n << " ";
                }
                std::cerr << "\n";
                return 1;
            }

            auto dsCreator = DownsamplerRegistry::instance().get(downsampleMethod);
            auto downsampler = dsCreator();

            if (auto* octree = dynamic_cast<OctreeDownsampler*>(downsampler.get())) {
                octree->setLevel(octreeLevel);
                octree->setMode(octreeMode);
            }

            std::cout << "Downsampling with " << downsampler->name()
                      << " (level=" << octreeLevel << ", mode=" << octreeMode << ")...\n";
            auto startDS = std::chrono::high_resolution_clock::now();

            downsampleIndices = downsampler->downsample(cloud);

            auto endDS = std::chrono::high_resolution_clock::now();
            std::cout << "  Downsampled to " << downsampleIndices.size() << " points"
                      << " (" << std::chrono::duration<double>(endDS - startDS).count() << "s)\n";

            // 使用降采样后的点云（包含对应的权重数据）
            workingCloud = cloud.subset(downsampleIndices);

            // 导出降采样后的点云
            std::string dsFilename;
            if (!outputFile.empty() && outputFile != "output.ply") {
                // 从 output 文件名生成降采样文件名
                size_t dotPos = outputFile.rfind('.');
                if (dotPos != std::string::npos) {
                    dsFilename = outputFile.substr(0, dotPos) + "_downsampled" + outputFile.substr(dotPos);
                } else {
                    dsFilename = outputFile + "_downsampled.ply";
                }
            } else {
                dsFilename = "output_downsampled.ply";
            }
            std::cout << "Exporting downsampled cloud to " << dsFilename << "...\n";
            PlyIO::write(workingCloud, dsFilename, asBinary);
        }

        // ========== 4. 构建 KDTree ==========
        std::cout << "Building KDTree...\n";
        auto startTree = std::chrono::high_resolution_clock::now();
        KDTree kdtree(workingCloud);
        auto endTree = std::chrono::high_resolution_clock::now();
        std::cout << "  Built in "
                  << std::chrono::duration<double>(endTree - startTree).count() << "s\n";

        // ========== 5. 创建加权策略 ==========
        if (!WeightRegistry::instance().has(strategyName)) {
            std::cerr << "Error: Unknown strategy '" << strategyName << "'\n";
            std::cerr << "Available: ";
            for (const auto& n : WeightRegistry::instance().list()) {
                std::cerr << n << " ";
            }
            std::cerr << "\n";
            return 1;
        }

        auto creator = WeightRegistry::instance().get(strategyName);
        auto strategy = creator();

        // 配置策略参数
        if (auto* cw = dynamic_cast<CurvatureWeight*>(strategy.get())) {
            cw->setK(k);
        } else if (auto* dw = dynamic_cast<DensityWeight*>(strategy.get())) {
            dw->setK(k);
            dw->setRadius(radius);
        } else if (auto* csw = dynamic_cast<ConsistencyStabilityWeight*>(strategy.get())) {
            csw->setGamma(gamma);
        }

        if (percentRange.enabled && strategy->name() != "uniform") {
            strategy = std::make_unique<PercentileGatedWeight>(
                std::move(strategy), percentRange.start, percentRange.end);
        }

        std::cout << "Strategy: " << strategy->name()
                  << " (alpha=" << alpha << ")";
        if (percentRange.enabled) {
            std::cout << " (percent=" << percentRange.label << ")";
        }
        if (auto* csw = dynamic_cast<ConsistencyStabilityWeight*>(strategy.get())) {
            std::cout << " (gamma=" << csw->gamma() << ")";
        }
        std::cout << "\n";

        std::cout << "Target: " << numTarget << " points from " << workingCloud.size() << "\n";

        if (isColorMappedStrategy(strategyName, workingCloud)) {
            std::cout << "Applying " << strategyName << " colors...\n";
            assignStrategyColors(
                workingCloud, strategyName,
                computeColorScores(strategyName, workingCloud, kdtree, k));
        }

        if (auto* gatedStrategy = dynamic_cast<PercentileGatedWeight*>(strategy.get())) {
            const std::vector<double> gateMask = gatedStrategy->computeGateMask(workingCloud, kdtree);
            if (!gateMask.empty()) {
                const std::vector<size_t> gateIndices = collectPositiveMaskIndices(gateMask);
                PointCloud gatedCloud = workingCloud.subset(gateIndices);
                const std::string gatedFilename =
                    deriveSidecarFilename(outputFile, "_gated", "output");

                std::cout << "Exporting gated cloud to " << gatedFilename
                          << " (" << gatedCloud.size() << " points)...\n";
                PlyIO::write(gatedCloud, gatedFilename, asBinary);
            }
        }

        // ========== 6. FPS 采样 ==========
        FPSSampler sampler;
        sampler.setProgressCallback([](size_t done, size_t total) {
            if (done % 100 == 0 || done == total) {
                std::cout << "\r  Progress: " << done << "/" << total << std::flush;
            }
        });

        std::cout << "Running weighted FPS...\n";
        auto startSample = std::chrono::high_resolution_clock::now();
        auto indices = sampler.sample(workingCloud, *strategy, numTarget, alpha);
        auto endSample = std::chrono::high_resolution_clock::now();
        std::cout << "\r  Sampled " << indices.size() << "/" << numTarget << " points"
                  << " in " << std::chrono::duration<double>(endSample - startSample).count() << "s\n";

        // ========== 7. 构建输出点云 ==========
        PointCloud output = workingCloud.subset(indices);

        // ========== 8. 写入 ==========
        if (outputFile.empty()) outputFile = "output.ply";
        std::cout << "Writing " << outputFile << "...\n";
        PlyIO::write(output, outputFile, asBinary);

        std::cout << "Done.\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
