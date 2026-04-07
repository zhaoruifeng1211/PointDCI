#include "PercentileGatedWeight.h"

#include "ConsistencyStabilityWeight.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

void validateConsistencyStabilityData(const PointCloud& cloud) {
    if (!cloud.hasConsistency() || !cloud.hasStability()) {
        throw std::runtime_error(
            "Percentile gating for consistency_stability requires consistency and stability data.");
    }
    if (cloud.consistency.size() != cloud.size() || cloud.stability.size() != cloud.size()) {
        throw std::runtime_error("Consistency/stability data size mismatch with point cloud size.");
    }
}

std::vector<std::pair<double, size_t>> rankScoresDescending(const std::vector<double>& scores) {
    std::vector<std::pair<double, size_t>> ranked;
    ranked.reserve(scores.size());
    for (size_t i = 0; i < scores.size(); ++i) {
        ranked.emplace_back(scores[i], i);
    }

    std::sort(ranked.begin(), ranked.end(),
              [](const auto& a, const auto& b) {
                  if (a.first != b.first) {
                      return a.first > b.first;
                  }
                  return a.second < b.second;
              });
    return ranked;
}

}  // namespace

std::vector<double> PercentileGatedWeight::computeGateMask(const PointCloud& cloud, const KDTree& tree) const {
    if (base_->name() == "uniform") {
        return {};
    }

    if (base_->name() == "consistency_stability") {
        validateConsistencyStabilityData(cloud);

        const std::vector<double> consistencyMask =
            applyPercentileRange(cloud.consistency, percentStart_, percentEnd_);
        const std::vector<double> stabilityMask =
            applyPercentileRange(cloud.stability, percentStart_, percentEnd_);

        std::vector<double> gateMask(cloud.size(), 0.0);
        for (size_t i = 0; i < gateMask.size(); ++i) {
            if (consistencyMask[i] > 0.0 && stabilityMask[i] > 0.0) {
                gateMask[i] = 1.0;
            }
        }
        return gateMask;
    }

    std::vector<double> baseWeights = base_->compute(cloud, tree);
    std::vector<double> gateScores = buildGateScores(cloud, base_->name());
    if (gateScores.empty()) {
        gateScores = baseWeights;
    }

    return applyPercentileRange(gateScores, percentStart_, percentEnd_);
}

std::vector<double> PercentileGatedWeight::compute(const PointCloud& cloud, const KDTree& tree) {
    if (base_->name() == "uniform") {
        return base_->compute(cloud, tree);
    }

    const std::vector<double> gateMask = computeGateMask(cloud, tree);
    if (gateMask.empty()) {
        return base_->compute(cloud, tree);
    }

    if (base_->name() == "consistency_stability") {
        validateConsistencyStabilityData(cloud);

        const auto* csWeight = dynamic_cast<const ConsistencyStabilityWeight*>(base_.get());
        const double gamma = csWeight ? csWeight->gamma() : 1.0;

        std::vector<double> weights(cloud.size(), 0.0);
        for (size_t i = 0; i < cloud.size(); ++i) {
            if (gateMask[i] <= 0.0) {
                continue;
            }
            // const double sum = std::abs(cloud.consistency[i] + cloud.stability[i]);
            const double sum = std::abs(cloud.consistency[i] - cloud.stability[i]);

            weights[i] = std::pow(1.0 + sum, gamma);
        }
        return weights;
    }

    return gateMask;
}

double PercentileGatedWeight::adjustDistance(double rawDist, double weight, double alpha) const {
    if (base_->name() == "consistency_stability") {
        return rawDist * (1.0 + weight);
    }
    return base_->adjustDistance(rawDist, weight, alpha);
}

std::vector<double> PercentileGatedWeight::buildGateScores(const PointCloud& cloud, const std::string& strategyName) {
    return {};
}

std::vector<double> PercentileGatedWeight::applyPercentileRange(const std::vector<double>& scores,
                                                                double percentStart,
                                                                double percentEnd) {
    const size_t n = scores.size();
    if (n == 0) {
        return {};
    }

    if (percentStart < 0.0 || percentEnd > 1.0 || percentStart >= percentEnd) {
        throw std::runtime_error("Invalid percentile range.");
    }

    size_t startRank = static_cast<size_t>(std::floor(percentStart * static_cast<double>(n)));
    size_t endRank = static_cast<size_t>(std::ceil(percentEnd * static_cast<double>(n)));
    startRank = std::min(startRank, n);
    endRank = std::min(endRank, n);

    if (startRank >= endRank) {
        return std::vector<double>(n, 0.0);
    }

    const std::vector<std::pair<double, size_t>> ranked = rankScoresDescending(scores);
    std::vector<double> gated(n, 0.0);
    for (size_t rank = startRank; rank < endRank; ++rank) {
        gated[ranked[rank].second] = 1.0;
    }

    return gated;
}
