#include "ConsistencyStabilityWeight.h"
#include <cmath>
#include <stdexcept>

std::vector<double> ConsistencyStabilityWeight::compute(const PointCloud& cloud, const KDTree& tree) {
    std::vector<double> weights;

    if (!cloud.hasConsistency() || !cloud.hasStability()) {
        throw std::runtime_error(
            "ConsistencyStabilityWeight requires consistency and stability data. "
            "Please provide --consistency and --stability files.");
    }

    if (cloud.consistency.size() != cloud.size() || cloud.stability.size() != cloud.size()) {
        throw std::runtime_error(
            "Consistency/stability data size mismatch with point cloud size.");
    }

    weights.resize(cloud.size());

    #pragma omp parallel for schedule(static, 4096)
    for (size_t i = 0; i < cloud.size(); ++i) {
        double diff = std::abs(cloud.consistency[i] - cloud.stability[i]);
        weights[i] = std::pow(1.0 + diff, gamma_);
    }

    return weights;
}

double ConsistencyStabilityWeight::adjustDistance(double rawDist, double weight, double alpha) const {
    // 与其他加权策略一致: D'_i = D_i * (1 + α * Score_i)
    return rawDist * (1.0 + alpha * weight);
}
