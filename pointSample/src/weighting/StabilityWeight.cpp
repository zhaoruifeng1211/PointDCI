#include "StabilityWeight.h"
#include <stdexcept>

std::vector<double> StabilityWeight::compute(const PointCloud& cloud, const KDTree& tree) {
    if (!cloud.hasStability()) {
        throw std::runtime_error(
            "StabilityWeight requires stability data. "
            "Please provide --stability file.");
    }

    return cloud.stability;  // 直接返回 stability 作为权重
}

double StabilityWeight::adjustDistance(double rawDist, double weight, double alpha) const {
    return rawDist * (1.0 + alpha * weight);
}
