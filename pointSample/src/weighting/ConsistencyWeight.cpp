#include "ConsistencyWeight.h"
#include <stdexcept>

std::vector<double> ConsistencyWeight::compute(const PointCloud& cloud, const KDTree& tree) {
    if (!cloud.hasConsistency()) {
        throw std::runtime_error(
            "ConsistencyWeight requires consistency data. "
            "Please provide --consistency file.");
    }

    return cloud.consistency;  // 直接返回 consistency 作为权重
}

double ConsistencyWeight::adjustDistance(double rawDist, double weight, double alpha) const {
    return rawDist * (1.0 + alpha * weight);
}
