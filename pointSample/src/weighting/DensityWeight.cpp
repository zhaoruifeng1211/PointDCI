#include "DensityWeight.h"
#include <omp.h>
#include <cmath>

std::vector<double> DensityWeight::compute(const PointCloud& cloud, const KDTree& tree) {
    const size_t n = cloud.size();
    std::vector<double> densities(n, 0.0);

    // 计算局部密度: neighbors within radius / volume
    const double volume = (4.0 / 3.0) * M_PI * radius_ * radius_ * radius_;

    #pragma omp parallel for schedule(dynamic, 512)
    for (size_t i = 0; i < n; ++i) {
        const auto& pt = cloud[i];
        auto neighbors = tree.radiusSearch(pt, radius_, 0);

        // 密度 = 邻域点数 / 球体积
        // 使用的是 kNN 的实际返回数(不超过 maxNeighbors)
        densities[i] = static_cast<double>(neighbors.size()) / volume;
    }

    return densities;
}

double DensityWeight::adjustDistance(double rawDist, double weight, double alpha) const {
    // D' = D / (1 + α * density)
    // 密度越高, weight越大, D'越小 → 不容易被选中
    // 密度越低, weight越小, D'越接近 D → 更容易被选中
    return rawDist / (1.0 + alpha * weight);
}
