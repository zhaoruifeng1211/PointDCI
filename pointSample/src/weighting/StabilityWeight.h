#pragma once

#include "WeightStrategy.h"
#include <string>

/**
 * @brief Stability 直接加权策略
 *
 * 直接使用 stability 值作为权重进行 FPS 加权。
 * Score(p_i) = S_i
 *
 * 修正公式: D'_i = D_i * (1 + α * S_i)
 */
class StabilityWeight : public WeightStrategy {
public:
    StabilityWeight() = default;

    std::string name() const override { return "stability"; }
    std::string description() const override {
        return "Stability-weighted FPS: Score = S. "
               "Directly uses stability values as weights. "
               "Requires --stability data file.";
    }

    std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) override;
    double adjustDistance(double rawDist, double weight, double alpha) const override;
};

REGISTER_WEIGHT(StabilityWeight, "stability");
