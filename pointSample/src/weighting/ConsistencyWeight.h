#pragma once

#include "WeightStrategy.h"
#include <string>

/**
 * @brief Consistency 直接加权策略
 *
 * 直接使用 consistency 值作为权重进行 FPS 加权。
 * Score(p_i) = C_i
 *
 * 修正公式: D'_i = D_i * (1 + α * C_i)
 */
class ConsistencyWeight : public WeightStrategy {
public:
    ConsistencyWeight() = default;

    std::string name() const override { return "consistency"; }
    std::string description() const override {
        return "Consistency-weighted FPS: Score = C. "
               "Directly uses consistency values as weights. "
               "Requires --consistency data file.";
    }

    std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) override;
    double adjustDistance(double rawDist, double weight, double alpha) const override;
};

REGISTER_WEIGHT(ConsistencyWeight, "consistency");
