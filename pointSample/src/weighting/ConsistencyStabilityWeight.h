#pragma once

#include "WeightStrategy.h"
#include <string>
#include <cstddef>

/**
 * @brief Consistency-Stability 差异加权策略
 *
 * 公式: Score(p_i) = (1 + |C_i - S_i|)^γ
 *
 * 其中:
 *   - C_i = 第 i 个点的 consistency 值（从外部数据读取）
 *   - S_i = 第 i 个点的 stability 值（从外部数据读取）
 *   - γ (gamma) = 指数参数，控制差异的放大程度
 *
 * 修正公式: D'_i = D_i * (1 + α * Score_i)
 *
 * 适用于:
 *   - 选择 consistency 和 stability 差异显著的点
 *   - 这类点通常位于特征边界或变化剧烈的区域
 */
class ConsistencyStabilityWeight : public WeightStrategy {
public:
    ConsistencyStabilityWeight() = default;
    explicit ConsistencyStabilityWeight(double gamma) : gamma_(gamma) {}

    std::string name() const override { return "consistency_stability"; }
    std::string description() const override {
        return "Consistency-Stability weighted FPS: Score = (1 + |C-S|)^gamma. "
               "Selects points with high difference between consistency and stability. "
               "Requires --consistency and --stability data files.";
    }

    std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) override;
    double adjustDistance(double rawDist, double weight, double alpha) const override;

    double gamma() const { return gamma_; }
    void setGamma(double gamma) { gamma_ = gamma; }

private:
    double gamma_ = 1.0;  // 指数参数
};

REGISTER_WEIGHT(ConsistencyStabilityWeight, "consistency_stability");
