#pragma once

#include "WeightStrategy.h"

/**
 * @brief 密度加权策略
 *
 * 对每个点计算局部密度: 该点k近邻范围内点的数量 / 体积
 * 稀疏区域权重高 → 倾向于在稀疏区域采样更多点
 * 密集区域权重低 → 减少密集区域的冗余采样
 *
 * 修正公式: D'_i = D_i / (1 + α * density_i)
 *          效果: 密度越低(weight越大), D'越大, 越容易被选中
 */
class DensityWeight : public WeightStrategy {
public:
    DensityWeight() = default;
    explicit DensityWeight(int k, double radius = 1.0) : k_(k), radius_(radius) {}

    std::string name() const override { return "density"; }
    std::string description() const override {
        return "Density-weighted FPS: D' = D / (1 + α * ρ). "
               "Favors sparse regions over dense ones, "
               "producing a more uniform sampling in feature-space density.";
    }

    std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) override;
    double adjustDistance(double rawDist, double weight, double alpha) const override;

    int k() const { return k_; }
    double radius() const { return radius_; }
    void setK(int k) { k_ = k; }
    void setRadius(double r) { radius_ = r; }

private:
    int k_ = 20;
    double radius_ = 1.0;  // 密度估计的搜索半径
};

REGISTER_WEIGHT(DensityWeight, "density");
