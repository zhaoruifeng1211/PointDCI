#pragma once

#include "WeightStrategy.h"

/**
 * @brief 曲率加权策略
 *
 * 对每个点计算局部曲率指标 C_i = λ_min / λ_max
 * （协方差矩阵最小特征值与最大特征值之比）
 *
 * 曲率大 → 边缘/褶皱/尖锐区域 → 更容易被选中
 * 曲率小 → 平坦区域 → 保持均匀采样
 *
 * 修正公式: D'_i = D_i * (1 + α * C_i)
 */
class CurvatureWeight : public WeightStrategy {
public:
    CurvatureWeight() = default;
    explicit CurvatureWeight(int k) : k_(k) {}

    std::string name() const override { return "curvature"; }
    std::string description() const override {
        return "Curvature-weighted FPS: D' = D * (1 + α * C), "
               "where C = λ_min/λ_max from local PCA. "
               "Preferentially samples edges, folds, and sharp geometric features.";
    }

    std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) override;
    double adjustDistance(double rawDist, double weight, double alpha) const override;

    int k() const { return k_; }
    void setK(int k) { k_ = k; }

private:
    int k_ = 20;  // K近邻数，用于PCA局部拟合
};

REGISTER_WEIGHT(CurvatureWeight, "curvature");
