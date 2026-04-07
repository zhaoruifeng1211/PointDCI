#pragma once

#include "WeightStrategy.h"

class UniformWeight : public WeightStrategy {
public:
    std::string name() const override { return "uniform"; }
    std::string description() const override { return "No weighting — standard FPS (uniform spatial coverage)"; }

    std::vector<double> compute(const PointCloud&, const KDTree&) override {
        return {};  // 空vector表示不使用预计算权重
    }

    double adjustDistance(double rawDist, double, double) const override {
        return rawDist;
    }

    bool supportsParallel() const override { return false; }
};

REGISTER_WEIGHT(UniformWeight, "uniform");
