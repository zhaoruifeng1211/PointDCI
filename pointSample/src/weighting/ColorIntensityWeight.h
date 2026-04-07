#pragma once

#include "WeightStrategy.h"

class ColorIntensityWeight : public WeightStrategy {
public:
    std::string name() const override { return "color-intensity"; }
    std::string description() const override {
        return "Color-intensity-weighted FPS: D' = D * (1 + alpha * I), "
               "where I = max(0, R - max(G, B)). Preferentially samples red-dominant vertices.";
    }

    std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) override;
    double adjustDistance(double rawDist, double weight, double alpha) const override;
    bool supportsParallel() const override { return false; }
};

REGISTER_WEIGHT(ColorIntensityWeight, "color-intensity");
