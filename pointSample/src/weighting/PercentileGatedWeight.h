#pragma once

#include "WeightStrategy.h"

#include <memory>

class PercentileGatedWeight : public WeightStrategy {
public:
    PercentileGatedWeight(std::unique_ptr<WeightStrategy> base, double percentStart, double percentEnd)
        : base_(std::move(base)), percentStart_(percentStart), percentEnd_(percentEnd) {}

    std::string name() const override { return base_->name(); }
    std::string description() const override { return base_->description(); }

    std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) override;
    std::vector<double> computeGateMask(const PointCloud& cloud, const KDTree& tree) const;
    double adjustDistance(double rawDist, double weight, double alpha) const override;
    bool supportsParallel() const override { return base_->supportsParallel(); }

private:
    static std::vector<double> buildGateScores(const PointCloud& cloud, const std::string& strategyName);
    static std::vector<double> applyPercentileRange(const std::vector<double>& scores,
                                                    double percentStart,
                                                    double percentEnd);

    std::unique_ptr<WeightStrategy> base_;
    double percentStart_ = 0.0;
    double percentEnd_ = 1.0;
};
