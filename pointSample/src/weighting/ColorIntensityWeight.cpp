#include "ColorIntensityWeight.h"

#include <stdexcept>

std::vector<double> ColorIntensityWeight::compute(const PointCloud& cloud, const KDTree&) {
    if (!cloud.hasIntensity()) {
        throw std::runtime_error("ColorIntensityWeight requires point intensity data");
    }
    return cloud.intensity;
}

double ColorIntensityWeight::adjustDistance(double rawDist, double weight, double alpha) const {
    return rawDist * (1.0 + alpha * weight);
}
