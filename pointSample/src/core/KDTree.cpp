#include "KDTree.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

double squaredDistance(const Point3D& lhs, const Point3D& rhs) {
    return lhs.squaredDistance(rhs);
}

}  // namespace

KDTree::KDTree() = default;
KDTree::~KDTree() = default;

KDTree::KDTree(const PointCloud& cloud) {
    build(cloud);
}

void KDTree::build(const PointCloud& cloud) {
    cloud_ = &cloud;
}

void KDTree::clear() {
    cloud_ = nullptr;
}

std::vector<std::pair<size_t, double>> KDTree::knnSearch(
    const Point3D& point, int k) const {

    std::vector<std::pair<size_t, double>> ret;
    if (!cloud_ || cloud_->size() == 0 || k <= 0) return ret;

    ret.reserve(cloud_->size());
    for (size_t i = 0; i < cloud_->size(); ++i) {
        ret.emplace_back(i, squaredDistance(point, (*cloud_)[i]));
    }

    const size_t limit = std::min(static_cast<size_t>(k), ret.size());
    std::nth_element(ret.begin(), ret.begin() + (limit - 1), ret.end(),
                     [](const auto& lhs, const auto& rhs) {
                         return lhs.second < rhs.second;
                     });
    ret.resize(limit);
    std::sort(ret.begin(), ret.end(),
              [](const auto& lhs, const auto& rhs) {
                  return lhs.second < rhs.second;
              });

    for (auto& item : ret) {
        if (!std::isfinite(item.second)) {
            item.second = std::numeric_limits<double>::infinity();
        }
    }
    return ret;
}

std::vector<std::pair<size_t, double>> KDTree::radiusSearch(
    const Point3D& point, double radius, int maxNeighbors) const {

    std::vector<std::pair<size_t, double>> ret;
    if (!cloud_ || cloud_->size() == 0 || radius <= 0) return ret;

    const double radiusSquared = radius * radius;
    ret.reserve(cloud_->size());
    for (size_t i = 0; i < cloud_->size(); ++i) {
        const double dist = squaredDistance(point, (*cloud_)[i]);
        if (dist <= radiusSquared) {
            ret.emplace_back(i, dist);
        }
    }

    std::sort(ret.begin(), ret.end(),
              [](const auto& lhs, const auto& rhs) {
                  return lhs.second < rhs.second;
              });

    if (maxNeighbors > 0 && static_cast<int>(ret.size()) > maxNeighbors) {
        ret.resize(maxNeighbors);
    }

    return ret;
}
