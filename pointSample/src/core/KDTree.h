#pragma once

#include "PointCloud.h"
#include <vector>

class KDTree {
public:
    KDTree();
    ~KDTree();
    explicit KDTree(const PointCloud& cloud);

    void build(const PointCloud& cloud);
    void clear();

    std::vector<std::pair<size_t, double>> knnSearch(const Point3D& point, int k) const;
    std::vector<std::pair<size_t, double>> radiusSearch(
        const Point3D& point, double radius, int maxNeighbors = 0) const;

private:
    const PointCloud* cloud_ = nullptr;
};
