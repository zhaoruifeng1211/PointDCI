#pragma once

#include <vector>
#include <string>
#include <cstddef>
#include <cstdint>

struct Point3D {
    double x = 0, y = 0, z = 0;
    double nx = 0, ny = 0, nz = 0;  // optional normals

    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Point3D(double x_, double y_, double z_, double nx_, double ny_, double nz_)
        : x(x_), y(y_), z(z_), nx(nx_), ny(ny_), nz(nz_) {}

    double squaredDistance(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return dx*dx + dy*dy + dz*dz;
    }
};

class PointCloud {
public:
    std::vector<Point3D> points;
    bool hasNormals = false;

    // 权重数据（与点一一对应）
    std::vector<double> intensity;
    std::vector<double> consistency;
    std::vector<double> stability;
    std::vector<std::uint8_t> red;
    std::vector<std::uint8_t> green;
    std::vector<std::uint8_t> blue;

    PointCloud() = default;
    explicit PointCloud(size_t n) : points(n) {}

    size_t size() const { return points.size(); }
    void reserve(size_t n) { points.reserve(n); }
    void clear() {
        points.clear();
        hasNormals = false;
        intensity.clear();
        consistency.clear();
        stability.clear();
        red.clear();
        green.clear();
        blue.clear();
    }

    const Point3D& operator[](size_t i) const { return points[i]; }
    Point3D& operator[](size_t i) { return points[i]; }

    void addPoint(const Point3D& p) { points.push_back(p); }

    bool hasIntensity() const { return !intensity.empty(); }
    bool hasConsistency() const { return !consistency.empty(); }
    bool hasStability() const { return !stability.empty(); }
    bool hasWeights() const { return hasConsistency() && hasStability(); }
    bool hasColors() const { return !red.empty() && red.size() == points.size()
                                  && green.size() == points.size()
                                  && blue.size() == points.size(); }

    // 根据索引列表提取子点云（保留权重）
    PointCloud subset(const std::vector<size_t>& indices) const {
        PointCloud result(indices.size());
        result.hasNormals = hasNormals;
        for (size_t i = 0; i < indices.size(); ++i) {
            result.points[i] = points[indices[i]];
            if (hasIntensity()) {
                result.intensity.push_back(intensity[indices[i]]);
            }
            if (hasConsistency()) {
                result.consistency.push_back(consistency[indices[i]]);
            }
            if (hasStability()) {
                result.stability.push_back(stability[indices[i]]);
            }
            if (hasColors()) {
                result.red.push_back(red[indices[i]]);
                result.green.push_back(green[indices[i]]);
                result.blue.push_back(blue[indices[i]]);
            }
        }
        return result;
    }
};
