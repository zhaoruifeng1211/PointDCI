#pragma once

#include "Downsampler.h"
#include <string>
#include <cstddef>

/**
 * @brief Octree 降采样策略（CloudCompare 风格）
 *
 * 基于空间八叉树划分的降采样。
 * 将点云空间递归八分到指定的 subdivision level，
 * 每个体素内保留一个代表点。
 *
 * Subdivision Level 含义:
 *   - 空间被划分为 2^level × 2^level × 2^level 个体素
 *   - Level 6:  64³ = 262,144 体素
 *   - Level 7: 128³ = 2,097,152 体素
 *   - Level 8: 256³ = 16,777,216 体素
 *   - Level 12: 4096³ = 68,719,476,736 体素
 *
 * 对于表面点云，建议使用 Level 6-12。
 */
class OctreeDownsampler : public Downsampler {
public:
    OctreeDownsampler() = default;
    explicit OctreeDownsampler(int level) : level_(level) {}

    std::string name() const override { return "octree"; }
    std::string description() const override {
        return "Octree-based downsampling. "
               "Divides space into 2^level voxels, keeps one point per voxel. "
               "Level 6-12 recommended for surface point clouds.";
    }

    std::vector<size_t> downsample(const PointCloud& cloud) override;

    int level() const { return level_; }
    void setLevel(int level) { level_ = level; }

    std::string mode() const { return mode_; }
    void setMode(const std::string& mode) { mode_ = mode; }

private:
    int level_ = 6;  // default subdivision level
    std::string mode_ = "center";  // "center" or "first"
};
