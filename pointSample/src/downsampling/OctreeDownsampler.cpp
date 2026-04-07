#include "OctreeDownsampler.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <limits>

namespace {

// 计算体素的哈希 key
inline uint64_t voxelKey(int ix, int iy, int iz, int level) {
    // 将三个 16 位坐标压缩成一个 64 位 key
    // 每个坐标最多 2^16 = 65536，对应 level 16
    return (static_cast<uint64_t>(ix & 0xFFFF) << 48) |
           (static_cast<uint64_t>(iy & 0xFFFF) << 32) |
           (static_cast<uint64_t>(iz & 0xFFFF) << 16) |
           (static_cast<uint64_t>(level & 0xFFFF));
}

// 获取点相对于体素中心的距离平方
inline double distToVoxelCenter(const Point3D& p, double cx, double cy, double cz) {
    double dx = p.x - cx;
    double dy = p.y - cy;
    double dz = p.z - cz;
    return dx*dx + dy*dy + dz*dz;
}

} // anonymous namespace

std::vector<size_t> OctreeDownsampler::downsample(const PointCloud& cloud) {
    if (cloud.size() == 0) {
        return {};
    }

    const int level = level_;
    const int n = static_cast<int>(cloud.size());

    // 1. 计算点云包围盒
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    double maxZ = std::numeric_limits<double>::lowest();

    for (const auto& p : cloud.points) {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        minZ = std::min(minZ, p.z);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
        maxZ = std::max(maxZ, p.z);
    }

    // 防止无效包围盒
    if (minX == maxX) maxX = minX + 1e-6;
    if (minY == maxY) maxY = minY + 1e-6;
    if (minZ == maxZ) maxZ = minZ + 1e-6;

    // 2. 根据 level 计算网格划分
    // level 6 -> 64, level 7 -> 128, ..., level 12 -> 4096
    const int gridSize = 1 << level;  // 2^level
    const double voxelSizeX = (maxX - minX) / gridSize;
    const double voxelSizeY = (maxY - minY) / gridSize;
    const double voxelSizeZ = (maxZ - minZ) / gridSize;

    // 3. 将点分配到体素
    // voxel_data: key -> (point_index, dist_to_center)
    std::unordered_map<uint64_t, std::pair<size_t, double>> voxelMap;
    voxelMap.reserve(n / 2);  // 预分配

    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud[i];

        // 计算该点所属的体素索引
        int ix = static_cast<int>((p.x - minX) / voxelSizeX);
        int iy = static_cast<int>((p.y - minY) / voxelSizeY);
        int iz = static_cast<int>((p.z - minZ) / voxelSizeZ);

        // 边界情况：点恰好在最大边界上
        ix = std::min(ix, gridSize - 1);
        iy = std::min(iy, gridSize - 1);
        iz = std::min(iz, gridSize - 1);

        uint64_t key = voxelKey(ix, iy, iz, level);

        // 计算体素中心
        double cx = minX + (ix + 0.5) * voxelSizeX;
        double cy = minY + (iy + 0.5) * voxelSizeY;
        double cz = minZ + (iz + 0.5) * voxelSizeZ;

        double dist = distToVoxelCenter(p, cx, cy, cz);

        auto it = voxelMap.find(key);
        if (it == voxelMap.end()) {
            // 新体素，插入
            voxelMap[key] = {i, dist};
        } else {
            // 体素已有点，比较距离
            if (mode_ == "center") {
                // 保留最接近体素中心的点
                if (dist < it->second.second) {
                    it->second = {i, dist};
                }
            } else {
                // "first" 模式：保留第一个遇到的点，不更新
            }
        }
    }

    // 4. 提取结果
    std::vector<size_t> indices;
    indices.reserve(voxelMap.size());
    for (auto& kv : voxelMap) {
        indices.push_back(kv.second.first);
    }

    return indices;
}

// 注册到全局注册表
REGISTER_DOWNSAMPLER(OctreeDownsampler, "octree");
