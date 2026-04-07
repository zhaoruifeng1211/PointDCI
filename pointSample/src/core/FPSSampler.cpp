#include "FPSSampler.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <chrono>
#include <iostream>

/**
 * @brief Lazy FPS with priority queue
 *
 * Uses O(log n) priority queue instead of O(n) linear scan.
 * Supports decrease-key via "lazy" approach: push new entries,
 * skip stale ones when popping.
 */
std::vector<size_t> FPSSampler::sample(
    const PointCloud& cloud,
    WeightStrategy& strategy,
    size_t numTarget,
    double alpha) {

    const size_t n = cloud.size();
    if (n == 0 || numTarget == 0) return {};
    numTarget = std::min(numTarget, n);

    // 预计算权重（可能为空）
    auto t0 = std::chrono::high_resolution_clock::now();
    KDTree tree(cloud);
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cerr << "  [KDTree build: " << std::chrono::duration<double>(t1-t0).count() << "s]" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    std::vector<double> weights = strategy.compute(cloud, tree);
    t1 = std::chrono::high_resolution_clock::now();
    std::cerr << "  [Weight compute: " << std::chrono::duration<double>(t1-t0).count() << "s]" << std::endl;

    // 已选中的点
    std::vector<char> selected(n, 0);

    // dist[i] = 点i到已选采样集的最小欧氏距离
    std::vector<double> dist(n, std::numeric_limits<double>::max());

    // 找第一个种子：离原点最远的点
    size_t firstSeed = 0;
    double maxDist2 = 0.0;
    Point3D origin(0, 0, 0);
    for (size_t i = 0; i < n; ++i) {
        double d2 = cloud[i].squaredDistance(origin);
        if (d2 > maxDist2) {
            maxDist2 = d2;
            firstSeed = i;
        }
    }

    std::vector<size_t> result;
    result.reserve(numTarget);
    result.push_back(firstSeed);
    selected[firstSeed] = 1;
    dist[firstSeed] = 0;

    // 初始化 dist
    for (size_t i = 0; i < n; ++i) {
        if (i == firstSeed) continue;
        dist[i] = std::sqrt(cloud[i].squaredDistance(cloud[firstSeed]));
    }

    // 优先队列: (负的距离, 点索引) 用最小堆模拟最大堆
    // 或者直接用 pair<dist, idx> 因为默认是最大堆
    std::priority_queue<std::pair<double, size_t>> pq;
    for (size_t i = 0; i < n; ++i) {
        if (i == firstSeed) continue;
        double w = weights.empty() ? 0.0 : weights[i];
        double adjDist = strategy.adjustDistance(dist[i], w, alpha);
        pq.emplace(adjDist, i);
    }

    // 贪心选择
    size_t loopCount = 0;
    size_t staleCount = 0;
    size_t skipCount = 0;
    auto fpsStart = std::chrono::high_resolution_clock::now();

    while (!pq.empty() && result.size() < numTarget) {
        auto [adjDist, idx] = pq.top();
        pq.pop();
        ++loopCount;

        if (loopCount % 50000 == 0) {
            auto now = std::chrono::high_resolution_clock::now();
            double secs = std::chrono::duration<double>(now - fpsStart).count();
            std::cerr << "  [FPS loop=" << loopCount << " pq=" << pq.size()
                      << " stale=" << staleCount << " skip=" << skipCount
                      << " rate=" << (result.size() / secs) << " pts/s]" << std::endl;
        }

        if (selected[idx]) { ++skipCount; continue; }

        // Lazy stale check: 如果 dist[idx] 比 adjDist 对应的真实距离小，说明过期
        // 由于我们每次 push 新距离时 dist[idx] 已经在变小，adjDist 是用旧距离算的
        // 如果当前 pq 顶点的 adjDist 和真实的 adjustDistance(dist[idx]) 差太多，说明过期
        double w = weights.empty() ? 0.0 : weights[idx];
        double trueAdjDist = strategy.adjustDistance(dist[idx], w, alpha);
        if (std::abs(trueAdjDist - adjDist) > 1e-9 * std::max(1.0, adjDist)) {
            ++staleCount;
            continue;
        }

        result.push_back(idx);
        selected[idx] = 1;

        if (progressCallback_) {
            progressCallback_(result.size(), numTarget);
        }

        // 更新邻居距离（不使用 dist，用真实距离比较）
        for (size_t i = 0; i < n; ++i) {
            if (selected[i]) continue;

            double d = std::sqrt(cloud[i].squaredDistance(cloud[idx]));
            if (d < dist[i]) {
                dist[i] = d;
                double w = weights.empty() ? 0.0 : weights[i];
                double newAdjDist = strategy.adjustDistance(d, w, alpha);
                pq.emplace(newAdjDist, i);
            }
        }
    }

    return result;
}
