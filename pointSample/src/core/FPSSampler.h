#pragma once

#include "PointCloud.h"
#include "KDTree.h"
#include "../weighting/WeightStrategy.h"
#include <vector>
#include <memory>
#include <queue>
#include <functional>
#include <cstddef>
#include <algorithm>

/**
 * @brief FPS 采样器核心
 *
 * 支持加权策略的 Farthest Point Sampling 实现。
 *
 * 修正距离 D'_i 由 WeightStrategy::adjustDistance() 定义。
 */
class FPSSampler {
public:
    FPSSampler() = default;

    /**
     * @brief 执行 FPS 采样
     * @param cloud 输入点云
     * @param strategy 加权策略
     * @param numTarget 目标采样点数
     * @param alpha 加权强度系数
     * @return 采样点的索引列表
     */
    std::vector<size_t> sample(
        const PointCloud& cloud,
        WeightStrategy& strategy,
        size_t numTarget,
        double alpha = 1.0);

    /**
     * @brief 获取采样进度回调（每选出一个点调用一次）
     */
    void setProgressCallback(std::function<void(size_t, size_t)> cb) {
        progressCallback_ = std::move(cb);
    }

private:
    // 优先队列元素: (修正后的距离, 点索引)
    using QueueElement = std::pair<double, size_t>;

    std::function<void(size_t, size_t)> progressCallback_;
};
