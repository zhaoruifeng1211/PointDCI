#pragma once

#include "../core/PointCloud.h"
#include "../core/KDTree.h"
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <map>

// 前向声明
class WeightStrategy;

/**
 * @brief 加权策略注册表
 *
 * 所有加权策略需先注册到 registry 中才能被 CLI 使用。
 * 注册发生在各策略源文件的静态初始化块中。
 */
class WeightRegistry {
public:
    using CreatorFunc = std::function<std::unique_ptr<WeightStrategy>()>;

    static WeightRegistry& instance();

    // 注册一个策略
    void register_(const std::string& name, CreatorFunc creator);

    // 检查是否已注册
    bool has(const std::string& name) const;

    // 获取策略创建函数
    CreatorFunc get(const std::string& name) const;

    // 列出所有已注册策略名
    std::vector<std::string> list() const;

private:
    WeightRegistry() = default;
    WeightRegistry(const WeightRegistry&) = delete;
    WeightRegistry& operator=(const WeightRegistry&) = delete;

    std::map<std::string, CreatorFunc> creators_;
};

/**
 * @brief 加权策略基类
 *
 * 子类实现 compute() 定义如何计算每个点的权重，
 * 实现 adjustDistance() 定义如何将权重融入 FPS 距离度量。
 *
 * 标准 FPS 距离: D_i = min_{s in S} ||p_i - s||_2
 * 修正后:        D_i' = adjustDistance(D_i, weight_i, alpha)
 *
 * 例如曲率加权:   D_i' = D_i * (1 + alpha * curvature_i)
 */
class WeightStrategy {
public:
    virtual ~WeightStrategy() = default;

    // 策略名称
    virtual std::string name() const = 0;

    // 策略描述（用于 --help）
    virtual std::string description() const = 0;

    /**
     * @brief 预计算每个点的权重
     * @param cloud 输入点云
     * @param tree 预先构建好的 KDTree（用于 KNN 查询）
     * @return 每个点对应的权重值，越大越"显著"，越容易被 FPS 优先选中
     */
    virtual std::vector<double> compute(const PointCloud& cloud, const KDTree& tree) = 0;

    /**
     * @brief 用权重修正 FPS 原始距离
     * @param rawDist 原始最小欧氏距离 D
     * @param weight 该点的预计算权重
     * @param alpha 全局强度系数（CLI 参数传入）
     * @return 修正后的距离值
     */
    virtual double adjustDistance(double rawDist, double weight, double alpha) const = 0;

    /**
     * @brief 是否需要在 compute 阶段使用多线程并行
     */
    virtual bool supportsParallel() const { return true; }
};

// 用于自动注册策略的辅助宏
#define REGISTER_WEIGHT(StrategyClass, name) \
    static struct StrategyClass##Registrar { \
        StrategyClass##Registrar() { \
            WeightRegistry::instance().register_(name, []() { \
                return std::make_unique<StrategyClass>(); \
            }); \
        } \
    } strategy_class_##StrategyClass##_registrar;
