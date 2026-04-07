#pragma once

#include "../core/PointCloud.h"
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <map>

// Forward declaration
class Downsampler;

/**
 * @brief 降采样策略注册表
 *
 * 所有降采样策略需先注册到 registry 中才能被 CLI 使用。
 * 注册发生在各策略源文件的静态初始化块中。
 */
class DownsamplerRegistry {
public:
    using CreatorFunc = std::function<std::unique_ptr<Downsampler>()>;

    static DownsamplerRegistry& instance();

    // 注册一个策略
    void register_(const std::string& name, CreatorFunc creator);

    // 检查是否已注册
    bool has(const std::string& name) const;

    // 获取策略创建函数
    CreatorFunc get(const std::string& name) const;

    // 列出所有已注册策略名
    std::vector<std::string> list() const;

private:
    DownsamplerRegistry() = default;
    DownsamplerRegistry(const DownsamplerRegistry&) = delete;
    DownsamplerRegistry& operator=(const DownsamplerRegistry&) = delete;

    std::map<std::string, CreatorFunc> creators_;
};

/**
 * @brief 降采样策略基类
 *
 * 子类实现 downsample() 定义具体的降采样逻辑。
 * 返回降采样后保留点的索引列表。
 */
class Downsampler {
public:
    virtual ~Downsampler() = default;

    // 策略名称
    virtual std::string name() const = 0;

    // 策略描述
    virtual std::string description() const = 0;

    /**
     * @brief 执行降采样
     * @param cloud 输入点云
     * @return 保留点的索引列表
     */
    virtual std::vector<size_t> downsample(const PointCloud& cloud) = 0;
};

// 用于自动注册策略的辅助宏
#define REGISTER_DOWNSAMPLER(DownsamplerClass, name) \
    static struct DownsamplerClass##Registrar { \
        DownsamplerClass##Registrar() { \
            DownsamplerRegistry::instance().register_(name, []() { \
                return std::make_unique<DownsamplerClass>(); \
            }); \
        } \
    } downsampler_class_##DownsamplerClass##_registrar;
