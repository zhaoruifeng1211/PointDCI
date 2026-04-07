#include "WeightStrategy.h"
#include <map>

// WeightStrategy.cpp 仅包含 Registry 单例实现
// 具体策略的注册在各策略源文件中通过 REGISTER_WEIGHT 宏完成

WeightRegistry& WeightRegistry::instance() {
    static WeightRegistry registry;
    return registry;
}

void WeightRegistry::register_(const std::string& name, CreatorFunc creator) {
    creators_[name] = std::move(creator);
}

bool WeightRegistry::has(const std::string& name) const {
    return creators_.find(name) != creators_.end();
}

WeightRegistry::CreatorFunc WeightRegistry::get(const std::string& name) const {
    auto it = creators_.find(name);
    if (it != creators_.end()) return it->second;
    return nullptr;
}

std::vector<std::string> WeightRegistry::list() const {
    std::vector<std::string> names;
    names.reserve(creators_.size());
    for (const auto& kv : creators_) names.push_back(kv.first);
    return names;
}
