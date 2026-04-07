#include "Downsampler.h"

// ============= DownsamplerRegistry implementation =============

DownsamplerRegistry& DownsamplerRegistry::instance() {
    static DownsamplerRegistry inst;
    return inst;
}

void DownsamplerRegistry::register_(const std::string& name, CreatorFunc creator) {
    creators_[name] = std::move(creator);
}

bool DownsamplerRegistry::has(const std::string& name) const {
    return creators_.find(name) != creators_.end();
}

DownsamplerRegistry::CreatorFunc DownsamplerRegistry::get(const std::string& name) const {
    auto it = creators_.find(name);
    if (it != creators_.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<std::string> DownsamplerRegistry::list() const {
    std::vector<std::string> names;
    names.reserve(creators_.size());
    for (const auto& kv : creators_) {
        names.push_back(kv.first);
    }
    return names;
}
