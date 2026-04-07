#pragma once

#include "../core/PointCloud.h"
#include <string>

class ObjIO {
public:
    struct ParseOptions {
        bool flipYZ = false;
    };

    static PointCloud read(const std::string& filepath);
    static PointCloud read(const std::string& filepath, ParseOptions opts);
};
