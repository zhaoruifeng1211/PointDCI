#include "ObjIO.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

double parseColorComponent(double value) {
    if (value > 1.0) {
        value /= 255.0;
    }
    return std::clamp(value, 0.0, 1.0);
}

double redDominanceIntensity(double r, double g, double b) {
    return std::max(0.0, r - std::max(g, b));
}

}  // namespace

PointCloud ObjIO::read(const std::string& filepath, ParseOptions opts) {
    std::FILE* f = std::fopen(filepath.c_str(), "r");
    if (!f) {
        return {};
    }

    PointCloud cloud;
    char line[4096];

    while (std::fgets(line, sizeof(line), f)) {
        if (line[0] != 'v' || (line[1] != ' ' && line[1] != '\t')) {
            continue;
        }

        std::vector<double> vals;
        char* token = std::strtok(line + 1, " \t\r\n");
        while (token) {
            vals.push_back(std::atof(token));
            token = std::strtok(nullptr, " \t\r\n");
        }

        if (vals.size() < 6) {
            continue;
        }

        double x = vals[0];
        double y = vals[1];
        double z = vals[2];
        if (opts.flipYZ) {
            std::swap(y, z);
        }

        const double r = parseColorComponent(vals[3]);
        const double g = parseColorComponent(vals[4]);
        const double b = parseColorComponent(vals[5]);

        cloud.addPoint(Point3D(x, y, z));
        cloud.intensity.push_back(redDominanceIntensity(r, g, b));
    }

    std::fclose(f);
    return cloud;
}

PointCloud ObjIO::read(const std::string& filepath) {
    return read(filepath, ParseOptions{});
}
