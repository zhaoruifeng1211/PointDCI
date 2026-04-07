#include "PlyIO.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <cmath>

namespace {

struct PlyProperty {
    std::string name;
    bool isFloat = false;
    bool isDouble = false;
    bool isInt = false;
    bool isUChar = false;
};

struct PlyHeader {
    PlyIO::Format format = PlyIO::Format::Ascii;
    size_t vertexCount = 0;
    std::vector<PlyProperty> props;
};

PlyHeader parseHeader(std::FILE* f) {
    PlyHeader header;
    char line[512];

    if (!fgets(line, sizeof(line), f)) return header;
    if (strncmp(line, "ply", 3) != 0) return header;

    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "format", 6) == 0) {
            if (strstr(line, "ascii")) header.format = PlyIO::Format::Ascii;
            else if (strstr(line, "binary_little_endian"))
                header.format = PlyIO::Format::BinaryLE;
        } else if (strncmp(line, "element vertex", 14) == 0) {
            header.vertexCount = std::atoi(line + 14);
        } else if (strncmp(line, "property", 8) == 0) {
            PlyProperty prop;
            char* p = line + 9;
            // property [type] [name]
            if (strncmp(p, "float ", 6) == 0) {
                prop.isFloat = true;
                p += 6;
            } else if (strncmp(p, "double ", 7) == 0) {
                prop.isDouble = true;
                p += 7;
            } else if (strncmp(p, "int ", 4) == 0) {
                prop.isInt = true;
                p += 4;
            } else if (strncmp(p, "uchar ", 6) == 0) {
                prop.isUChar = true;
                p += 6;
            } else {
                continue;
            }
            while (*p && std::isspace(*p)) ++p;
            char* end = p;
            while (*end && !std::isspace(*end) && *end != '\n') ++end;
            *end = '\0';
            prop.name = p;
            header.props.push_back(prop);
        } else if (strncmp(line, "end_header", 10) == 0) {
            break;
        }
    }
    return header;
}

bool readBinaryValue(std::FILE* f, void* dst, size_t size) {
    return std::fread(dst, size, 1, f) == 1;
}

} // anonymous namespace

PointCloud PlyIO::read(const std::string& filepath) {
    return read(filepath, ParseOptions());
}

PointCloud PlyIO::read(const std::string& filepath, ParseOptions opts) {
    std::FILE* f = std::fopen(filepath.c_str(), "rb");
    if (!f) return {};

    PlyHeader header = parseHeader(f);
    if (header.vertexCount == 0) {
        std::fclose(f);
        return {};
    }

    // 找到属性索引
    int ix = -1, iy = -1, iz = -1, inx = -1, iny = -1, inz = -1;
    int ir = -1, ig = -1, ib = -1;
    for (int i = 0; i < static_cast<int>(header.props.size()); ++i) {
        const auto& p = header.props[i].name;
        if (p == "x") ix = i;
        else if (p == "y") iy = i;
        else if (p == "z") iz = i;
        else if (p == "nx") inx = i;
        else if (p == "ny") iny = i;
        else if (p == "nz") inz = i;
        else if (p == "red" || p == "r") ir = i;
        else if (p == "green" || p == "g") ig = i;
        else if (p == "blue" || p == "b") ib = i;
    }

    if (ix < 0 || iy < 0 || iz < 0) {
        std::fclose(f);
        return {};
    }

    std::vector<int> propIndices = {ix, iy, iz, inx, iny, inz, ir, ig, ib};

    PointCloud cloud(header.vertexCount);
    cloud.hasNormals = (inx >= 0 && iny >= 0 && inz >= 0);

    if (header.format == Format::Ascii) {
        cloud = parseAscii(f, header.vertexCount, propIndices, opts);
    } else {
        cloud = parseBinary(f, header.vertexCount, propIndices, opts);
    }

    std::fclose(f);
    return cloud;
}

PointCloud PlyIO::parseAscii(std::FILE* f, size_t vertexCount,
                             const std::vector<int>& propIndices,
                             ParseOptions opts) {
    int ix = propIndices[0], iy = propIndices[1], iz = propIndices[2];
    int inx = propIndices[3], iny = propIndices[4], inz = propIndices[5];
    int ir = propIndices[6], ig = propIndices[7], ib = propIndices[8];

    PointCloud cloud(vertexCount);
    cloud.hasNormals = (inx >= 0 && iny >= 0 && inz >= 0);
    const bool hasColors = (ir >= 0 && ig >= 0 && ib >= 0);
    if (hasColors) {
        cloud.red.reserve(vertexCount);
        cloud.green.reserve(vertexCount);
        cloud.blue.reserve(vertexCount);
    }

    char line[4096];
    size_t count = 0;

    while (count < vertexCount && fgets(line, sizeof(line), f)) {
        std::vector<double> vals;
        char* token = std::strtok(line, " \t\r\n");
        while (token) {
            vals.push_back(std::atof(token));
            token = std::strtok(nullptr, " \t\r\n");
        }

        if ((int)vals.size() <= std::max({ix, iy, iz})) continue;

        double x = vals[ix], y = vals[iy], z = vals[iz];
        if (opts.flipYZ) std::swap(y, z);

        double nx = 0, ny = 0, nz = 0;
        if (inx >= 0 && iny >= 0 && inz >= 0 && (int)vals.size() > inz) {
            nx = vals[inx]; ny = vals[iny]; nz = vals[inz];
        }

        cloud[count++] = Point3D(x, y, z, nx, ny, nz);
        if (hasColors && (int)vals.size() > std::max({ir, ig, ib})) {
            cloud.red.push_back(static_cast<std::uint8_t>(std::clamp(vals[ir], 0.0, 255.0)));
            cloud.green.push_back(static_cast<std::uint8_t>(std::clamp(vals[ig], 0.0, 255.0)));
            cloud.blue.push_back(static_cast<std::uint8_t>(std::clamp(vals[ib], 0.0, 255.0)));
        }
    }

    return cloud;
}

PointCloud PlyIO::parseBinary(std::FILE* f, size_t vertexCount,
                              const std::vector<int>& propIndices,
                              ParseOptions opts) {
    int ix = propIndices[0], iy = propIndices[1], iz = propIndices[2];
    int inx = propIndices[3], iny = propIndices[4], inz = propIndices[5];
    int ir = propIndices[6], ig = propIndices[7], ib = propIndices[8];

    // Re-parse header to get property types
    std::rewind(f);
    auto header = parseHeader(f);

    PointCloud cloud(vertexCount);
    cloud.hasNormals = (inx >= 0 && iny >= 0 && inz >= 0);
    const bool hasColors = (ir >= 0 && ig >= 0 && ib >= 0);
    if (hasColors) {
        cloud.red.resize(vertexCount);
        cloud.green.resize(vertexCount);
        cloud.blue.resize(vertexCount);
    }

    std::vector<double> valBuffer(header.props.size());

    for (size_t count = 0; count < vertexCount && !std::feof(f); ++count) {
        for (size_t j = 0; j < header.props.size(); ++j) {
            const auto& prop = header.props[j];
            if (prop.isFloat) {
                float v;
                if (!readBinaryValue(f, &v, sizeof(float))) {
                    return cloud;
                }
                valBuffer[j] = static_cast<double>(v);
            } else if (prop.isDouble) {
                if (!readBinaryValue(f, &valBuffer[j], sizeof(double))) {
                    return cloud;
                }
            } else if (prop.isInt) {
                int v;
                if (!readBinaryValue(f, &v, sizeof(int))) {
                    return cloud;
                }
                valBuffer[j] = static_cast<double>(v);
            } else if (prop.isUChar) {
                unsigned char v;
                if (!readBinaryValue(f, &v, sizeof(unsigned char))) {
                    return cloud;
                }
                valBuffer[j] = static_cast<double>(v);
            }
        }

        double x = valBuffer[ix], y = valBuffer[iy], z = valBuffer[iz];
        if (opts.flipYZ) std::swap(y, z);

        double nx = 0, ny = 0, nz = 0;
        if (inx >= 0) nx = valBuffer[inx];
        if (iny >= 0) ny = valBuffer[iny];
        if (inz >= 0) nz = valBuffer[inz];

        cloud[count] = Point3D(x, y, z, nx, ny, nz);
        if (hasColors) {
            cloud.red[count] = static_cast<std::uint8_t>(std::clamp(valBuffer[ir], 0.0, 255.0));
            cloud.green[count] = static_cast<std::uint8_t>(std::clamp(valBuffer[ig], 0.0, 255.0));
            cloud.blue[count] = static_cast<std::uint8_t>(std::clamp(valBuffer[ib], 0.0, 255.0));
        }
    }

    return cloud;
}

void PlyIO::write(const PointCloud& cloud, const std::string& filepath, bool asBinary) {
    std::FILE* f = std::fopen(filepath.c_str(), "wb");
    if (!f) return;

    size_t n = cloud.size();
    const char* fmt = asBinary ? "binary_little_endian" : "ascii";

    std::fprintf(f, "ply\nformat %s 1.0\n", fmt);
    std::fprintf(f, "element vertex %zu\n", n);
    std::fprintf(f, "property double x\nproperty double y\nproperty double z\n");
    if (cloud.hasNormals) {
        std::fprintf(f, "property double nx\nproperty double ny\nproperty double nz\n");
    }
    if (cloud.hasColors()) {
        std::fprintf(f, "property uchar red\nproperty uchar green\nproperty uchar blue\n");
    }
    std::fprintf(f, "end_header\n");

    if (asBinary) {
        for (size_t i = 0; i < n; ++i) {
            const auto& p = cloud[i];
            std::fwrite(&p.x, sizeof(double), 1, f);
            std::fwrite(&p.y, sizeof(double), 1, f);
            std::fwrite(&p.z, sizeof(double), 1, f);
            if (cloud.hasNormals) {
                std::fwrite(&p.nx, sizeof(double), 1, f);
                std::fwrite(&p.ny, sizeof(double), 1, f);
                std::fwrite(&p.nz, sizeof(double), 1, f);
            }
            if (cloud.hasColors()) {
                std::fwrite(&cloud.red[i], sizeof(std::uint8_t), 1, f);
                std::fwrite(&cloud.green[i], sizeof(std::uint8_t), 1, f);
                std::fwrite(&cloud.blue[i], sizeof(std::uint8_t), 1, f);
            }
        }
    } else {
        for (size_t i = 0; i < n; ++i) {
            const auto& p = cloud[i];
            if (cloud.hasNormals && cloud.hasColors()) {
                std::fprintf(f, "%.17g %.17g %.17g %.17g %.17g %.17g %u %u %u\n",
                    p.x, p.y, p.z, p.nx, p.ny, p.nz,
                    static_cast<unsigned>(cloud.red[i]),
                    static_cast<unsigned>(cloud.green[i]),
                    static_cast<unsigned>(cloud.blue[i]));
            } else if (cloud.hasNormals) {
                std::fprintf(f, "%.17g %.17g %.17g %.17g %.17g %.17g\n",
                    p.x, p.y, p.z, p.nx, p.ny, p.nz);
            } else if (cloud.hasColors()) {
                std::fprintf(f, "%.17g %.17g %.17g %u %u %u\n",
                    p.x, p.y, p.z,
                    static_cast<unsigned>(cloud.red[i]),
                    static_cast<unsigned>(cloud.green[i]),
                    static_cast<unsigned>(cloud.blue[i]));
            } else {
                std::fprintf(f, "%.17g %.17g %.17g\n", p.x, p.y, p.z);
            }
        }
    }

    std::fclose(f);
}

std::vector<double> PlyIO::readWeights(const std::string& filepath) {
    std::vector<double> weights;
    std::FILE* f = std::fopen(filepath.c_str(), "r");
    if (!f) return weights;

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        char* end = line;
        while (*end && (*end == ' ' || *end == '\t')) ++end;
        if (*end == '\n' || *end == '\r' || *end == '#' || *end == '\0') continue;
        double v = std::atof(end);
        weights.push_back(v);
    }

    std::fclose(f);
    return weights;
}
