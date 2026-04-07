#pragma once

#include "../core/PointCloud.h"
#include <string>
#include <vector>
#include <cstddef>

/**
 * @brief PLY 文件读写
 *
 * 支持格式:
 *   - ascii PLY (text)
 *   - binary_little_endian PLY
 *
 * 支持属性:
 *   - x, y, z (必需)
 *   - nx, ny, nz (如果存在则读取)
 *   - red, green, blue / r, g, b
 */
class PlyIO {
public:
    enum class Format { Ascii, BinaryLE };

    struct ParseOptions {
        bool flipYZ = false;   // OpenGL -> Right-handed coord system
        bool normalize = false;
    };

    static PointCloud read(const std::string& filepath);
    static PointCloud read(const std::string& filepath, ParseOptions opts);
    static void write(const PointCloud& cloud, const std::string& filepath, bool asBinary = true);

    /**
     * @brief 读取权重数据文件（每行一个浮点数）
     * @param filepath 权重文件路径
     * @return 权重值列表
     */
    static std::vector<double> readWeights(const std::string& filepath);

private:
    static PointCloud parseAscii(std::FILE* f, size_t vertexCount,
                                  const std::vector<int>& propIndices,
                                  ParseOptions opts);
    static PointCloud parseBinary(std::FILE* f, size_t vertexCount,
                                   const std::vector<int>& propIndices,
                                   ParseOptions opts);
};
