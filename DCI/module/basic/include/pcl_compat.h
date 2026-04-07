#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace pcl {

struct PointXYZ {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    PointXYZ() = default;
    PointXYZ(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct RGB {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    float rgb = 0.0f;

    RGB() = default;
    RGB(std::uint8_t r_, std::uint8_t g_, std::uint8_t b_) : r(r_), g(g_), b(b_) {}
};

struct Normal {
    float normal_x = 0.0f;
    float normal_y = 0.0f;
    float normal_z = 0.0f;
    float curvature = 0.0f;

    Normal() = default;
    Normal(float nx, float ny, float nz, float c = 0.0f)
        : normal_x(nx), normal_y(ny), normal_z(nz), curvature(c) {}
};

struct PointXYZRGB {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

struct PointXYZRGBNormal {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    float normal_x = 0.0f;
    float normal_y = 0.0f;
    float normal_z = 0.0f;
    float curvature = 0.0f;
};

struct PointXYZINormal {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    float normal_x = 0.0f;
    float normal_y = 0.0f;
    float normal_z = 0.0f;
    float curvature = 0.0f;
};

template <typename PointT>
struct PointCloud {
    using Ptr = std::shared_ptr<PointCloud<PointT>>;
    using iterator = typename std::vector<PointT>::iterator;
    using const_iterator = typename std::vector<PointT>::const_iterator;

    std::vector<PointT> points;
    std::uint32_t width = 0;
    std::uint32_t height = 1;
    bool is_dense = true;

    void push_back(const PointT& point) { points.push_back(point); }
    std::size_t size() const { return points.size(); }
    PointT& at(std::size_t index) { return points.at(index); }
    const PointT& at(std::size_t index) const { return points.at(index); }
    PointT& operator[](std::size_t index) { return points[index]; }
    const PointT& operator[](std::size_t index) const { return points[index]; }
    iterator begin() { return points.begin(); }
    iterator end() { return points.end(); }
    const_iterator begin() const { return points.begin(); }
    const_iterator end() const { return points.end(); }
};

namespace io {
namespace detail {

struct PlyProperty {
    std::string type;
    std::string name;
};

struct PlyHeader {
    bool ascii = true;
    bool binary_little_endian = false;
    std::size_t vertex_count = 0;
    std::vector<PlyProperty> properties;
};

inline std::string trim(std::string value)
{
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
        value.pop_back();
    }
    std::size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
        ++start;
    }
    return value.substr(start);
}

inline std::string lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

inline bool parse_header(std::FILE* file, PlyHeader& header)
{
    char line[512];
    bool in_vertex_element = false;
    bool saw_ply = false;
    while (std::fgets(line, sizeof(line), file)) {
        std::string text = trim(line);
        if (text.empty()) {
            continue;
        }
        if (!saw_ply) {
            saw_ply = (text == "ply");
            if (!saw_ply) {
                return false;
            }
            continue;
        }
        if (text == "end_header") {
            return header.vertex_count > 0;
        }

        char keyword[64] = {0};
        if (std::sscanf(text.c_str(), "%63s", keyword) != 1) {
            continue;
        }

        std::string kw = lower(keyword);
        if (kw == "format") {
            if (text.find("ascii") != std::string::npos) {
                header.ascii = true;
                header.binary_little_endian = false;
            } else if (text.find("binary_little_endian") != std::string::npos) {
                header.ascii = false;
                header.binary_little_endian = true;
            } else {
                return false;
            }
        } else if (kw == "element") {
            char element_name[64] = {0};
            unsigned long long count = 0;
            if (std::sscanf(text.c_str(), "element %63s %llu", element_name, &count) == 2) {
                in_vertex_element = (std::string(element_name) == "vertex");
                if (in_vertex_element) {
                    header.vertex_count = static_cast<std::size_t>(count);
                }
            }
        } else if (kw == "property" && in_vertex_element) {
            char type[64] = {0};
            char name[64] = {0};
            if (std::sscanf(text.c_str(), "property %63s %63s", type, name) == 2) {
                header.properties.push_back({lower(type), lower(name)});
            }
        }
    }
    return false;
}

inline bool property_matches(const std::string& property_name, const std::initializer_list<const char*> aliases)
{
    for (const char* alias : aliases) {
        if (property_name == alias) {
            return true;
        }
    }
    return false;
}

template <typename Number>
inline bool read_binary(std::FILE* file, Number& value)
{
    return std::fread(&value, sizeof(Number), 1, file) == 1;
}

inline bool read_scalar(std::FILE* file, const std::string& type, double& value)
{
    if (type == "float" || type == "float32") {
        float v = 0.0f;
        if (!read_binary(file, v)) return false;
        value = static_cast<double>(v);
        return true;
    }
    if (type == "double" || type == "float64") {
        return read_binary(file, value);
    }
    if (type == "uchar" || type == "uint8") {
        std::uint8_t v = 0;
        if (!read_binary(file, v)) return false;
        value = static_cast<double>(v);
        return true;
    }
    if (type == "char" || type == "int8") {
        std::int8_t v = 0;
        if (!read_binary(file, v)) return false;
        value = static_cast<double>(v);
        return true;
    }
    if (type == "ushort" || type == "uint16") {
        std::uint16_t v = 0;
        if (!read_binary(file, v)) return false;
        value = static_cast<double>(v);
        return true;
    }
    if (type == "short" || type == "int16") {
        std::int16_t v = 0;
        if (!read_binary(file, v)) return false;
        value = static_cast<double>(v);
        return true;
    }
    if (type == "uint" || type == "uint32") {
        std::uint32_t v = 0;
        if (!read_binary(file, v)) return false;
        value = static_cast<double>(v);
        return true;
    }
    if (type == "int" || type == "int32") {
        std::int32_t v = 0;
        if (!read_binary(file, v)) return false;
        value = static_cast<double>(v);
        return true;
    }
    return false;
}

template <typename PointT>
inline void assign_common_fields(const std::vector<double>& values,
                                 const std::vector<PlyProperty>& properties,
                                 PointT& point)
{
    for (std::size_t i = 0; i < properties.size() && i < values.size(); ++i) {
        const std::string& name = properties[i].name;
        const double value = values[i];
        if (name == "x") point.x = static_cast<float>(value);
        else if (name == "y") point.y = static_cast<float>(value);
        else if (name == "z") point.z = static_cast<float>(value);
    }
}

template <typename PointT>
inline void assign_extra_fields(const std::vector<double>&, const std::vector<PlyProperty>&, PointT&)
{
}

template <>
inline void assign_extra_fields<PointXYZRGBNormal>(const std::vector<double>& values,
                                                   const std::vector<PlyProperty>& properties,
                                                   PointXYZRGBNormal& point)
{
    for (std::size_t i = 0; i < properties.size() && i < values.size(); ++i) {
        const std::string& name = properties[i].name;
        const double value = values[i];
        if (property_matches(name, {"red", "r"})) point.r = static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
        else if (property_matches(name, {"green", "g"})) point.g = static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
        else if (property_matches(name, {"blue", "b"})) point.b = static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
        else if (property_matches(name, {"nx", "normal_x"})) point.normal_x = static_cast<float>(value);
        else if (property_matches(name, {"ny", "normal_y"})) point.normal_y = static_cast<float>(value);
        else if (property_matches(name, {"nz", "normal_z"})) point.normal_z = static_cast<float>(value);
        else if (name == "curvature") point.curvature = static_cast<float>(value);
    }
}

template <typename PointT>
inline void assign_point(const std::vector<double>& values,
                         const std::vector<PlyProperty>& properties,
                         PointT& point)
{
    assign_common_fields(values, properties, point);
    assign_extra_fields(values, properties, point);
}

template <typename PointT>
inline int load_impl(const std::string& path, PointCloud<PointT>& cloud)
{
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) {
        return -1;
    }

    PlyHeader header;
    if (!parse_header(file, header)) {
        std::fclose(file);
        return -1;
    }
    if (!header.ascii && !header.binary_little_endian) {
        std::fclose(file);
        return -1;
    }

    cloud.points.clear();
    cloud.points.reserve(header.vertex_count);
    cloud.width = static_cast<std::uint32_t>(header.vertex_count);
    cloud.height = 1;
    cloud.is_dense = true;

    if (header.ascii) {
        char line[8192];
        while (cloud.points.size() < header.vertex_count && std::fgets(line, sizeof(line), file)) {
            std::vector<double> values;
            char* token = std::strtok(line, " \t\r\n");
            while (token) {
                values.push_back(std::atof(token));
                token = std::strtok(nullptr, " \t\r\n");
            }
            if (values.size() < 3) {
                continue;
            }
            PointT point{};
            assign_point(values, header.properties, point);
            cloud.points.push_back(point);
        }
    } else {
        std::vector<double> values(header.properties.size(), 0.0);
        while (cloud.points.size() < header.vertex_count) {
            bool ok = true;
            for (std::size_t i = 0; i < header.properties.size(); ++i) {
                ok = read_scalar(file, header.properties[i].type, values[i]);
                if (!ok) {
                    break;
                }
            }
            if (!ok) {
                break;
            }
            PointT point{};
            assign_point(values, header.properties, point);
            cloud.points.push_back(point);
        }
    }

    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    std::fclose(file);
    return cloud.points.empty() ? -1 : 0;
}

template <typename PointT>
struct Writer;

template <>
struct Writer<PointXYZ> {
    static void write_header(std::FILE* file, std::size_t count)
    {
        std::fprintf(file,
                     "ply\nformat ascii 1.0\n"
                     "element vertex %zu\n"
                     "property float x\nproperty float y\nproperty float z\n"
                     "end_header\n",
                     count);
    }

    static void write_point(std::FILE* file, const PointXYZ& point)
    {
        std::fprintf(file, "%.9g %.9g %.9g\n", point.x, point.y, point.z);
    }
};

template <>
struct Writer<PointXYZRGB> {
    static void write_header(std::FILE* file, std::size_t count)
    {
        std::fprintf(file,
                     "ply\nformat ascii 1.0\n"
                     "element vertex %zu\n"
                     "property float x\nproperty float y\nproperty float z\n"
                     "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                     "end_header\n",
                     count);
    }

    static void write_point(std::FILE* file, const PointXYZRGB& point)
    {
        std::fprintf(file, "%.9g %.9g %.9g %u %u %u\n",
                     point.x, point.y, point.z,
                     static_cast<unsigned>(point.r),
                     static_cast<unsigned>(point.g),
                     static_cast<unsigned>(point.b));
    }
};

template <>
struct Writer<PointXYZINormal> {
    static void write_header(std::FILE* file, std::size_t count)
    {
        std::fprintf(file,
                     "ply\nformat ascii 1.0\n"
                     "element vertex %zu\n"
                     "property float x\nproperty float y\nproperty float z\n"
                     "property float intensity\n"
                     "property float normal_x\nproperty float normal_y\nproperty float normal_z\n"
                     "property float curvature\n"
                     "end_header\n",
                     count);
    }

    static void write_point(std::FILE* file, const PointXYZINormal& point)
    {
        std::fprintf(file, "%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
                     point.x, point.y, point.z, point.intensity,
                     point.normal_x, point.normal_y, point.normal_z, point.curvature);
    }
};

template <typename PointT>
inline int save_impl(const std::string& path, const PointCloud<PointT>& cloud)
{
    std::FILE* file = std::fopen(path.c_str(), "wb");
    if (!file) {
        return -1;
    }
    Writer<PointT>::write_header(file, cloud.points.size());
    for (const auto& point : cloud.points) {
        Writer<PointT>::write_point(file, point);
    }
    std::fclose(file);
    return 0;
}

}  // namespace detail

template <typename PointT>
inline int loadPLYFile(const std::string& path, PointCloud<PointT>& cloud)
{
    return detail::load_impl(path, cloud);
}

template <typename PointT>
inline int savePLYFile(const std::string& path, const PointCloud<PointT>& cloud)
{
    return detail::save_impl(path, cloud);
}

}  // namespace io
}  // namespace pcl
