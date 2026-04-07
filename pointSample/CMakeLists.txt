cmake_minimum_required(VERSION 3.16)
project(pointSample VERSION 1.0 LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OpenMP
find_package(OpenMP REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# Third-party headers
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/fetch_cxxopts.cmake)

# Project sources
set(SOURCES
    src/main.cpp
    src/core/PointCloud.cpp
    src/core/KDTree.cpp
    src/core/FPSSampler.cpp
    src/weighting/WeightStrategy.cpp
    src/weighting/ColorIntensityWeight.cpp
    src/weighting/UniformWeight.cpp
    src/weighting/CurvatureWeight.cpp
    src/weighting/DensityWeight.cpp
    src/weighting/PercentileGatedWeight.cpp
    src/weighting/ConsistencyStabilityWeight.cpp
    src/weighting/ConsistencyWeight.cpp
    src/weighting/StabilityWeight.cpp
    src/io/PlyIO.cpp
    src/io/ObjIO.cpp
    src/downsampling/Downsampler.cpp
    src/downsampling/OctreeDownsampler.cpp
)

add_executable(pointSample ${SOURCES})
target_include_directories(pointSample PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${CXXOPTS_INCLUDE_DIR}
)
target_link_libraries(pointSample PRIVATE Eigen3::Eigen OpenMP::OpenMP_CXX)

# Testing
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
