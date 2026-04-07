file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(basic STATIC ${srcs})

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "CGAL version: ${CGAL_VERSION}")
target_include_directories(basic PUBLIC include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(basic PUBLIC CGAL::CGAL spdlog::spdlog_header_only)
