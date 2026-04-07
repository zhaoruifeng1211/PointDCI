find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_executable(evaluate ${srcs})
target_include_directories(evaluate
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../basic
        ${CMAKE_CURRENT_SOURCE_DIR}/../../src  # Path to globInfo.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/cxxopts-2.2.1/include
)
target_link_libraries(evaluate PUBLIC basic energyTerm OpenMP::OpenMP_CXX Eigen3::Eigen)
