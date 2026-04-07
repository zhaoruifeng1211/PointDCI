cmake_minimum_required(VERSION 3.10)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake;${CMAKE_MODULE_PATH}")

project(DCI LANGUAGES C CXX)

option(DCI_BUILD_MAIN "Build the public improved executable" ON)
option(DCI_BUILD_GENERATE "Build the research generate executable" OFF)
option(DCI_BUILD_EVALUATE "Build the evaluate executable" OFF)
option(DCI_BUILD_OPTIMIZATION "Build optimization support library" OFF)

add_subdirectory(module)

if(DCI_BUILD_MAIN)
    add_subdirectory(src)
endif()
