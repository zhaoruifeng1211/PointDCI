include(FetchContent)

find_path(CXXOPTS_INCLUDE_DIR cxxopts.hpp)

if(NOT CXXOPTS_INCLUDE_DIR AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../DCI/module/3rd/cxxopts-2.2.1/include/cxxopts.hpp")
    set(CXXOPTS_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../DCI/module/3rd/cxxopts-2.2.1/include")
endif()

if(NOT CXXOPTS_INCLUDE_DIR)
    FetchContent_Declare(
        cxxopts
        URL https://github.com/jarro2783/cxxopts/archive/refs/tags/v3.2.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(cxxopts)
    set(CXXOPTS_INCLUDE_DIR "${cxxopts_SOURCE_DIR}/include")
endif()
