if (MSVC)
    add_compile_options(/utf-8)
endif()

add_subdirectory(3rd/spdlog)
add_subdirectory(basic)
add_subdirectory(energyTerm)

if(DCI_BUILD_OPTIMIZATION)
    add_subdirectory(optimization)
endif()

if(DCI_BUILD_GENERATE)
    add_subdirectory(generate)
endif()

if(DCI_BUILD_EVALUATE)
    add_subdirectory(evaluate)
endif()
