add_test(
    NAME pointSample_help
    COMMAND pointSample --help
)

add_test(
    NAME pointSample_uniform_ascii
    COMMAND pointSample
        --input ${CMAKE_CURRENT_SOURCE_DIR}/data/minimal_ascii.ply
        --output ${CMAKE_CURRENT_BINARY_DIR}/sampled_ascii.ply
        --num 3
        --strategy uniform
        --binary false
)
