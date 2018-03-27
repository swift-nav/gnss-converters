cmake_minimum_required(VERSION 2.8.7)

if (CMAKE_CROSSCOMPILING)
    message(STATUS "Skipping unit tests, cross compiling")
else (CMAKE_CROSSCOMPILING)
    # Set relative path prefix to make unit test paths absolute
    set(RELATIVE_PATH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")
    # Write configuration to a templated header file.
    configure_file("../include/config.h.in"
            "${CMAKE_CURRENT_BINARY_DIR}/config.h")

    add_executable (gnss_converter_test rtcm3_sbp_test.c)

    include_directories (BEFORE ${CMAKE_CURRENT_BINARY_DIR})
    include_directories (BEFORE ${PROJECT_SOURCE_DIR}/libsbp/c/include/libsbp)
    include_directories (BEFORE ${PROJECT_SOURCE_DIR}/include)
    include_directories (BEFORE ${PROJECT_SOURCE_DIR}/librtcm/c/include)
    target_link_libraries(gnss_converter_test LINK_PUBLIC gnss-converters)
        
    add_custom_command(
        TARGET gnss_converter_test POST_BUILD
        COMMENT "Running unit tests"
        COMMAND gnss_converter_test
    )
endif(CMAKE_CROSSCOMPILING)
