cmake_minimum_required(VERSION 2.8.7)

if(CMAKE_CROSSCOMPILING)
    message(STATUS "Skipping unit tests, cross compiling")
    return()
endif()

# The C unit tests are dependent on the Check framework.
find_package(Check)
if(NOT CHECK_FOUND)
    message(STATUS "Skipping unit tests, Check library not found!")
    return()
endif()

set(TEST_SOURCE_FILES
    check_main.c
    check_rtcm3.c
    check_rtcm3_ssr.c
    )
add_executable(test_gnss_converters ${TEST_SOURCE_FILES})

# Set relative path prefix to make unit test paths absolute
set(RELATIVE_PATH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")
# Set config locaation
set(CONFIG_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/config.h")
# Write configuration to a templated header file.
configure_file("../include/config.h.in"
               ${CONFIG_LOCATION})

target_include_directories(test_gnss_converters PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_gnss_converters gnss_converters ${CHECK_LIBRARIES} pthread)
add_custom_command(
    TARGET test_gnss_converters POST_BUILD
    COMMENT "Running unit tests"
    COMMAND test_gnss_converters
    )
