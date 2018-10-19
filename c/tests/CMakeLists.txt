cmake_minimum_required(VERSION 2.8.7)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

if(CMAKE_CROSSCOMPILING)
    message(STATUS "Skipping unit tests, cross compiling")
    return()
endif()

find_package(Check)
if(NOT CHECK_FOUND)
    message(STATUS "Skipping unit tests, Check library not found!")
    return()
endif()

set(TEST_SOURCE_FILES
    check_main.c
    check_rtcm3.c
    check_rtcm3_ssr.c
    check_nmea.c
    check_utils.c
    )
add_executable(test_gnss_converters ${TEST_SOURCE_FILES})

# Used to generate absolute paths in the unit tests
set(RELATIVE_PATH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(config.h.in config.h)

target_include_directories(test_gnss_converters PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_gnss_converters gnss_converters ${CHECK_LIBRARIES} pthread)
add_custom_command(
    TARGET test_gnss_converters POST_BUILD
    COMMENT "Running unit tests"
    COMMAND test_gnss_converters
    )
