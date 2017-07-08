cmake_minimum_required(VERSION 2.8.7)

if (CMAKE_CROSSCOMPILING)
    message(STATUS "Skipping unit tests, cross compiling")
else (CMAKE_CROSSCOMPILING)
    add_executable (gnss_convertor_test rtcm3_sbp_test.c)

    include_directories (BEFORE ${PROJECT_SOURCE_DIR}/libsbp/c/include/libsbp)
    include_directories (BEFORE ${PROJECT_SOURCE_DIR}/include)
    include_directories (BEFORE ${PROJECT_SOURCE_DIR}/librtcm/c/include)
    target_link_libraries(gnss_convertor_test LINK_PUBLIC gnss-convertors)

    add_custom_command(
        TARGET gnss_convertor_test POST_BUILD
        COMMENT "Running unit tests"
        COMMAND gnss_convertor_test
    )
endif(CMAKE_CROSSCOMPILING)
