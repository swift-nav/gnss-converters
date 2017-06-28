cmake_minimum_required(VERSION 2.8.7)

add_executable (gnss_convertor_test rtcm3_sbp_test.c)

include_directories (${PROJECT_SOURCE_DIR}/include)
target_link_libraries(gnss_convertor_test LINK_PUBLIC gnss-convertors)

add_custom_command(
        TARGET gnss_convertor_test POST_BUILD
        COMMENT "Running unit tests"
        COMMAND gnss_convertor_test
)
