cmake_minimum_required(VERSION 2.8.7)

add_library (gnss-convertors rtcm3_sbp.c)
target_link_libraries(gnss-convertors m librtcm sbp)
include_directories (${PROJECT_SOURCE_DIR}/include)
include_directories (${PROJECT_SOURCE_DIR}/interface)
include_directories (${PROJECT_SOURCE_DIR}/libsbp/c/include/libsbp)

