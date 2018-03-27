cmake_minimum_required(VERSION 2.8.7)

include_directories (BEFORE ${PROJECT_SOURCE_DIR}/libsbp/c/include)
include_directories (BEFORE ${PROJECT_SOURCE_DIR}/include)
include_directories (BEFORE ${PROJECT_SOURCE_DIR}/librtcm/c/include)

add_library (gnss-converters rtcm3_sbp.c)
target_link_libraries(gnss-converters m sbp rtcm)

file(GLOB gnss-converters_HEADERS "${PROJECT_SOURCE_DIR}/include/*.h")

install(TARGETS gnss-converters DESTINATION lib${LIB_SUFFIX})
install(FILES ${gnss-converters_HEADERS} DESTINATION include)


