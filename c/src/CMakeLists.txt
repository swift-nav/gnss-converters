cmake_minimum_required(VERSION 2.8.7)

add_library(gnss_converters rtcm3_sbp.c)
target_link_libraries(gnss_converters m sbp rtcm)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/src)

file(GLOB gnss_converters_HEADERS "${PROJECT_SOURCE_DIR}/include/*.h")

install(TARGETS gnss_converters DESTINATION lib${LIB_SUFFIX})
install(FILES ${gnss_converters_HEADERS} DESTINATION include)


