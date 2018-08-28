file(GLOB gnss_converters_HEADERS "${PROJECT_SOURCE_DIR}/include/*.h")

add_library(gnss_converters rtcm3_sbp.c rtcm3_sbp_ephemeris.c rtcm3_sbp_ssr.c)
target_link_libraries(gnss_converters m sbp rtcm swiftnav)

target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_executable(rtcm3tosbp EXCLUDE_FROM_ALL rtcm3tosbp.c)
target_link_libraries(rtcm3tosbp gnss_converters)

install(TARGETS gnss_converters DESTINATION lib${LIB_SUFFIX})
install(FILES ${gnss_converters_HEADERS} DESTINATION include)
