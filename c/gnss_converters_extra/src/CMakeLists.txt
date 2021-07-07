swift_add_library(gnss_converters_extra
  SOURCES
    sbp_conv.c
    sbp_rtcm3.c
    sbp_rtcm3_ephemeris.c
  REMOVE_COMPILE_OPTIONS
    -pedantic
    -Wconversion
)
add_library(swiftnav::gnss_converters_extra ALIAS gnss_converters_extra)

target_compile_options(gnss_converters_extra PRIVATE "-UNDEBUG")

target_link_libraries(gnss_converters_extra PUBLIC gnss_converters rtcm sbp swiftnav)
target_include_directories(gnss_converters_extra PUBLIC ${PROJECT_SOURCE_DIR}/gnss_converters_extra/include)

install(TARGETS gnss_converters_extra DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/gnss_converters_extra/include/gnss-converters-extra
  DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/
)
