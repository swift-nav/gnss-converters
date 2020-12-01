add_library(gnss_converters_extra
  sbp_conv.c
)

swift_set_language_standards(gnss_converters_extra)
swift_set_compile_options(gnss_converters_extra)
target_compile_options(gnss_converters_extra PRIVATE "-UNDEBUG")

target_link_libraries(gnss_converters_extra PUBLIC gnss_converters rtcm sbp swiftnav)
target_include_directories(gnss_converters_extra PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_code_coverage(gnss_converters_extra AUTO ALL)

install(TARGETS gnss_converters_extra DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/gnss-converters-extra
  DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/
)
