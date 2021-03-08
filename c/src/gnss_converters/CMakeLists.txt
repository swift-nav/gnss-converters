add_library(gnss_converters
  common.c
  ixcom_sbp.c
  nmea.c
  options.c
  rtcm3_sbp.c
  rtcm3_sbp_ephemeris.c
  rtcm3_sbp_ssr.c
  rtcm3_utils.c
  sbp_nmea.c
  sbp_rtcm3.c
  sbp_rtcm3_ephemeris.c
  time_truth.c
  ubx_ephemeris/bds.c
  ubx_ephemeris/gal.c
  ubx_ephemeris/glo.c
  ubx_ephemeris/gps.c
  ephemeris/sbas.c
  ubx_sbp.c
)


if(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
  target_compile_definitions(gnss_converters PRIVATE GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
endif()
# this is done to avoid issues with opaque time_truth_t type
set_source_files_properties(time_truth.c PROPERTIES COMPILE_FLAGS -fno-strict-aliasing)

swift_set_language_standards(gnss_converters)
swift_set_compile_options(gnss_converters REMOVE -Wconversion -Wstack-protector)
target_compile_options(gnss_converters PRIVATE "-UNDEBUG")

target_link_libraries(gnss_converters PUBLIC m swiftnav sbp rtcm ubx ixcom)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(gnss_converters PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(gnss_converters PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ubx_ephemeris)
target_code_coverage(gnss_converters AUTO ALL)

install(TARGETS gnss_converters DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/gnss-converters
  DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/
)
