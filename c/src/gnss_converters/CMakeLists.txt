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
  ubx_ephemeris/bds.c
  ubx_ephemeris/gal.c
  ubx_ephemeris/glo.c
  ubx_ephemeris/gps.c
  ubx_ephemeris/sbas.c
  ubx_sbp.c
)

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