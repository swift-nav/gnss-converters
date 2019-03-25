set(gnss_converters_HEADERS
  ${PROJECT_SOURCE_DIR}/include/gnss-converters/nmea.h
  ${PROJECT_SOURCE_DIR}/include/gnss-converters/rtcm3_sbp.h
  ${PROJECT_SOURCE_DIR}/include/gnss-converters/sbp_nmea.h
  )

add_library(gnss_converters rtcm3_sbp.c rtcm3_sbp_ephemeris.c rtcm3_sbp_ssr.c sbp_nmea.c nmea.c rtcm3_msm_utils.c sbp_conv.c)
target_link_libraries(gnss_converters m swiftnav sbp rtcm)

target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_executable(rtcm3tosbp rtcm3tosbp.c)
target_link_libraries(rtcm3tosbp gnss_converters)

add_executable(sbp2rtcm sbp2rtcm.c)
target_link_libraries(sbp2rtcm gnss_converters)

install(TARGETS gnss_converters DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS rtcm3tosbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
install(TARGETS sbp2rtcm DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
install(FILES ${gnss_converters_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/gnss-converters)
