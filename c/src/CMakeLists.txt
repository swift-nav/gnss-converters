set(gnss_converters_HEADERS
  ${PROJECT_SOURCE_DIR}/include/gnss-converters/nmea.h
  ${PROJECT_SOURCE_DIR}/include/gnss-converters/rtcm3_sbp.h
  ${PROJECT_SOURCE_DIR}/include/gnss-converters/sbp_rtcm3.h
  ${PROJECT_SOURCE_DIR}/include/gnss-converters/sbp_nmea.h
  )

add_library(gnss_converters rtcm3_sbp.c sbp_rtcm3.c rtcm3_sbp_ephemeris.c rtcm3_sbp_ssr.c sbp_nmea.c nmea.c rtcm3_utils.c sbp_conv.c sbp_rtcm3_ephemeris.c)
target_link_libraries(gnss_converters PUBLIC m swiftnav sbp rtcm)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_code_coverage(gnss_converters AUTO ALL)

add_executable(rtcm3tosbp rtcm3tosbp.c)
target_link_libraries(rtcm3tosbp PRIVATE gnss_converters)
target_code_coverage(rtcm3tosbp AUTO ALL)

add_executable(sbp2rtcm sbp2rtcm.c)
target_link_libraries(sbp2rtcm PRIVATE gnss_converters)
target_code_coverage(sbp2rtcm AUTO ALL)

install(TARGETS gnss_converters DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS rtcm3tosbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
install(TARGETS sbp2rtcm DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
install(FILES ${gnss_converters_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/gnss-converters)
