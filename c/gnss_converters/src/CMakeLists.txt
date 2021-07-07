# object library is to allow single source file to use C11 standard
swift_add_library(gnss_converters_c11
  OBJECT
  C_STANDARD 11
  SOURCES
    time_truth.c
)

# this is done to avoid issues with opaque time_truth_t type
set_source_files_properties(time_truth.c PROPERTIES COMPILE_FLAGS -fno-strict-aliasing)

# since we are using OBJECT library, the BUILD_SHARED_LIBS variable does not
# alter how the sources are compiles (see: https://gitlab.kitware.com/cmake/cmake/-/issues/17356)
# as such, we will need to manually fix it up. the BUILD_SHARED_LIBS is used by
# Piksi. if we ever upgrade our C standards to C11, we should be able to
# removed this and the need for a OBJECT library.
if(BUILD_SHARED_LIBS)
  set_target_properties(gnss_converters_c11
    PROPERTIES
      POSITION_INDEPENDENT_CODE true
  )
endif()

target_include_directories(gnss_converters_c11 PUBLIC ${PROJECT_SOURCE_DIR}/gnss_converters/include)
target_link_libraries(gnss_converters_c11
  PUBLIC
    swiftnav::sbp
    swiftnav::swiftnav
)

swift_add_library(gnss_converters
  SOURCES
    common.c
    ixcom_sbp.c
    nmea.c
    options.c
    rtcm3_sbp.c
    rtcm3_sbp_ephemeris.c
    rtcm3_sbp_ssr.c
    rtcm3_utils.c
    sbp_nmea.c
    ubx_ephemeris/bds.c
    ubx_ephemeris/gal.c
    ubx_ephemeris/glo.c
    ubx_ephemeris/gps.c
    ephemeris/sbas.c
    ubx_sbp.c
    $<TARGET_OBJECTS:gnss_converters_c11>
  REMOVE_COMPILE_OPTIONS
    -Wconversion
    -Wstack-protector
)
add_library(swiftnav::gnss_converters ALIAS gnss_converters)

if(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
  target_compile_definitions(gnss_converters PRIVATE GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
endif()

target_compile_options(gnss_converters PRIVATE "-UNDEBUG")
target_link_libraries(gnss_converters PUBLIC m swiftnav sbp rtcm ubx ixcom)
target_include_directories(gnss_converters PUBLIC ${PROJECT_SOURCE_DIR}/gnss_converters/include)
target_include_directories(gnss_converters PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(gnss_converters PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ubx_ephemeris)

install(TARGETS gnss_converters DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/gnss_converters/include/gnss-converters
  DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/
)
