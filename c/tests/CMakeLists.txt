find_package(Threads)

# Used to generate absolute paths in the unit tests
set(RELATIVE_PATH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(config.h.in config.h)

set(TEST_SOURCE_FILES
    check_main.c
    check_rtcm3.c
    check_rtcm3_ssr.c
    check_nmea.c
    check_ubx.c
    check_ixcom.c
    check_utils.c
    check_options.c
    check_rtcm3_time.c
    check_time_truth.c
    )
swift_add_test(test-gnss-converters
  UNIT_TEST
  POST_BUILD
  SRCS ${TEST_SOURCE_FILES}
  INCLUDE ${CMAKE_CURRENT_BINARY_DIR}
  LINK gnss_converters check Threads::Threads
  )

swift_set_language_standards(test-gnss-converters)
swift_set_compile_options(test-gnss-converters REMOVE -Wconversion -Wfloat-equal)
if(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
  target_compile_definitions(test-gnss-converters PRIVATE GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
endif()

add_subdirectory(afl)
