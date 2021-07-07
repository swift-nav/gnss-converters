find_package(Threads)

# Used to generate absolute paths in the unit tests
set(RELATIVE_PATH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(config.h.in config.h)

set(TEST_SOURCE_FILES
  check_rtcm3_main.c
  check_rtcm3.c
  check_rtcm3_ssr.c
  check_rtcm3_time.c
  )

swift_add_test(test-rtcm3tosbp
  UNIT_TEST
  SRCS ${TEST_SOURCE_FILES}
  INCLUDE ${CMAKE_CURRENT_BINARY_DIR}
  LINK gnss_converters gnss_converters_extra check Threads::Threads
  )

swift_set_compile_options(test-rtcm3tosbp REMOVE -Wconversion -Wfloat-equal -Wswitch-enum)
if(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
  target_compile_definitions(test-rtcm3tosbp PRIVATE GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
endif()
