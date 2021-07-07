find_package(Threads)

# Used to generate absolute paths in the unit tests
set(RELATIVE_PATH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(config.h.in config.h)

set(TEST_SOURCE_FILES
  check_gnss_converters_main.c
  check_nmea.c
  check_nmea_gpths.c
  check_options.c
  check_time_truth.c
  )

swift_add_test(test-gnss-converters
  UNIT_TEST
  SRCS ${TEST_SOURCE_FILES}
  INCLUDE ${CMAKE_CURRENT_BINARY_DIR}
  LINK gnss_converters check Threads::Threads
  )

swift_set_compile_options(test-gnss-converters REMOVE -Wconversion -Wfloat-equal -Wswitch-enum)
if(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
  target_compile_definitions(test-gnss-converters PRIVATE GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
endif()
