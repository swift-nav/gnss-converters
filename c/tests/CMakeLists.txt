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
    )
swift_add_test(test-gnss-converters 
  POST_BUILD
  SRCS ${TEST_SOURCE_FILES}
  INCLUDE ${CMAKE_CURRENT_BINARY_DIR}
  LINK gnss_converters check Threads::Threads
  )

swift_set_language_standards(test-gnss-converters)
swift_set_compile_options(test-gnss-converters REMOVE -Wconversion -Wfloat-equal)
