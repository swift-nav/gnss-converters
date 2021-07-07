find_package(Threads)

swift_add_test(test-utils
  UNIT_TEST
  SRCS check_utils.c
  INCLUDE ${CMAKE_CURRENT_BINARY_DIR}
  LINK gnss_converters_extra check Threads::Threads
  )

swift_set_compile_options(test-utils REMOVE -Wconversion -Wfloat-equal -Wswitch-enum)
if(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
  target_compile_definitions(test-utils PRIVATE GNSS_CONVERTERS_DISABLE_CRC_VALIDATION)
endif()
