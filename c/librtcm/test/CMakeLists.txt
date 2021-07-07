swift_add_test(test-rtcm
  UNIT_TEST
  SRCS rtcm_decoder_tests.c
  LINK rtcm
  )
swift_set_compile_options(test-rtcm REMOVE -Wconversion ADD -Wno-discarded-qualifiers)

swift_add_test(test-sta
  UNIT_TEST
  SRCS sta_rtcm3_tests.c
  LINK rtcm
  )
swift_set_compile_options(test-sta REMOVE -Wconversion)

swift_add_test(test-rtcm-pedantic
  UNIT_TEST
  SRCS check_pedantic.cc
  LINK rtcm
)
swift_set_compile_options(test-rtcm-pedantic ADD -pedantic)
