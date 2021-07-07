set(SRCS
    binary_header.cc
    crc_checker.cc
    message_gpsephem.cc
    message_bestpos.cc
    message_bestvel.cc
    message_rangecmp.cc
    message_insatt.cc
    message_gloephemeris.cc
    message_rawimusx.cc
    parser.cc)

set(TEST_SRCS
    catch/catch.cpp
    test_read_little_endian.cc
    test_parser.cc)

################################################################################
# Enumerate targets.
################################################################################
swift_add_library(novatel-parser
  SOURCES ${SRCS}
  ADD_COMPILE_OPTIONS
    ${STARLING_COMPILE_OPTIONS}
    -Wno-strict-aliasing
    -Wno-long-long
    -Wno-effc++
    -Wno-conversion
  REMOVE_COMPILE_OPTIONS
    -pedantic
)
add_library(swiftnav::novatel-parser ALIAS novatel-parser)

target_include_directories(novatel-parser PUBLIC ${PROJECT_SOURCE_DIR}/nov2sbp)
target_link_libraries(novatel-parser PUBLIC swiftnav)

install(TARGETS novatel-parser DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})

################################################################################
# Tests. 
################################################################################
swift_add_test(test-novatel-parser
  UNIT_TEST
  SRCS ${TEST_SRCS}
)
swift_set_compile_options(test-novatel-parser
  EXCEPTIONS
  ADD
    ${STARLING_COMPILE_OPTIONS}
    -Wno-strict-aliasing
    -Wno-long-long
    -Wno-effc++
    -Wno-conversion
)
target_link_libraries(test-novatel-parser PRIVATE novatel-parser)
target_include_directories(test-novatel-parser PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(test-novatel-parser PRIVATE
    UNIT_TEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../data")

add_custom_target(run_test_novatel_parser
    COMMAND test-novatel-parser
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMENT "Running Novatel parser tests.")


