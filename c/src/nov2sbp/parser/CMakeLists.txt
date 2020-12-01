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
add_library(novatel-parser ${SRCS})
target_include_directories(novatel-parser PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS novatel-parser DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})

target_compile_options(novatel-parser PRIVATE ${STARLING_COMPILE_OPTIONS} -Wno-strict-aliasing -Wno-long-long -Wno-effc++ -Wno-conversion)
set_target_properties(novatel-parser PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS ON
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF)

################################################################################
# Tests. 
################################################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")
add_executable(test-novatel-parser ${TEST_SRCS})
target_link_libraries(test-novatel-parser novatel-parser)
target_compile_definitions(test-novatel-parser PRIVATE
    UNIT_TEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../data")
target_compile_options(test-novatel-parser PRIVATE ${STARLING_COMPILE_OPTIONS} -Wno-strict-aliasing -Wno-long-long -Wno-effc++ -Wno-conversion -fexceptions)
set_target_properties(test-novatel-parser PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS ON
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF)

add_custom_target(run_test_novatel_parser
    COMMAND test-novatel-parser
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMENT "Running Novatel parser tests.")


