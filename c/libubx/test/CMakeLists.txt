set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Threads)

set(TEST_SOURCE_FILES
    check_main.c
    check_ubx.c
    )

swift_add_test(test_libubx
        UNIT_TEST
        SRCS ${TEST_SOURCE_FILES}
        LINK ubx check Threads::Threads
        )

swift_set_language_standards(test_libubx)
swift_set_compile_options(test_libubx REMOVE -Wconversion)

swift_add_test(test_libubx_pedantic
  UNIT_TEST
  SRCS
    check_pedantic.cc
  LINK
    ubx
)
swift_set_compile_options(test_libubx_pedantic ADD -pedantic)
