set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Threads)

set(TEST_SOURCE_FILES
    check_main.c
    check_ixcom.c
    )

swift_add_test(test_libixcom
        UNIT_TEST
        SRCS ${TEST_SOURCE_FILES}
        LINK ixcom check Threads::Threads
        )
swift_set_language_standards(test_libixcom)
swift_set_compile_options(test_libixcom)

swift_add_test(test_libixcom_pedantic
  UNIT_TEST
  SRCS check_pedantic.cc
  LINK ixcom
)
swift_set_language_standards(test_libixcom_pedantic)
swift_set_compile_options(test_libixcom_pedantic ADD -pedantic)
