cmake_minimum_required(VERSION 2.8.7)
project(gnss-converters)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake" "${CMAKE_CURRENT_LIST_DIR}/cmake/common")

include(GNUInstallDirs)
include(CCache)
include(SwiftCmakeOptions)
include(ASAN-targets)
include(LanguageStandards)
include(CompileOptions)

swift_create_project_options(
    HAS_TESTS
    TEST_PACKAGES "Check"
    )

option(gnss-converters_BUILD_BINARIES "" true)

include(ClangFormat)
swift_setup_clang_format()
include(ClangTidy)

if(CMAKE_C_COMPILER_ID MATCHES "GNU")
  # clang-tidy doesn't like -Wno-psabi which it picks up from compile_commands.json
  # when using a GNU compiler. Doesn't apply when using clang compiler.
  set(clang_tidy_extra_args "EXTRA_ARGS" "-extra-arg=-Wno-unknown-warning-option")
endif()

swift_setup_clang_tidy(PATTERNS 'src/*.c' 'src/*.cc' ${clang_tidy_extra_args})

include(CodeCoverage)
add_code_coverage_all_targets()

include(TestTargets)

option(libsbp_ENABLE_TESTS "" OFF)
option(libsbp_ENABLE_DOCS "" OFF)
find_package(Sbp REQUIRED)

option(libswiftnav_ENABLE_TESTS "" OFF)
find_package(Swiftnav REQUIRED)

option(librtcm_ENABLE_TESTS "" OFF)
find_package(Rtcm REQUIRED)

option(libubx_ENABLE_TESTS "" OFF)
find_package(Ubx REQUIRED)

option(libixcom_ENABLE_TESTS "" OFF)
find_package(Ixcom REQUIRED)

option(nov2sbp_BUILD "" OFF)
option(BUILD_TESTING "" OFF)
option(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION "Disable all CRC validation" OFF)


if (nov2sbp_BUILD)
  target_compile_options(swiftnav PRIVATE -DGEOID_MODEL_15_MINUTE_RESOLUTION)
  set(CMAKE_C_FLAGS "-DGEOID_MODEL_15_MINUTE_RESOLUTION ${CMAKE_C_FLAGS}")
endif()

add_subdirectory(src)
if(gnss-converters_BUILD_TESTS)
  add_subdirectory(tests)
endif()
