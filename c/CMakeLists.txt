cmake_minimum_required(VERSION 2.8.7)
project(gnss-converters)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake" "${CMAKE_CURRENT_LIST_DIR}/cmake/common")

include(GNUInstallDirs)
include(CCache)
include(SwiftCmakeOptions)

swift_create_project_options(
    HAS_TESTS
    TEST_PACKAGES "Check"
    )

include(ClangFormat)
swift_setup_clang_format()
include(ClangTidy)
if(CMAKE_C_COMPILER_ID MATCHES "GNU")
  # clang-tidy doesn't like -Wno-psabi which it picks up from compile_commands.json
  # when using a GNU compiler. Doesn't apply when using clang compiler.
  set(clang_tidy_extra_args "EXTRA_ARGS" "-extra-arg=-Wno-unknown-warning-option")
endif()
swift_setup_clang_tidy(PATTERNS 'c/src/*.cc' ${clang_tidy_extra_args})
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

# If NDEBUG is defined, all asserts() will be knocked out, and
# currently we rely on asserts() for correct behavior.
add_compile_options(-UNDEBUG)

# Some compiler options used globally
set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Werror -std=gnu99 -fno-unwind-tables -fno-asynchronous-unwind-tables -Wimplicit -Wshadow -Wswitch-default -Wswitch-enum -Wundef -Wuninitialized -Wcast-align -Wformat=2 -Wimplicit-function-declaration -Wredundant-decls -Wformat-security -ggdb ${CMAKE_C_FLAGS}")

if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.0)
  set(CMAKE_C_FLAGS "-Wfloat-conversion ${CMAKE_C_FLAGS}")
endif()

add_subdirectory(src)
if(gnss-converters_BUILD_TESTS)
  add_subdirectory(tests)
endif()
