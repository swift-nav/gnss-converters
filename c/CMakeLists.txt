cmake_minimum_required(VERSION 3.3)
project(gnss-converters)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake" "${CMAKE_CURRENT_LIST_DIR}/cmake/common")

include(GNUInstallDirs)
include(CCache)
include(SwiftCmakeOptions)
include(LanguageStandards)
include(CompileOptions)
include(SwiftTargets)

swift_create_project_options(
    HAS_TESTS
    TEST_PACKAGES "Check"
    )

option(gnss-converters_BUILD_BINARIES "" true)

include(ClangFormat)
swift_setup_clang_format()
include(ClangTidy)

include(CheckAttributes)
create_check_attributes_target()

include(CodeCoverage)
add_code_coverage_all_targets()

option(libsbp_ENABLE_TESTS "" OFF)
option(libsbp_ENABLE_DOCS "" OFF)
find_package(Sbp REQUIRED)

option(libswiftnav_ENABLE_TESTS "" OFF)
find_package(Swiftnav REQUIRED)

option(nov2sbp_BUILD "" OFF)
option(BUILD_TESTING "" OFF)
option(I_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION "Disable all CRC validation" OFF)

if (nov2sbp_BUILD)
  target_compile_options(swiftnav PRIVATE -DGEOID_MODEL_15_MINUTE_RESOLUTION)
  set(CMAKE_C_FLAGS "-DGEOID_MODEL_15_MINUTE_RESOLUTION ${CMAKE_C_FLAGS}")
endif()

add_subdirectory(libixcom)
add_subdirectory(librtcm)
add_subdirectory(libubx)
add_subdirectory(gnss_converters)
add_subdirectory(gnss_converters_extra)
if (gnss-converters_BUILD_BINARIES)
  add_subdirectory(ixcom2sbp)
  add_subdirectory(rtcm3tosbp)
  add_subdirectory(sbp2rtcm)
  add_subdirectory(ubx2sbp)

  if (nov2sbp_BUILD)
    add_subdirectory(nov2sbp)
  endif()
endif()

if(gnss-converters_BUILD_TESTS)
  add_subdirectory(afl)
endif()

swift_validate_targets()
swift_create_clang_tidy_targets()

