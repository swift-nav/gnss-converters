cmake_minimum_required(VERSION 2.8.7)
project(gnss-converters)
include(GNUInstallDirs)

# If NDEBUG is defined, all asserts() will be knocked out, and
# currently we rely on asserts() for correct behavior.
add_compile_options(-UNDEBUG)

# Some compiler options used globally
set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Werror -std=gnu99 -fno-unwind-tables -fno-asynchronous-unwind-tables -Wimplicit -Wshadow -Wswitch-default -Wswitch-enum -Wundef -Wuninitialized -Wcast-align -Wformat=2 -Wimplicit-function-declaration -Wredundant-decls -Wformat-security -ggdb ${CMAKE_C_FLAGS}")

if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.0)
  set(CMAKE_C_FLAGS "-Wfloat-conversion ${CMAKE_C_FLAGS}")
endif()

if(NOT TARGET rtcm AND EXISTS ${PROJECT_SOURCE_DIR}/librtcm/c)
  add_subdirectory(librtcm/c)
endif()

if(NOT TARGET sbp AND EXISTS ${PROJECT_SOURCE_DIR}/libsbp/c)
  add_subdirectory(libsbp/c)
endif()

if(NOT TARGET swiftnav AND EXISTS ${PROJECT_SOURCE_DIR}/libswiftnav)
  add_subdirectory(libswiftnav)
endif()

add_subdirectory(src)
add_subdirectory(tests)
