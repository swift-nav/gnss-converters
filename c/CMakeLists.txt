cmake_minimum_required(VERSION 3.7)

include("cmake/HunterGate.cmake")
HunterGate(
    URL https://github.com/woodfell/hunter/archive/v1.0.3.tar.gz
    SHA1 cf2c679d59dbf934fbc6fa7c6ba204a6e72e850f
  )

include("cmake/VersionFromGit.cmake")
version_from_git()
message(STATUS "Building gnss-converters version ${VERSION}")

project(gnss-converters LANGUAGES C CXX VERSION ${VERSION})

hunter_add_package(libsbp)
find_package(libsbp)
hunter_add_package(librtcm)
find_package(librtcm)
hunter_add_package(libswiftnav)
find_package(libswiftnav)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)

# Some compiler options used globally
set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Werror -std=gnu99 -fno-unwind-tables -fno-asynchronous-unwind-tables -Wimplicit -Wshadow -Wswitch-default -Wswitch-enum -Wundef -Wuninitialized -Wcast-align -Wformat=2 -Wimplicit-function-declaration -Wredundant-decls -Wformat-security -Wfloat-conversion -ggdb ${CMAKE_C_FLAGS}")

add_subdirectory(src)
add_subdirectory(tests)
