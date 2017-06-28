cmake_minimum_required(VERSION 2.8.7)
project(gnss-convertors)

# This brings in the external project support in cmake
include(ExternalProject)

# This adds libsbp as an external project with the specified parameters.
ExternalProject_Add(libsbp
        # We use SOURCE_DIR because we use version control to track the
        # version of this library instead of using the build tool
        SOURCE_DIR ${CMAKE_SOURCE_DIR}/libsbp/c
        # We don't want to install this globally; we just want to use it in
        # place.
        INSTALL_COMMAND cmake -E echo "Not installing libsbp globally."
        # This determines the subdirectory of the build directory in which
        # libsbp gets built.
        PREFIX sbp
        # This simply passes down cmake arguments, which allows us to define
        # libsbp-specific cmake flags as arguments to the toplevel cmake
        # invocation.
        CMAKE_ARGS ${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Release)

# This pulls out the variables `source_dir` and `binary_dir` from the
# libsbp project, so we can refer to them below.
ExternalProject_Get_Property(libsbp source_dir binary_dir)

# This tells later `target_link_libraries` commands about the sbp
# library.
add_library(sbp SHARED IMPORTED GLOBAL)

# This tells where the static libsbp binary will end up.  I have no
# idea how to control this and just found it with `locate`.
set_property(TARGET sbp
        PROPERTY IMPORTED_LOCATION "${binary_dir}/src/libsbp-static.a")

# This makes the sbp library depend on the libsbp external
# project, so that when you ask to link against sbp, the external
# project will get built.
add_dependencies(sbp libsbp)

# This tells where the libsbp headers generated during the build
# process will end up.  I have no idea how to control this and just
# found it with `locate`.  Note that any targets specified after this
# file fragment is included will now include libsbp headers as part of
# their compile commands.
include_directories(SYSTEM "${source_dir}/include")

# Some compiler options used globally
set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Werror -std=gnu99 -fno-unwind-tables -fno-asynchronous-unwind-tables -Wimplicit -Wshadow -Wswitch-default -Wswitch-enum -Wundef -Wuninitialized -Wpointer-arith -Wstrict-prototypes -Wcast-align -Wformat=2 -Wimplicit-function-declaration -Wredundant-decls -Wformat-security ${CMAKE_C_FLAGS}")

add_subdirectory (librtcm/c)
add_subdirectory (src)
add_subdirectory (tests)
