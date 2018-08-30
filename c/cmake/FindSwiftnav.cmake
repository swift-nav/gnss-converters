cmake_minimum_required(VERSION 2.8.7)

# This brings in the external project support in swiftnav
include(ExternalProject)

# This adds libswiftnav as an external project with the specified parameters.
ExternalProject_Add(libswiftnav
        # We use SOURCE_DIR because we use version control to track the
        # version of this library instead of using the build tool
        SOURCE_DIR ${PROJECT_SOURCE_DIR}/libswiftnav
        # We don't want to install this globally; we just want to use it in
        # place.
        INSTALL_COMMAND cmake -E echo "Not installing libswiftnav globally."
        # This determines the subdirectory of the build directory in which
        # libswiftnav gets built.
        PREFIX swiftnav
        # This simply passes down cmake arguments, which allows us to define
        # libswiftnav-specific cmake flags as arguments to the toplevel cmake
        # invocation.
        CMAKE_ARGS ${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug)

# This pulls out the variables `source_dir` and `binary_dir` from the
# libswiftnav project, so we can refer to them below.
ExternalProject_Get_Property(libswiftnav source_dir binary_dir)

# This tells later `target_link_libraries` commands about the swiftnav
# library.
add_library(swiftnav SHARED IMPORTED GLOBAL)

# This tells where the static libswiftnav binary will end up.  I have no
# idea how to control this and just found it with `locate`.
set_property(TARGET swiftnav
        PROPERTY IMPORTED_LOCATION "${binary_dir}/src/libswiftnav-static.a")

# This makes the swiftnav library depend on the libswiftnav external
# project, so that when you ask to link against swiftnav, the external
# project will get built.
add_dependencies(swiftnav libswiftnav)

# This tells where the libswiftnav headers generated during the build
# process will end up.  I have no idea how to control this and just
# found it with `locate`.  Note that any targets specified after this
# file fragment is included will now include libswiftnav headers as part of
# their compile commands.
include_directories(SYSTEM "${source_dir}/include")
