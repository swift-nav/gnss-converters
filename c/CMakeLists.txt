cmake_minimum_required(VERSION 2.8.7)
project(gnss-converters)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Some compiler options used globally
set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Werror -std=gnu99 -fno-unwind-tables -fno-asynchronous-unwind-tables -Wimplicit -Wshadow -Wswitch-default -Wswitch-enum -Wundef -Wuninitialized -Wpointer-arith -Wstrict-prototypes -Wcast-align -Wformat=2 -Wimplicit-function-declaration -Wredundant-decls -Wformat-security -ggdb ${CMAKE_C_FLAGS}")

# This library is dependent on libsbp
if(EXISTS ${CMAKE_SOURCE_DIR}/libsbp/c)
    find_package(Sbp)
endif()

if(EXISTS ${CMAKE_SOURCE_DIR}/librtcm/c)
    add_subdirectory (librtcm/c)
endif()

add_subdirectory(src)
add_subdirectory(tests)
