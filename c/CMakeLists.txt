cmake_minimum_required(VERSION 2.8.7)
project(gnss-converters)

# Some compiler options used globally
set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Werror -std=gnu99 -fno-unwind-tables -fno-asynchronous-unwind-tables -Wimplicit -Wshadow -Wswitch-default -Wswitch-enum -Wundef -Wuninitialized -Wpointer-arith -Wstrict-prototypes -Wcast-align -Wformat=2 -Wimplicit-function-declaration -Wredundant-decls -Wformat-security -ggdb ${CMAKE_C_FLAGS}")

IF (EXISTS ${CMAKE_SOURCE_DIR}/librtcm/c)
    add_subdirectory (librtcm/c)
ENDIF (EXISTS ${CMAKE_SOURCE_DIR}/librtcm/c)

add_subdirectory (src)
add_subdirectory (tests)
