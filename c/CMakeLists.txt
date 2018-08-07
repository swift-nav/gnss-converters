cmake_minimum_required(VERSION 2.8.7)
project(gnss-converters)

# Some compiler options used globally
set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Werror -std=gnu99 -fno-unwind-tables -fno-asynchronous-unwind-tables -Wimplicit -Wshadow -Wswitch-default -Wswitch-enum -Wundef -Wuninitialized -Wcast-align -Wformat=2 -Wimplicit-function-declaration -Wredundant-decls -Wformat-security -ggdb ${CMAKE_C_FLAGS}")

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/libsbp/c/include")
include_directories("${PROJECT_SOURCE_DIR}/librtcm/c/include")

add_subdirectory(libsbp/c)
add_subdirectory(librtcm/c)
add_subdirectory(src)
add_subdirectory(tests)
