add_library(ubx2sbp_main ubx2sbp.c)
add_executable(ubx2sbp ubx2sbp_main.c)

target_link_libraries(ubx2sbp_main PUBLIC gnss_converters)
target_link_libraries(ubx2sbp PRIVATE ubx2sbp_main)

swift_set_language_standards(ubx2sbp_main)
swift_set_compile_options(ubx2sbp_main REMOVE -Wconversion)
swift_set_language_standards(ubx2sbp)
swift_set_compile_options(ubx2sbp REMOVE -Wconversion)

install(TARGETS ubx2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ubx2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})

add_asan_variant(ubx2sbp)