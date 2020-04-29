add_library(ubx2sbp_main ubx2sbp.c)
add_executable(ubx2sbp ubx2sbp_main.c)

target_link_libraries(ubx2sbp_main PUBLIC gnss_converters)
target_link_libraries(ubx2sbp PRIVATE ubx2sbp_main)

target_code_coverage(ubx2sbp AUTO ALL)

install(TARGETS ubx2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ubx2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})