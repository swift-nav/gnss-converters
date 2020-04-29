add_library(rtcm3tosbp_main rtcm3tosbp.c)
add_executable(rtcm3tosbp rtcm3tosbp_main.c)

target_link_libraries(rtcm3tosbp_main PUBLIC gnss_converters)
target_link_libraries(rtcm3tosbp PRIVATE rtcm3tosbp_main)

target_code_coverage(rtcm3tosbp AUTO ALL)

install(TARGETS rtcm3tosbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS rtcm3tosbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})