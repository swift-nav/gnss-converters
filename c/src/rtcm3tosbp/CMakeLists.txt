add_library(rtcm3tosbp_main rtcm3tosbp.c)
add_executable(rtcm3tosbp rtcm3tosbp_main.c)

target_link_libraries(rtcm3tosbp_main PUBLIC gnss_converters)
target_link_libraries(rtcm3tosbp PRIVATE rtcm3tosbp_main)

swift_set_language_standards(rtcm3tosbp_main)
swift_set_compile_options(rtcm3tosbp_main REMOVE -Wconversion)
swift_set_language_standards(rtcm3tosbp)
swift_set_compile_options(rtcm3tosbp REMOVE -Wconversion)

install(TARGETS rtcm3tosbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS rtcm3tosbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
