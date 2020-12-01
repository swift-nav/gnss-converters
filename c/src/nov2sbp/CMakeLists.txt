add_subdirectory(parser)

add_library(nov2sbp_main nov2sbp.cc send_sbp_obs_messages.cc swiftnav_conversion_helpers.cc)

target_link_libraries(nov2sbp_main novatel-parser swiftnav sbp)

# Build SBP conversion tool.
add_executable(nov2sbp nov2sbp_main.cc)
target_link_libraries(nov2sbp nov2sbp_main)

swift_set_language_standards(nov2sbp_main)
swift_set_compile_options(nov2sbp_main REMOVE -Wconversion ADD -Wno-extern-c-compat)
target_compile_options(nov2sbp_main PRIVATE -UNDEBUG)
swift_set_language_standards(nov2sbp)
swift_set_compile_options(nov2sbp REMOVE -Wconversion ADD -Wno-extern-c-compat)
target_compile_options(nov2sbp PRIVATE -UNDEBUG)

install(TARGETS nov2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS nov2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
