add_library(ixcom2sbp_main ixcom2sbp.c)
add_executable(ixcom2sbp ixcom2sbp_main.c)

target_link_libraries(ixcom2sbp_main PRIVATE gnss_converters)
target_link_libraries(ixcom2sbp PRIVATE ixcom2sbp_main)

swift_set_language_standards(ixcom2sbp_main)
swift_set_compile_options(ixcom2sbp_main REMOVE -Wconversion)
swift_set_language_standards(ixcom2sbp)
swift_set_compile_options(ixcom2sbp REMOVE -Wconversion)

install(TARGETS ixcom2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ixcom2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
