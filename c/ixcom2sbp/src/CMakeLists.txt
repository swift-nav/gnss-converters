swift_add_tool_library(ixcom2sbp_main
  SOURCES
    ixcom2sbp.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
swift_add_tool(ixcom2sbp
  SOURCES
    ixcom2sbp_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

target_link_libraries(ixcom2sbp_main PRIVATE gnss_converters)
target_link_libraries(ixcom2sbp PRIVATE ixcom2sbp_main)

install(TARGETS ixcom2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ixcom2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
