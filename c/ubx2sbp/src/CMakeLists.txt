swift_add_tool_library(ubx2sbp_main
  SOURCES
    ubx2sbp.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
swift_add_tool(ubx2sbp
  SOURCES
    ubx2sbp_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

target_link_libraries(ubx2sbp_main PUBLIC gnss_converters)
target_link_libraries(ubx2sbp PRIVATE ubx2sbp_main)

install(TARGETS ubx2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ubx2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
