swift_add_tool_library(rtcm3tosbp_main
  SOURCES
    rtcm3tosbp.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
swift_add_tool(rtcm3tosbp
  SOURCES
    rtcm3tosbp_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

target_link_libraries(rtcm3tosbp_main PUBLIC gnss_converters)
target_link_libraries(rtcm3tosbp PRIVATE rtcm3tosbp_main)

install(TARGETS rtcm3tosbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS rtcm3tosbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
