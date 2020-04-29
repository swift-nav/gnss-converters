add_library(sbp2rtcm_main sbp2rtcm.c)
add_executable(sbp2rtcm sbp2rtcm_main.c)

target_link_libraries(sbp2rtcm_main PUBLIC gnss_converters)
target_link_libraries(sbp2rtcm PRIVATE sbp2rtcm_main)

target_code_coverage(sbp2rtcm AUTO ALL)

install(TARGETS sbp2rtcm_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS sbp2rtcm DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})