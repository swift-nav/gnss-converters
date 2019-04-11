set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
set(include_install_dir "${CMAKE_INSTALL_INCLUDEDIR}")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

set(libgnss-converters_SRCS
  rtcm3_sbp.c 
  rtcm3_sbp_ephemeris.c 
  rtcm3_sbp_ssr.c 
  sbp_nmea.c 
  nmea.c 
  rtcm3_msm_utils.c 
  sbp_conv.c)

add_library(gnss-converters ${libgnss-converters_SRCS})
add_library(${namespace}gnss-converters ALIAS gnss-converters)

target_link_libraries(gnss-converters 
    m 
    libswiftnav::swiftnav
    libsbp::sbp 
    librtcm::rtcm
    )

target_include_directories(gnss-converters
    PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    PRIVATE "${PROJECT_SOURCE_DIR}/src"
    )

include(CMakePackageConfigHelpers)

write_basic_package_version_file("${version_config}" COMPATIBILITY SameMajorVersion)

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
    )

add_executable(rtcm3tosbp rtcm3tosbp.c)
target_link_libraries(rtcm3tosbp gnss-converters::gnss-converters)

add_executable(sbp2rtcm sbp2rtcm.c)
target_link_libraries(sbp2rtcm gnss-converters::gnss-converters)

install(
    TARGETS gnss-converters rtcm3tosbp sbp2rtcm
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${include_install_dir}"
    )

install(
    DIRECTORY "${PROJECT_SOURCE_DIR}/include/gnss-converters"
    DESTINATION "${include_install_dir}"
    )

install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
    )

install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
    )

