# todo: use bits.[ch] from libswiftnav, MAP-605
set(librtcm_HEADERS
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/bits.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/constants.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/messages.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/encode.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/decode.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/sta_decode.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/eph_decode.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/eph_encode.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/ssr_decode.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/msm_utils.h
  ${PROJECT_SOURCE_DIR}/librtcm/include/rtcm3/logging.h
  )

swift_add_library(rtcm
  SOURCES
    decode.c
    encode.c
    msm_utils.c
    eph_decode.c
    eph_encode.c
    ssr_decode.c
    sta_decode.c
    bits.c
    logging.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion -Wstack-protector
  )
add_library(swiftnav::rtcm ALIAS rtcm)
target_include_directories(rtcm
  PUBLIC ${PROJECT_SOURCE_DIR}/librtcm/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
  )
target_link_libraries(rtcm PUBLIC swiftnav)

install(TARGETS rtcm DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${librtcm_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/rtcm3)
