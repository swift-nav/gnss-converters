/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RTCM3_SSR_DECODE_H
#define SWIFTNAV_RTCM3_SSR_DECODE_H

#include <rtcm3/messages.h>
#include <swiftnav/bitstream.h>

rtcm3_rc rtcm3_decode_orbit_bitstream(swiftnav_in_bitstream_t *buff,
                                      rtcm_msg_orbit *msg_orbit);
rtcm3_rc rtcm3_decode_clock_bitstream(swiftnav_in_bitstream_t *buff,
                                      rtcm_msg_clock *msg_clock);
rtcm3_rc rtcm3_decode_orbit_clock_bitstream(
    swiftnav_in_bitstream_t *buff, rtcm_msg_orbit_clock *msg_orbit_clock);
rtcm3_rc rtcm3_decode_code_bias_bitstream(swiftnav_in_bitstream_t *buff,
                                          rtcm_msg_code_bias *msg_code_bias);
rtcm3_rc rtcm3_decode_phase_bias_bitstream(swiftnav_in_bitstream_t *buff,
                                           rtcm_msg_phase_bias *msg_phase_bias);

static inline rtcm3_rc rtcm3_decode_orbit(const uint8_t buff[],
                                          rtcm_msg_orbit *msg_orbit) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_orbit_bitstream(&bitstream, msg_orbit);
}

static inline rtcm3_rc rtcm3_decode_clock(const uint8_t buff[],
                                          rtcm_msg_clock *msg_clock) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_clock_bitstream(&bitstream, msg_clock);
}

static inline rtcm3_rc rtcm3_decode_orbit_clock(
    const uint8_t buff[], rtcm_msg_orbit_clock *msg_orbit_clock) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_orbit_clock_bitstream(&bitstream, msg_orbit_clock);
}

static inline rtcm3_rc rtcm3_decode_code_bias(
    const uint8_t buff[], rtcm_msg_code_bias *msg_code_bias) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_code_bias_bitstream(&bitstream, msg_code_bias);
}

static inline rtcm3_rc rtcm3_decode_phase_bias(
    const uint8_t buff[], rtcm_msg_phase_bias *msg_phase_bias) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_phase_bias_bitstream(&bitstream, msg_phase_bias);
}

#endif /* SWIFTNAV_RTCM3_SSR_DECODE_H */
