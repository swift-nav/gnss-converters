/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RTCM3_DECODE_H
#define SWIFTNAV_RTCM3_DECODE_H

#include <stdint.h>
#include <swiftnav/bitstream.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <rtcm3/messages.h>

rtcm3_rc rtcm3_decode_1001_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1001);
rtcm3_rc rtcm3_decode_1002_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1002);
rtcm3_rc rtcm3_decode_1003_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1003);
rtcm3_rc rtcm3_decode_1004_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1004);
rtcm3_rc rtcm3_decode_1005_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1005 *msg_1005);
rtcm3_rc rtcm3_decode_1006_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1006 *msg_1006);
rtcm3_rc rtcm3_decode_1007_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1007 *msg_1007);
rtcm3_rc rtcm3_decode_1008_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1008 *msg_1008);
rtcm3_rc rtcm3_decode_1010_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1010);
rtcm3_rc rtcm3_decode_1012_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1012);
rtcm3_rc rtcm3_decode_1029_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1029 *msg_1029);
rtcm3_rc rtcm3_decode_1033_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1033 *msg_1033);
rtcm3_rc rtcm3_decode_1230_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1230 *msg_1230);
rtcm3_rc rtcm3_decode_msm4_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_msm5_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_msm6_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_msm7_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_4062_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_swift_proprietary *msg);
rtcm3_rc rtcm3_decode_4075(const uint8_t buff[], rtcm_msg_ndf *msg);

double rtcm3_decode_lock_time(uint8_t lock);

/* Backwards compatibility versions, these are all susceptible to buffer
 * overflows */

static inline rtcm3_rc rtcm3_decode_1001(const uint8_t buff[],
                                         rtcm_obs_message *msg_1001) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1001_bitstream(&bitstream, msg_1001);
}

static inline rtcm3_rc rtcm3_decode_1002(const uint8_t buff[],
                                         rtcm_obs_message *msg_1002) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1002_bitstream(&bitstream, msg_1002);
}

static inline rtcm3_rc rtcm3_decode_1003(const uint8_t buff[],
                                         rtcm_obs_message *msg_1003) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1003_bitstream(&bitstream, msg_1003);
}

static inline rtcm3_rc rtcm3_decode_1004(const uint8_t buff[],
                                         rtcm_obs_message *msg_1004) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1004_bitstream(&bitstream, msg_1004);
}

static inline rtcm3_rc rtcm3_decode_1005(const uint8_t buff[],
                                         rtcm_msg_1005 *msg_1005) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1005_bitstream(&bitstream, msg_1005);
}

static inline rtcm3_rc rtcm3_decode_1006(const uint8_t buff[],
                                         rtcm_msg_1006 *msg_1006) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1006_bitstream(&bitstream, msg_1006);
}

static inline rtcm3_rc rtcm3_decode_1007(const uint8_t buff[],
                                         rtcm_msg_1007 *msg_1007) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1007_bitstream(&bitstream, msg_1007);
}

static inline rtcm3_rc rtcm3_decode_1008(const uint8_t buff[],
                                         rtcm_msg_1008 *msg_1008) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1008_bitstream(&bitstream, msg_1008);
}

static inline rtcm3_rc rtcm3_decode_1010(const uint8_t buff[],
                                         rtcm_obs_message *msg_1010) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1010_bitstream(&bitstream, msg_1010);
}

static inline rtcm3_rc rtcm3_decode_1012(const uint8_t buff[],
                                         rtcm_obs_message *msg_1012) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1012_bitstream(&bitstream, msg_1012);
}

static inline rtcm3_rc rtcm3_decode_1029(const uint8_t buff[],
                                         rtcm_msg_1029 *msg_1029) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1029_bitstream(&bitstream, msg_1029);
}

static inline rtcm3_rc rtcm3_decode_1033(const uint8_t buff[],
                                         rtcm_msg_1033 *msg_1033) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1033_bitstream(&bitstream, msg_1033);
}

static inline rtcm3_rc rtcm3_decode_1230(const uint8_t buff[],
                                         rtcm_msg_1230 *msg_1230) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1230_bitstream(&bitstream, msg_1230);
}

static inline rtcm3_rc rtcm3_decode_msm4(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm4_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_msm5(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm5_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_msm6(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm6_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_msm7(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm7_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_4062(const uint8_t buff[],
                                         rtcm_msg_swift_proprietary *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_4062_bitstream(&bitstream, msg);
}

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_DECODE_H */
