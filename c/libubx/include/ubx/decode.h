/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_UBX_DECODE_H
#define SWIFTNAV_UBX_DECODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <swiftnav/bytestream.h>
#include <ubx/constants.h>
#include <ubx/ubx_messages.h>

void ubx_checksum(const uint8_t buff[], size_t length, uint8_t *checksum);
ubx_rc ubx_decode_hnr_pvt_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_hnr_pvt *msg_hnr_pvt);
ubx_rc ubx_decode_rxm_rawx_bytestream(swiftnav_bytestream_t *buff,
                                      ubx_rxm_rawx *msg_rawx);
ubx_rc ubx_decode_nav_att_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_nav_att *msg_nav_att);
ubx_rc ubx_decode_nav_clock_bytestream(swiftnav_bytestream_t *buff,
                                       ubx_nav_clock *msg_nav_clock);
ubx_rc ubx_decode_nav_pvt_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_nav_pvt *msg_nav_pvt);
ubx_rc ubx_decode_nav_velecef_bytestream(swiftnav_bytestream_t *buff,
                                         ubx_nav_velecef *msg_nav_velecef);
ubx_rc ubx_decode_nav_sat_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_nav_sat *msg_nav_sat);
ubx_rc ubx_decode_nav_status_bytestream(swiftnav_bytestream_t *buff,
                                        ubx_nav_status *msg_nav_status);
ubx_rc ubx_decode_mga_gps_eph_bytestream(swiftnav_bytestream_t *buff,
                                         ubx_mga_gps_eph *msg_mga_gps_eph);
ubx_rc ubx_decode_rxm_sfrbx_bytestream(swiftnav_bytestream_t *buff,
                                       ubx_rxm_sfrbx *msg_rxm_sfrbx);
ubx_rc ubx_decode_esf_ins_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_esf_ins *msg_esf_ins);
ubx_rc ubx_decode_esf_meas_bytestream(swiftnav_bytestream_t *buff,
                                      ubx_esf_meas *msg_esf_meas);
ubx_rc ubx_decode_esf_raw_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_esf_raw *msg_esf_raw);

static inline ubx_rc ubx_decode_hnr_pvt(const uint8_t buff[],
                                        ubx_hnr_pvt *msg_hnr_pvt) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_hnr_pvt_bytestream(&bytestream, msg_hnr_pvt);
}

static inline ubx_rc ubx_decode_rxm_rawx(const uint8_t buff[],
                                         ubx_rxm_rawx *msg_rawx) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_rxm_rawx_bytestream(&bytestream, msg_rawx);
}

static inline ubx_rc ubx_decode_nav_att(const uint8_t buff[],
                                        ubx_nav_att *msg_nav_att) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_nav_att_bytestream(&bytestream, msg_nav_att);
}

static inline ubx_rc ubx_decode_nav_clock(const uint8_t buff[],
                                          ubx_nav_clock *msg_nav_clock) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_nav_clock_bytestream(&bytestream, msg_nav_clock);
}

static inline ubx_rc ubx_decode_nav_pvt(const uint8_t buff[],
                                        ubx_nav_pvt *msg_nav_pvt) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_nav_pvt_bytestream(&bytestream, msg_nav_pvt);
}

static inline ubx_rc ubx_decode_nav_velecef(const uint8_t buff[],
                                            ubx_nav_velecef *msg_nav_velecef) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_nav_velecef_bytestream(&bytestream, msg_nav_velecef);
}

static inline ubx_rc ubx_decode_nav_sat(const uint8_t buff[],
                                        ubx_nav_sat *msg_nav_sat) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_nav_sat_bytestream(&bytestream, msg_nav_sat);
}

static inline ubx_rc ubx_decode_nav_status(const uint8_t buff[],
                                           ubx_nav_status *msg_nav_status) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_nav_status_bytestream(&bytestream, msg_nav_status);
}

static inline ubx_rc ubx_decode_mga_gps_eph(const uint8_t buff[],
                                            ubx_mga_gps_eph *msg_mga_gps_eph) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_mga_gps_eph_bytestream(&bytestream, msg_mga_gps_eph);
}

static inline ubx_rc ubx_decode_rxm_sfrbx(const uint8_t buff[],
                                          ubx_rxm_sfrbx *msg_rxm_sfrbx) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_rxm_sfrbx_bytestream(&bytestream, msg_rxm_sfrbx);
}

static inline ubx_rc ubx_decode_esf_ins(const uint8_t buff[],
                                        ubx_esf_ins *msg_esf_ins) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_esf_ins_bytestream(&bytestream, msg_esf_ins);
}

static inline ubx_rc ubx_decode_esf_meas(const uint8_t buff[],
                                         ubx_esf_meas *msg_esf_meas) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_esf_meas_bytestream(&bytestream, msg_esf_meas);
}

static inline ubx_rc ubx_decode_esf_raw(const uint8_t buff[],
                                        ubx_esf_raw *msg_esf_raw) {
  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, buff, UINT32_MAX);
  return ubx_decode_esf_raw_bytestream(&bytestream, msg_esf_raw);
}

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_UBX_DECODE_H */
