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

/* These encoder functions are provided to allow for easier unit testing however
 * they are not robust to be used in production. Before using with real data, we
 * would need to handle ambiguity rollover and code carrier divergance at least
 */

#ifndef SWIFTNAV_UBX_ENCODE_H
#define SWIFTNAV_UBX_ENCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ubx/constants.h>
#include <ubx/ubx_messages.h>

uint16_t ubx_encode_hnr_pvt(const ubx_hnr_pvt *msg_hnr_pvt, uint8_t buff[]);
uint16_t ubx_encode_rawx(const ubx_rxm_rawx *msg_rawx, uint8_t buff[]);
uint16_t ubx_encode_nav_att(const ubx_nav_att *msg_nav_att, uint8_t buff[]);
uint16_t ubx_encode_nav_clock(const ubx_nav_clock *msg_nav_clock,
                              uint8_t buff[]);
uint16_t ubx_encode_nav_pvt(const ubx_nav_pvt *msg_nav_pvt, uint8_t buff[]);
uint16_t ubx_encode_nav_velecef(const ubx_nav_velecef *msg_nav_velecef,
                                uint8_t buff[]);
uint16_t ubx_encode_nav_sat(const ubx_nav_sat *msg_nav_sat, uint8_t buff[]);
uint16_t ubx_encode_nav_status(const ubx_nav_status *msg_nav_status,
                               uint8_t buff[]);
uint16_t ubx_encode_mga_gps_eph(const ubx_mga_gps_eph *msg_mga_gps_eph,
                                uint8_t buff[]);
uint16_t ubx_encode_rxm_sfrbx(const ubx_rxm_sfrbx *msg_rxm_sfrbx,
                              uint8_t buff[]);
uint16_t ubx_encode_esf_ins(const ubx_esf_ins *msg_esf_ins, uint8_t buff[]);
uint16_t ubx_encode_esf_meas(const ubx_esf_meas *msg_esf_meas, uint8_t buff[]);
uint16_t ubx_encode_esf_raw(const ubx_esf_raw *msg_esf_raw, uint8_t buff[]);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_UBX_ENCODE_H */
