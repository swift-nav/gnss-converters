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

#ifndef SWIFTNAV_RTCM3_EPH_ENCODE_H
#define SWIFTNAV_RTCM3_EPH_ENCODE_H

#include <rtcm3/messages.h>
#define BEIDOU_GEOS_MAX_PRN 5

#ifdef __cplusplus
extern "C" {
#endif

uint16_t rtcm3_encode_gps_eph(const rtcm_msg_eph *msg_1019, uint8_t buff[]);
uint16_t rtcm3_encode_glo_eph(const rtcm_msg_eph *msg_1020, uint8_t buff[]);
uint16_t rtcm3_encode_bds_eph(const rtcm_msg_eph *msg_1042, uint8_t buff[]);
uint16_t rtcm3_encode_gal_eph_inav(const rtcm_msg_eph *msg_eph, uint8_t buff[]);
uint16_t rtcm3_encode_gal_eph_fnav(const rtcm_msg_eph *msg_eph, uint8_t buff[]);
// uint16_t rtcm3_encode_qzss_eph(const uint8_t buff[], rtcm_msg_eph *msg_eph);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_EPH_ENCODE_H */
