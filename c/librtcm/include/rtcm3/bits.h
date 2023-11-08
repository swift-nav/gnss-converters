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

#ifndef SWIFTNAV_RTCM3_BITS_H
#define SWIFTNAV_RTCM3_BITS_H

#include <swiftnav/bitstream.h>

#include "rtcm3/messages.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t rtcm_getbitu(const uint8_t *buff, uint32_t pos, uint8_t len);
uint64_t rtcm_getbitul(const uint8_t *buff, uint32_t pos, uint8_t len);
int32_t rtcm_getbits(const uint8_t *buff, uint32_t pos, uint8_t len);
int64_t rtcm_getbitsl(const uint8_t *buff, uint32_t pos, uint8_t len);
void rtcm_setbitu(uint8_t *buff, uint32_t pos, uint32_t len, uint32_t data);
void rtcm_setbitul(uint8_t *buff, uint32_t pos, uint32_t len, uint64_t data);
void rtcm_setbits(uint8_t *buff, uint32_t pos, uint32_t len, int32_t data);
void rtcm_setbitsl(uint8_t *buff, uint32_t pos, uint32_t len, int64_t data);
int32_t rtcm_get_sign_magnitude_bit(const uint8_t *buff,
                                    uint32_t pos,
                                    uint8_t len);
rtcm3_rc rtcm_get_sign_magnitude_bitstream(swiftnav_in_bitstream_t *buff,
                                           uint8_t len,
                                           int32_t *out);
void rtcm_set_sign_magnitude_bit(uint8_t *buff,
                                 uint32_t pos,
                                 uint8_t len,
                                 int64_t data);
#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_BITS_H */
