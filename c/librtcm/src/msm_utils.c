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

#include <assert.h>
#include <rtcm3/msm_utils.h>
#include <stdio.h>

#define LIBRTCM_LOG_INTERNAL
#include <rtcm3/logging.h>

/** Convert message number into MSM message type
 *
 * \param msg_num RTCM message number
 * \return MSM type enum
 */
msm_enum to_msm_type(uint16_t msg_num) {
  if (msg_num < 1071 || msg_num > 1127) {
    return MSM_UNKNOWN;
  }
  switch (msg_num % 10) {
    case 1:
      return MSM1;
    case 2:
      return MSM2;
    case 3:
      return MSM3;
    case 4:
      return MSM4;
    case 5:
      return MSM5;
    case 6:
      return MSM6;
    case 7:
      return MSM7;
    default:
      return MSM_UNKNOWN;
  }
}

/** Convert message number into constellation enum
 * This also supports SSR messages
 * \param msg_num RTCM message number
 * \return constellation enum
 */
rtcm_constellation_t to_constellation(uint16_t msg_num) {
  if ((msg_num >= 1071 && msg_num <= 1077) ||
      (msg_num >= 1057 && msg_num <= 1062) || msg_num == 1265) {
    return RTCM_CONSTELLATION_GPS;
  }
  if ((msg_num >= 1081 && msg_num <= 1087) ||
      (msg_num >= 1063 && msg_num <= 1068) || msg_num == 1266) {
    return RTCM_CONSTELLATION_GLO;
  }
  if ((msg_num >= 1091 && msg_num <= 1097) ||
      (msg_num >= 1240 && msg_num <= 1245) || (msg_num == 1267)) {
    return RTCM_CONSTELLATION_GAL;
  }
  if ((msg_num >= 1101 && msg_num <= 1107) || msg_num == 1269) {
    return RTCM_CONSTELLATION_SBAS;
  }
  if ((msg_num >= 1111 && msg_num <= 1117) ||
      (msg_num >= 1246 && msg_num <= 1251) || msg_num == 1268) {
    return RTCM_CONSTELLATION_QZS;
  }
  if ((msg_num >= 1121 && msg_num <= 1127) ||
      (msg_num >= 1258 && msg_num <= 1263) || msg_num == 1270) {
    return RTCM_CONSTELLATION_BDS;
  }
  return RTCM_CONSTELLATION_INVALID;
}

/** Count the true values in a Boolean array
 *
 * \param mask_size
 * \param bool_mask Boolean array
 * \return number of true values in the array
 */
uint8_t count_mask_values(uint8_t mask_size, const bool mask[]) {
  uint8_t ret = 0;
  for (uint8_t i = 0; i < mask_size; i++) {
    ret += mask[i] ? 1 : 0;
  }
  return ret;
}

/** Return the position of the nth true value in a Boolean array
 *
 * \param mask_size
 * \param bool_mask Boolean array
 * \param n A number between 1 and count_mask_values (causes an assert if not)
 * \return The 0-based position of the nth true value in the array
 */
uint8_t find_nth_mask_value(const uint8_t mask_size,
                            const bool mask[],
                            const uint8_t n) {
  assert(n > 0);
  uint8_t trues_found = 0;
  for (uint8_t pos = 0; pos < mask_size; pos++) {
    trues_found += mask[pos] ? 1 : 0;
    if (n == trues_found) {
      /* this is the nth set value in the array, return its position */
      return pos;
    }
  }
  assert(!"n is larger than count_mask_values");
  /* this will never be reached */
  return 0;
}
