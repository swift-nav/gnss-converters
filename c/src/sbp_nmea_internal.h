/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters/nmea.h>
#include <stdbool.h>
#include <stdint.h>

#define TIME_SOURCE_MASK 0x07       /* Bits 0-2 */
#define POSITION_MODE_MASK 0x07     /* Bits 0-2 */
#define OBSERVATION_VALID 0x1000000 /* Bits 0-7 */
#define NO_TIME 0
#define TOW_INVALID 0xFFFFFFFF

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)

void get_utc_time_string(bool time,
                         bool date,
                         bool trunc_date,
                         const msg_utc_time_t *sbp_utc_time,
                         char *utc_str,
                         u8 size);
