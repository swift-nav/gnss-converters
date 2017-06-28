/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H
#define GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H

#include <libsbp/observation.h>

struct rtcm3_sbp_state;

void rtcm3_decode_frame(const uint8_t *frame, uint32_t frame_length, struct rtcm3_sbp_state *state);

void set_gps_time(gps_time_sec_t *current_time, struct rtcm3_sbp_state* state);

void init(struct rtcm3_sbp_state *state, void (*cb)(u8 msg_id, u8 buff, u8 *len));

#endif //GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H
