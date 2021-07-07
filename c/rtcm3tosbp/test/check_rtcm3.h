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

#ifndef CHECK_RTCM3_H
#define CHECK_RTCM3_H

#include <libsbp/legacy/logging.h>
#include "../../gnss_converters/src/rtcm3_sbp_internal.h"
#include "../../gnss_converters_extra/src/sbp_rtcm3_internal.h"

/* rtcm helper defines and functions */

#define MAX_FILE_SIZE 2337772
#define RTCM3_PREAMBLE 0xD3

#define FLOAT_EPS 1e-6
#define GLO_SATELLITE_POSITION_EPS_METERS 1e-3

/* fixture globals and functions */
extern gps_time_t current_time;

void rtcm3_setup_basic(void);
void update_obs_time(const sbp_msg_obs_t *msg);
void test_RTCM3(const char *filename,
                void (*cb_rtcm_to_sbp)(uint16_t sender_id,
                                       sbp_msg_type_t msg_type,
                                       const sbp_msg_t *msg,
                                       void *context),
                gps_time_t current_time);

typedef struct Suite Suite;

Suite *rtcm3_suite(void);
Suite *rtcm3_ssr_suite(void);
Suite *rtcm3_time_suite(void);

#endif /* CHECK_RTCM3_H */
