/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_COMMON_H
#define GNSS_CONVERTERS_COMMON_H

#include "libsbp/gnss.h"
#include "libsbp/observation.h"
#include "swiftnav/common.h"
#include "swiftnav/ephemeris.h"
#include "swiftnav/signal.h"

#define MSG_OBS_P_MULTIPLIER ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER ((float)4)
#define MSG_OBS_LF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_DF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID ((u8)(1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID ((u8)(1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN ((u8)(1 << 2))
#define MSG_OBS_FLAGS_DOPPLER_VALID ((u8)(1 << 3))
#define MSG_OBS_FLAGS_RAIM_EXCLUSION ((u8)(1 << 7))

typedef union {
  msg_ephemeris_gps_t gps;
  msg_ephemeris_bds_t bds;
  msg_ephemeris_gal_t gal;
  msg_ephemeris_sbas_t sbas;
  msg_ephemeris_glo_t glo;
  msg_ephemeris_qzss_t qzss;
} msg_ephemeris_t;

void pack_ephemeris_common_content(const ephemeris_t *e,
                                   ephemeris_common_content_t *common);

gps_time_t sbp_gps_time_2_gps_time(const gps_time_sec_t *s);

#endif /* #ifndef GNSS_CONVERTERS_COMMON_H */
