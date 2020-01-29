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

typedef union {
  msg_ephemeris_gps_t gps;
  msg_ephemeris_bds_t bds;
  msg_ephemeris_gal_t gal;
  msg_ephemeris_sbas_t sbas;
  msg_ephemeris_glo_t glo;
  msg_ephemeris_qzss_t qzss;
} msg_ephemeris_t;

sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from);
void pack_ephemeris_common(const ephemeris_t *e,
                           ephemeris_common_content_t *common);

#endif /* #ifndef GNSS_CONVERTERS_COMMON_H */
