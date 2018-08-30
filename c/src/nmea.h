/*
 * Copyright (C) 2013-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NMEA_H
#define SWIFTNAV_NMEA_H

#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <sbp_nmea.h>
#include <swiftnav/common.h>
#include <swiftnav/gnss_time.h>

/* The "QI" stands for "Quality Indicator" which is the terminology used
   for this field in the NMEA specification. */
#define NMEA_GGA_QI_INVALID 0
#define NMEA_GGA_QI_GPS 1
#define NMEA_GGA_QI_DGPS 2
#define NMEA_GGA_QI_PPS 3
#define NMEA_GGA_QI_RTK 4
#define NMEA_GGA_QI_FLOAT 5
#define NMEA_GGA_QI_EST 6
#define NMEA_GGA_QI_MANUAL 7
#define NMEA_GGA_QI_SIM 8

#define FULL_CIRCLE_DEG 360.0f

/* Convert millimeters to meters */
#define MM2M(x) (x / 1000.0)

/* Message heading scale factor */
#define MSG_HEADING_SCALE_FACTOR 1000.0

#define MS2KNOTS(x, y, z) (sqrt((x) * (x) + (y) * (y) + (z) * (z)) * 1.94385)
#define MS2KMHR(x, y, z) \
  (sqrt((x) * (x) + (y) * (y) + (z) * (z)) * (3600.0 / 1000.0))

bool check_nmea_rate(u32 rate, u32 gps_tow_ms, int32_t soln_freq);
void send_gpgga(const struct sbp_nmea_state *state);
void send_gprmc(const struct sbp_nmea_state *state);
void send_gpvtg(const struct sbp_nmea_state *state);
void send_gpgll(const struct sbp_nmea_state *state);
void send_gpzda(const struct sbp_nmea_state *state);
void send_gsa(const struct sbp_nmea_state *state);
char get_nmea_status(u8 flags);
char get_nmea_mode_indicator(u8 flags);
u8 get_nmea_quality_indicator(u8 flags);
#endif /* SWIFTNAV_NMEA_H */
